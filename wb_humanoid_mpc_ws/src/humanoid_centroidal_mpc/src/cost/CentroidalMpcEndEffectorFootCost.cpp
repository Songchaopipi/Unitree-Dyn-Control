/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

// 自动中文注释：src/humanoid_centroidal_mpc/src/cost/CentroidalMpcEndEffectorFootCost.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_centroidal_mpc/cost/CentroidalMpcEndEffectorFootCost.h"

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalMpcEndEffectorFootCost::CentroidalMpcEndEffectorFootCost(const SwitchedModelReferenceManager& referenceManager,
                                                                   EndEffectorKinematicsWeights weights,  // 变量说明：weights 表示代价/任务权重。
                                                                   const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                                   const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,  // 变量说明：mpcRobotModelAD 表示自动微分版本的 MPC 机器人模型。
                                                                   size_t contactIndex,  // 变量说明：contactIndex 表示左右脚接触点编号。
                                                                   std::string costName,  // 变量说明：costName 表示自动微分代价库名称。
                                                                   const ModelSettings& modelSettings)
    // 函数说明：初始化自动微分 Gauss-Newton 代价对象，注册状态/输入/参数维度和 CppAD 动态库。
    : StateInputCostGaussNewtonAd(),
      referenceManagerPtr_(&referenceManager),
      sqrtWeights_(weights.toVector().cwiseSqrt()),
      frameID_(pinocchioInterface.getModel().getFrameId(modelSettings.contactNames[contactIndex])),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelAdPtr_(mpcRobotModelAD.clone()),
      contactIndex_(contactIndex) {
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), 25, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd);
  std::cout << "Frame ID: " << frameID_ << std::endl;
  std::cout << "Initialized CentroidalMpcEndEffectorFootCost with weights: " << weights.toVector().transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalMpcEndEffectorFootCost::CentroidalMpcEndEffectorFootCost(const CentroidalMpcEndEffectorFootCost& other)
    // 函数说明：初始化自动微分 Gauss-Newton 代价对象，注册状态/输入/参数维度和 CppAD 动态库。
    : StateInputCostGaussNewtonAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      sqrtWeights_(other.sqrtWeights_),
      frameID_(other.frameID_),
      contactIndex_(other.contactIndex_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelAdPtr_(other.mpcRobotModelAdPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t CentroidalMpcEndEffectorFootCost::costVectorFunction(ad_scalar_t time,  // 变量说明：CentroidalMpcEndEffectorFootCost 表示质心动力学、MPC、end、effector、足端、代价。
                                                                 const ad_vector_t& state,  // 变量说明：state 表示状态。
                                                                 const ad_vector_t& input,  // 变量说明：input 表示输入。
                                                                 const ad_vector_t& parameters) {  // 变量说明：parameters 表示传给自动微分代价的参数向量。
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;  // 变量说明：rf 表示足端速度/Jacobian 使用的 LOCAL_WORLD_ALIGNED 参考系。

  const PlanarEndEffectorKinematicsPlanarReference<ad_scalar_t> reference(parameters.head(12));
  const ad_vector_t sqrtWeightParams = parameters.segment(12, 12);  // EndEffectorKinematicsWeights vector element
  const ad_scalar_t impactProximityScaler = parameters[24];  // 变量说明：impactProximityScaler 表示impact、proximity、scaler。

  const auto& model = pinocchioInterfaceCppAd_.getModel();  // 变量说明：model 表示模型。
  auto& data = pinocchioInterfaceCppAd_.getData();  // 变量说明：data 表示数据。

  const ad_vector_t q = mpcRobotModelAdPtr_->getGeneralizedCoordinates(state);  // 变量说明：q 表示广义坐标/关节位置。
  const ad_vector_t v = mpcRobotModelAdPtr_->getGeneralizedVelocities(state, input);  // 变量说明：v 表示由状态和输入恢复出的 Pinocchio 广义速度。
  pinocchio::forwardKinematics(model, data, q, v);
  auto frameData = pinocchio::updateFramePlacement(model, data, frameID_);  // 变量说明：frameData 表示坐标系/帧、数据。

  // auto oMf = data.oMf;
  ad_vector_t position = frameData.translation();  // 变量说明：position 表示位置。
  ad_vector_t linearVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).linear();  // 变量说明：linearVelocity 表示linear、速度。
  ad_matrix3_t orientation = frameData.rotation();  // 变量说明：orientation 表示姿态。
  ad_vector_t angularVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).angular();  // 变量说明：angularVelocity 表示angular、速度。

  ad_vector_t errors(12);
  errors << (position - reference.getPosition()), rotationMatrixDistanceToPlane<ad_scalar_t>(orientation, reference.getPlaneNormal()),
      (linearVelocity - reference.getLinearVelocity()) * impactProximityScaler, (angularVelocity - reference.getAngularVelocity());

  return errors.cwiseProduct(sqrtWeightParams);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t CentroidalMpcEndEffectorFootCost::getParameters(scalar_t time,  // 变量说明：CentroidalMpcEndEffectorFootCost 表示质心动力学、MPC、end、effector、足端、代价。
                                                         const TargetTrajectories& targetTrajectories,  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
                                                         const PreComputation& preComputation) const {  // 变量说明：preComputation 表示 OCS2 预计算缓存，本函数当前不直接使用。
  // Interpolate reference
  const vector_t xRef = targetTrajectories.getDesiredState(time);  // 变量说明：xRef 表示当前时间插值得到的目标状态。
  const vector_t uRef = targetTrajectories.getDesiredInput(time);  // 变量说明：uRef 表示当前时间插值得到的目标输入。

  const scalar_t impactProximityScaler = referenceManagerPtr_->getSwingTrajectoryPlanner()->getImpactProximityFactor(contactIndex_, time);  // 变量说明：impactProximityScaler 表示impact、proximity、scaler。

  // TODO Update this reference for non flat ground in the future
  vector_t parameters(25);
  parameters.head(3) = vector3_t(0.0, 0.0, 0.0);        // Reference position
  parameters.segment(3, 3) = vector3_t(0.0, 0.0, 1.0);  // Ground plane normal
  parameters.segment(6, 3) = vector3_t(0.0, 0.0, 0.0);  // Reference linear velocity
  parameters.segment(9, 3) = vector3_t(0.0, 0.0, 0.0);  // Reference angular velocity
  parameters.segment(12, 12) = sqrtWeights_;            // EndEffectorKinematicsWeights vector element

  parameters[24] = impactProximityScaler;

  return parameters;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
}  // namespace ocs2::humanoid

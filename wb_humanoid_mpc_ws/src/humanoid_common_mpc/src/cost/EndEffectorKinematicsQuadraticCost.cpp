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

// 自动中文注释：src/humanoid_common_mpc/src/cost/EndEffectorKinematicsQuadraticCost.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/cost/EndEffectorKinematicsQuadraticCost.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorKinematicsQuadraticCost::EndEffectorKinematicsQuadraticCost(EndEffectorKinematicsWeights weights,
                                                                       const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                                       const EndEffectorKinematics<scalar_t>& endEffectorKinematics,  // 变量说明：endEffectorKinematics 表示end、effector、kinematics。
                                                                       const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,  // 变量说明：mpcRobotModelAD 表示自动微分版本的 MPC 机器人模型。
                                                                       std::string endEffectorName,  // 变量说明：endEffectorName 表示end、effector、名称。
                                                                       const ModelSettings& modelSettings)
    // 函数说明：初始化自动微分 Gauss-Newton 代价对象，注册状态/输入/参数维度和 CppAD 动态库。
    : StateInputCostGaussNewtonAd(),
      sqrtWeights_(weights.toVector().cwiseSqrt()),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelADPtr(mpcRobotModelAD.clone()) {
  std::cout << "Initialized EndEffectorKinematicsQuadraticCost with weights: " << weights.toVector().transpose() << std::endl;
  std::cout << "Frame name: " << endEffectorName << std::endl;
  frameID_ = pinocchioInterface.getModel().getFrameId(endEffectorName);
  std::cout << "Frame ID: " << frameID_ << std::endl;
  std::cout << "State dim: " << mpcRobotModelADPtr->getStateDim() << std::endl;
  std::cout << "Input dim: " << mpcRobotModelADPtr->getInputDim() << std::endl;
  std::cout << "Parameters dim: " << n_parameters_ << std::endl;

  initialize(mpcRobotModelADPtr->getStateDim(), mpcRobotModelADPtr->getInputDim(), n_parameters_,
             endEffectorName + "_KinematicsQuadraticCost", modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd,
             modelSettings.verboseCppAd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorKinematicsQuadraticCost::EndEffectorKinematicsQuadraticCost(const EndEffectorKinematicsQuadraticCost& other)
    // 函数说明：初始化自动微分 Gauss-Newton 代价对象，注册状态/输入/参数维度和 CppAD 动态库。
    : StateInputCostGaussNewtonAd(other),
      sqrtWeights_(other.sqrtWeights_),
      n_parameters_(other.n_parameters_),
      frameID_(other.frameID_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      endEffectorKinematicsPtr_(other.endEffectorKinematicsPtr_->clone()),
      mpcRobotModelADPtr(other.mpcRobotModelADPtr->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t EndEffectorKinematicsQuadraticCost::getParameters(scalar_t time,  // 变量说明：EndEffectorKinematicsQuadraticCost 表示end、effector、kinematics、quadratic、代价。
                                                           const TargetTrajectories& targetTrajectories,  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
                                                           const PreComputation& preComputation) const {  // 变量说明：preComputation 表示 OCS2 预计算缓存，本函数当前不直接使用。
  // Interpolate reference
  const vector_t xRef = targetTrajectories.getDesiredState(time);  // 变量说明：xRef 表示当前时间插值得到的目标状态。
  const vector_t uRef = targetTrajectories.getDesiredInput(time);  // 变量说明：uRef 表示当前时间插值得到的目标输入。

  vector_t parameters(n_parameters_);
  parameters << getReferenceCostElement(xRef, uRef, *endEffectorKinematicsPtr_).getValues(), sqrtWeights_;
  return parameters;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorKinematicsCostElement<scalar_t> EndEffectorKinematicsQuadraticCost::getReferenceCostElement(
    const vector_t& state, const vector_t& input, const EndEffectorKinematics<scalar_t>& endEffectorKinematics) {  // 变量说明：state 表示状态。
  EndEffectorKinematicsCostElement<scalar_t> costElement;  // 变量说明：costElement 表示代价、element。
  costElement.setPosition(endEffectorKinematics.getPosition(state)[0]);
  costElement.setOrientation(endEffectorKinematics.getOrientation(state)[0]);
  costElement.setLinearVelocity(endEffectorKinematics.getVelocity(state, input)[0]);
  costElement.setAngularVelocity(endEffectorKinematics.getAngularVelocity(state, input)[0]);
  return costElement;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t EndEffectorKinematicsQuadraticCost::costVectorFunction(ad_scalar_t time,  // 变量说明：EndEffectorKinematicsQuadraticCost 表示end、effector、kinematics、quadratic、代价。
                                                                   const ad_vector_t& state,  // 变量说明：state 表示状态。
                                                                   const ad_vector_t& input,  // 变量说明：input 表示输入。
                                                                   const ad_vector_t& parameters) {  // 变量说明：parameters 表示传给自动微分代价的参数向量。
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;  // 变量说明：rf 表示足端速度/Jacobian 使用的 LOCAL_WORLD_ALIGNED 参考系。

  const auto& model = pinocchioInterfaceCppAd_.getModel();  // 变量说明：model 表示模型。
  auto& data = pinocchioInterfaceCppAd_.getData();  // 变量说明：data 表示数据。

  const ad_vector_t q = mpcRobotModelADPtr->getGeneralizedCoordinates(state);  // 变量说明：q 表示广义坐标/关节位置。
  const ad_vector_t v = mpcRobotModelADPtr->getGeneralizedVelocities(state, input);  // 变量说明：v 表示由状态和输入恢复出的 Pinocchio 广义速度。
  pinocchio::forwardKinematics(model, data, q, v);
  auto frameData = pinocchio::updateFramePlacement(model, data, frameID_);  // 变量说明：frameData 表示坐标系/帧、数据。

  // auto oMf = data.oMf;
  ad_vector_t position = frameData.translation();  // 变量说明：position 表示位置。
  ad_vector_t linearVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).linear();  // 变量说明：linearVelocity 表示linear、速度。
  ad_quaternion_t orientation = matrixToQuaternion(frameData.rotation());  // 变量说明：orientation 表示姿态。
  ad_vector_t angularVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).angular();  // 变量说明：angularVelocity 表示angular、速度。
  ad_vector_t taskSpaceVec(13);
  taskSpaceVec << position, orientation.coeffs(), linearVelocity, angularVelocity;
  // ad_vector_t errors = ad_vector_t::Zero(12);
  ad_vector_t errors = computeTaskSpaceErrors(EndEffectorKinematicsCostElement<ad_scalar_t>(taskSpaceVec),  // 变量说明：errors 表示当前任务 residual/误差向量。
                                              EndEffectorKinematicsCostElement<ad_scalar_t>(parameters.head(13)));

  const ad_vector_t sqrtWeightParams = parameters.segment<12>(13);  // EndEffectorKinematicsWeights vector element

  return errors.cwiseProduct(sqrtWeightParams);
}

}  // namespace ocs2::humanoid

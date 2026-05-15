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

// 自动中文注释：src/humanoid_centroidal_mpc/src/cost/ICPCost.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_centroidal_mpc/cost/ICPCost.h"

#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <cmath>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ICPCost::ICPCost(const SwitchedModelReferenceManager& referenceManager,
                 vector2_t weights,  // 变量说明：weights 表示代价/任务权重。
                 const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                 const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,  // 变量说明：mpcRobotModelAD 表示自动微分版本的 MPC 机器人模型。
                 std::string costName,  // 变量说明：costName 表示自动微分代价库名称。
                 const ModelSettings& modelSettings)
    // 函数说明：初始化自动微分 Gauss-Newton 代价对象，注册状态/输入/参数维度和 CppAD 动态库。
    : StateInputCostGaussNewtonAd(),
      referenceManagerPtr_(&referenceManager),
      sqrtWeights_(weights.cwiseSqrt()),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelAdPtr_(mpcRobotModelAD.clone()) {
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), 2, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd);
  std::cout << "Initialized ICPCost with weights: " << weights.transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ICPCost::ICPCost(const ICPCost& other)
    // 函数说明：初始化自动微分 Gauss-Newton 代价对象，注册状态/输入/参数维度和 CppAD 动态库。
    : StateInputCostGaussNewtonAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      sqrtWeights_(other.sqrtWeights_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelAdPtr_(other.mpcRobotModelAdPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t ICPCost::costVectorFunction(ad_scalar_t time,  // 变量说明：ICPCost 表示icpcost。
                                        const ad_vector_t& state,  // 变量说明：state 表示状态。
                                        const ad_vector_t& input,  // 变量说明：input 表示输入。
                                        const ad_vector_t& parameters) {  // 变量说明：parameters 表示传给自动微分代价的参数向量。
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;  // 变量说明：rf 表示足端速度/Jacobian 使用的 LOCAL_WORLD_ALIGNED 参考系。
  const ad_vector_t sqrtWeightParams = parameters.head(2);  // EndEffectorKinematicsWeights vector element

  const auto& model = pinocchioInterfaceCppAd_.getModel();  // 变量说明：model 表示模型。
  auto& data = pinocchioInterfaceCppAd_.getData();  // 变量说明：data 表示数据。
  scalar_t omega = std::sqrt(9.81 / 0.7);  // sqrt(g / z_0) This default com height should be added from the config.

  const ad_vector_t q = mpcRobotModelAdPtr_->getGeneralizedCoordinates(state);  // 变量说明：q 表示广义坐标/关节位置。
  pinocchio::centerOfMass(model, data, q, false);
  ad_vector2_t com = data.com[0].head(2);  // 变量说明：com 表示质心。

  // 函数说明：更新坐标系/帧、placements，连接当前模块的数据流和控制逻辑。
  pinocchio::updateFramePlacements(model, data);
  auto contactPositions = getContactPositions<ad_scalar_t>(pinocchioInterfaceCppAd_, *mpcRobotModelAdPtr_);  // 变量说明：contactPositions 表示接触、positions。
  ad_vector2_t desiredCOMPosition = (contactPositions[0] + contactPositions[1]).head(2) / ad_scalar_t(2.0);  // 变量说明：desiredCOMPosition 表示期望、composition。

  ad_vector_t com_vel(2);
  com_vel[0] = state[0];
  com_vel[1] = state[1];

  ad_vector_t capturePoint = com;  // 变量说明：capturePoint 表示capture、point。
  // ad_vector_t capturePoint = com + com_vel / ad_scalar_t(omega);
  ad_vector_t errors = desiredCOMPosition - capturePoint;  // 变量说明：errors 表示当前任务 residual/误差向量。

  return errors.cwiseProduct(sqrtWeightParams);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t ICPCost::getParameters(scalar_t time, const TargetTrajectories& targetTrajectories, const PreComputation& preComputation) const {  // 变量说明：ICPCost 表示icpcost。
  // TODO Update this reference for non flat ground in the future
  vector_t parameters = sqrtWeights_;  // EndEffectorKinematicsWeights vector element

  return parameters;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector2_t ICPCost::getWeights(const std::string& taskFile, const std::string prefix, bool verbose) {  // 变量说明：ICPCost 表示icpcost。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile, pt);

  // Load all weights
  scalar_t icpErrorWeight = 0;  // 变量说明：icpErrorWeight 表示icp、误差、权重。

  if (verbose) {
    std::cerr << "\n #### ICP Cost Weights: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, icpErrorWeight, prefix + "icpErrorWeight", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  vector2_t weights;  // 变量说明：weights 表示代价/任务权重。
  weights << icpErrorWeight, icpErrorWeight;

  return weights;
}
}  // namespace ocs2::humanoid

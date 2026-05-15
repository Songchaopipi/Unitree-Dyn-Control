/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

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

// 自动中文注释：src/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/cost/ExternalTorqueQuadraticCostAD.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ExternalTorqueQuadraticCostAD::ExternalTorqueQuadraticCostAD(size_t endEffectorIndex,
                                                             Config config,  // 变量说明：config 表示从配置文件读取出的参数集合。
                                                             const SwitchedModelReferenceManager& referenceManager,  // 变量说明：referenceManager 表示参考、manager。
                                                             const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                             const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,  // 变量说明：mpcRobotModelAD 表示自动微分版本的 MPC 机器人模型。
                                                             const ModelSettings& modelSettings)
    // 函数说明：初始化自动微分 Gauss-Newton 代价对象，注册状态/输入/参数维度和 CppAD 动态库。
    : StateInputCostGaussNewtonAd(),
      contactPointIndex_(endEffectorIndex),
      frameID_(pinocchioInterface.getModel().getFrameId(modelSettings.contactNames[endEffectorIndex])),
      n_parameters_(1 + config.weights.size()),
      sqrtWeights_(config.weights.cwiseSqrt()),
      activeJointNames_(config.activeJointNames),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelADPtr(mpcRobotModelAD.clone()) {
  assert(config.weights.size() == config.activeJointNames.size());
  std::cout << "Initialized ExternalTorqueQuadraticCostAD with weights: " << config.weights.cwiseSqrt() << std::endl;
  const std::string endEffectorName = modelSettings.contactNames[endEffectorIndex];  // 变量说明：endEffectorName 表示end、effector、名称。
  std::cout << "Frame name: " << endEffectorName << std::endl;

  std::cout << "Frame ID: " << frameID_ << std::endl;
  std::cout << "State dim: " << mpcRobotModelADPtr->getStateDim() << std::endl;
  std::cout << "Input dim: " << mpcRobotModelADPtr->getInputDim() << std::endl;
  std::cout << "Parameters dim: " << n_parameters_ << std::endl;

  initialize(mpcRobotModelADPtr->getStateDim(), mpcRobotModelADPtr->getInputDim(), n_parameters_,
             endEffectorName + "_ExternalTorqueQuadraticCost", modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd,
             modelSettings.verboseCppAd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ExternalTorqueQuadraticCostAD::ExternalTorqueQuadraticCostAD(const ExternalTorqueQuadraticCostAD& other)
    // 函数说明：初始化自动微分 Gauss-Newton 代价对象，注册状态/输入/参数维度和 CppAD 动态库。
    : StateInputCostGaussNewtonAd(other),
      contactPointIndex_(other.contactPointIndex_),
      frameID_(other.frameID_),
      n_parameters_(other.n_parameters_),
      sqrtWeights_(other.sqrtWeights_),
      activeJointNames_(other.activeJointNames_),
      referenceManagerPtr_(other.referenceManagerPtr_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelADPtr(other.mpcRobotModelADPtr->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool ExternalTorqueQuadraticCostAD::isActive(scalar_t time) const {  // 变量说明：ExternalTorqueQuadraticCostAD 表示external、力矩、quadratic、代价、自动微分。
  if (!isActive_) return false;
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t ExternalTorqueQuadraticCostAD::getParameters(scalar_t time,  // 变量说明：ExternalTorqueQuadraticCostAD 表示external、力矩、quadratic、代价、自动微分。
                                                      const TargetTrajectories& targetTrajectories,  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
                                                      const PreComputation& preComputation) const {  // 变量说明：preComputation 表示 OCS2 预计算缓存，本函数当前不直接使用。
  vector_t params(n_parameters_);
  const scalar_t impactProximityScaler = referenceManagerPtr_->getSwingTrajectoryPlanner()->getImpactProximityFactor(  // 变量说明：impactProximityScaler 表示impact、proximity、scaler。
      (1 - contactPointIndex_), time);  // Get impactproximity scaler from swing foot.
  params << sqrtWeights_, impactProximityScaler;
  return params;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t ExternalTorqueQuadraticCostAD::costVectorFunction(ad_scalar_t time,  // 变量说明：ExternalTorqueQuadraticCostAD 表示external、力矩、quadratic、代价、自动微分。
                                                              const ad_vector_t& state,  // 变量说明：state 表示状态。
                                                              const ad_vector_t& input,  // 变量说明：input 表示输入。
                                                              const ad_vector_t& parameters) {  // 变量说明：parameters 表示传给自动微分代价的参数向量。
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;  // 变量说明：rf 表示足端速度/Jacobian 使用的 LOCAL_WORLD_ALIGNED 参考系。

  const auto& model = pinocchioInterfaceCppAd_.getModel();  // 变量说明：model 表示模型。
  auto& data = pinocchioInterfaceCppAd_.getData();  // 变量说明：data 表示数据。

  const ad_vector_t q = mpcRobotModelADPtr->getGeneralizedCoordinates(state);  // 变量说明：q 表示广义坐标/关节位置。
  ad_matrix_t J_ee = ad_matrix_t::Zero(6, mpcRobotModelADPtr->getGenCoordinatesDim());  // 变量说明：J_ee 表示j、ee。
  // 函数说明：计算坐标系/帧、雅可比，连接当前模块的数据流和控制逻辑。
  pinocchio::computeFrameJacobian(model, data, q, frameID_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_ee);

  ad_vector_t tauExt = J_ee.transpose() * mpcRobotModelADPtr->getContactWrench(input, contactPointIndex_);  // 变量说明：tauExt 表示tau、ext。

  ad_vector_t tauExtActive = ad_vector_t::Zero(sqrtWeights_.size());  // 变量说明：tauExtActive 表示tau、ext、主动关节。
  for (size_t i = 0; i < sqrtWeights_.size(); i++) {
    tauExtActive[i] = tauExt[6 + mpcRobotModelADPtr->getJointIndex(activeJointNames_[i])];
  }

  const ad_vector_t sqrtWeightsAD = parameters.head(sqrtWeights_.size());  // 变量说明：sqrtWeightsAD 表示sqrt、weights、自动微分。
  const ad_scalar_t midSwingScaler =  // 变量说明：midSwingScaler 表示mid、摆动脚、scaler。
      1 - parameters[sqrtWeights_.size()];  // large when in the middle of the swing phase, close when close to impact.

  return tauExtActive.cwiseProduct(sqrtWeightsAD) * midSwingScaler;  // multiply with weights
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ExternalTorqueQuadraticCostAD::Config ExternalTorqueQuadraticCostAD::loadConfigFromFile(const std::string& filename,  // 变量说明：ExternalTorqueQuadraticCostAD 表示external、力矩、quadratic、代价、自动微分。
                                                                                        const std::string& fieldname,  // 变量说明：fieldname 表示配置文件中对应代价/约束块的字段名。
                                                                                        bool verbose) {  // 变量说明：verbose 表示是否打印配置和初始化信息。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(filename, pt);

  Config config;  // 变量说明：config 表示从配置文件读取出的参数集合。

  if (verbose) {
    std::cerr << "\n #### External Torque Quadratic Cost Weights: ";
    std::cerr << "Loading weigths from: " << fieldname;
    std::cerr << "\n #### =============================================================================\n";
  }
  // 函数说明：加载std、向量，连接当前模块的数据流和控制逻辑。
  loadData::loadStdVector(filename, fieldname + "activeJointNames", config.activeJointNames, verbose);

  vector_t weights(config.activeJointNames.size());
  // 函数说明：加载eigen、矩阵，连接当前模块的数据流和控制逻辑。
  loadData::loadEigenMatrix(filename, fieldname + "weights", weights);

  if (verbose) {
    std::cerr << "weights: " << weights.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  config.weights = weights;
  assert(config.weights.size() == config.activeJointNames.size());

  return config;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

}  // namespace ocs2::humanoid

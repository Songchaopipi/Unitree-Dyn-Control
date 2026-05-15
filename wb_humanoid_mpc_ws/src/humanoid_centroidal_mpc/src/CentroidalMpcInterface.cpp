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

// 自动中文注释：src/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include <iostream>
#include <string>

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_centroidal_mpc/CentroidalMpcInterface.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <humanoid_common_mpc/HumanoidCostConstraintFactory.h>
#include <humanoid_common_mpc/HumanoidPreComputation.h>
#include <humanoid_common_mpc/constraint/EndEffectorKinematicsTwistConstraint.h>
#include <humanoid_common_mpc/cost/EndEffectorKinematicsQuadraticCost.h>
#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>

#include "humanoid_centroidal_mpc/constraint/JointMimicKinematicConstraint.h"
#include "humanoid_centroidal_mpc/constraint/NormalVelocityConstraintCppAd.h"
#include "humanoid_centroidal_mpc/constraint/ZeroVelocityConstraintCppAd.h"
#include "humanoid_centroidal_mpc/cost/CentroidalMpcEndEffectorFootCost.h"
#include "humanoid_centroidal_mpc/cost/ICPCost.h"
#include "humanoid_centroidal_mpc/dynamics/CentroidalDynamicsAD.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalMpcInterface::CentroidalMpcInterface(const std::string& taskFile,
                                               const std::string& urdfFile,  // 变量说明：urdfFile 表示urdf、file。
                                               const std::string& referenceFile,  // 变量说明：referenceFile 表示参考、file。
                                               bool setupOCP)
    // 函数说明：处理task、file，连接当前模块的数据流和控制逻辑。
    : taskFile_(taskFile),
      urdfFile_(urdfFile),
      referenceFile_(referenceFile),
      modelSettings_(taskFile, urdfFile, "centroidal_mpc_", "true") {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[CentroidalMpcInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[CentroidalMpcInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[CentroidalMpcInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[CentroidalMpcInterface] URDF file not found: " + urdfFilePath.string());
  }
  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath)) {
    std::cerr << "[CentroidalMpcInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[CentroidalMpcInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(taskFile, "interface.verbose", verbose_);

  // load setting from loading file
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose_);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose_);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose_);
  sqpSettings_ = sqp::loadSettings(taskFile, "multiple_shooting", verbose_);

  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createCustomPinocchioInterface(taskFile, urdfFile, modelSettings_, false)));

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings_.contactNames3DoF,
      modelSettings_.contactNames6DoF);

  std::cout << "centroidalModelInfo_.numSixDofContacts: " << centroidalModelInfo_.numSixDofContacts << std::endl;
  for (int i = 0; i < centroidalModelInfo_.numSixDofContacts; i++) {
    std::cout << "frameIndices: " << centroidalModelInfo_.endEffectorFrameIndices[i] << std::endl;
  }

  // Setup Centroidal State Input Mapping
  mpcRobotModelPtr_.reset(new CentroidalMpcRobotModel<scalar_t>(modelSettings_, *pinocchioInterfacePtr_, centroidalModelInfo_));
  mpcRobotModelADPtr_.reset(
      new CentroidalMpcRobotModel<ad_scalar_t>(modelSettings_, (*pinocchioInterfacePtr_).toCppAd(), centroidalModelInfo_.toCppAd()));  // 变量说明：CentroidalMpcRobotModel 表示质心动力学、MPC、机器人、模型。

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose_), N_CONTACTS));

  referenceManagerPtr_ =
      std::make_shared<SwitchedModelReferenceManager>(GaitSchedule::loadGaitSchedule(referenceFile, modelSettings_, verbose_),
                                                      std::move(swingTrajectoryPlanner), *pinocchioInterfacePtr_, *mpcRobotModelPtr_);
  referenceManagerPtr_->setArmSwingReferenceActive(true);

  // initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  // 函数说明：加载eigen、矩阵，连接当前模块的数据流和控制逻辑。
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  if (setupOCP) {
    setupOptimalControlProblem();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void CentroidalMpcInterface::setupOptimalControlProblem() {  // 变量说明：CentroidalMpcInterface 表示质心动力学、MPC、interface。
  HumanoidCostConstraintFactory factory =  // 变量说明：factory 表示统一创建代价、约束和预计算模块的工厂对象。
      HumanoidCostConstraintFactory(taskFile_, referenceFile_, *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelPtr_,
                                    *mpcRobotModelADPtr_, modelSettings_, verbose_);

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;  // 变量说明：dynamicsPtr 表示dynamics、指针。
  const std::string modelName = "dynamics";  // 变量说明：modelName 表示模型、名称。
  dynamicsPtr.reset(new CentroidalDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelName, modelSettings_));

  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  problemPtr_->costPtr->add("stateInputQuadraticCost", factory.getStateInputQuadraticCost());
  problemPtr_->finalCostPtr->add("terminalCost", factory.getTerminalCost());

  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;  // 变量说明：eeKinematicsPtr 表示ee、kinematics、指针。

  const auto infoCppAd = centroidalModelInfo_.toCppAd();  // 变量说明：infoCppAd 表示模型信息、cpp、自动微分。
  const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);

  auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {  // 变量说明：velocityUpdateCallback 表示速度、update、callback。
    const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);  // 变量说明：q 表示广义坐标/关节位置。
    updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
  };

  addTaskSpaceKinematicsCosts(pinocchioMappingCppAd, velocityUpdateCallback);

  const vector2_t icpWeights = ICPCost::getWeights(taskFile_, "icp_cost_weights.", verbose_);  // 变量说明：icpWeights 表示icp、weights。
  problemPtr_->costPtr->add(
      "icp_Cost", std::unique_ptr<StateInputCost>(new ICPCost(*referenceManagerPtr_, std::move(icpWeights), *pinocchioInterfacePtr_,
                                                              *mpcRobotModelADPtr_, "icp_Cost", modelSettings_)));

  // Constraints
  problemPtr_->stateSoftConstraintPtr->add("jointLimits", factory.getJointLimitsConstraint());
  problemPtr_->stateSoftConstraintPtr->add("FootCollisionSoftConstraint", factory.getFootCollisionConstraint());

  // Constraint terms
  EndEffectorKinematicsWeights footTrackingCostWeights =  // 变量说明：footTrackingCostWeights 表示足端、tracking、代价、weights。
      // 函数说明：获取weights，连接当前模块的数据流和控制逻辑。
      EndEffectorKinematicsWeights::getWeights(taskFile_, "task_space_foot_cost_weights.", verbose_);

  // check for mimic joints
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile_, pt);
  bool hasMimicJoints = loadData::containsPtreeValueFind(pt, "mimicJoints");  // 变量说明：hasMimicJoints 表示has、mimic、joints。

  for (size_t i = 0; i < N_CONTACTS; i++) {
    const std::string& footName = modelSettings_.contactNames[i];  // 变量说明：footName 表示足端、名称。

    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
                                                                  centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                  velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
                                                                  modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

    problemPtr_->softConstraintPtr->add(footName + "_frictionForceCone", factory.getFrictionForceConeConstraint(i));
    problemPtr_->softConstraintPtr->add(footName + "_contactMomentXY",
                                        factory.getContactMomentXYConstraint(i, footName + "_contact_moment_XY_constraint"));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroWrench", factory.getZeroWrenchConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getStanceFootConstraint(*eeKinematicsPtr, i));
    problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity", getNormalVelocityConstraint(*eeKinematicsPtr, i));
    if (hasMimicJoints) {
      problemPtr_->equalityConstraintPtr->add(footName + "_kneeJointMimic", getJointMimicConstraint(i));
    }

    std::string footTrackingCostName = footName + "_TaskSpaceKinematicsCost";  // 变量说明：footTrackingCostName 表示足端、tracking、代价、名称。

    problemPtr_->costPtr->add(footTrackingCostName, std::unique_ptr<StateInputCost>(new CentroidalMpcEndEffectorFootCost(
                                                        *referenceManagerPtr_, footTrackingCostWeights, *pinocchioInterfacePtr_,
                                                        *mpcRobotModelADPtr_, i, footTrackingCostName, modelSettings_)));
    problemPtr_->costPtr->add(footName + "_ExternalTorqueQuadraticCost", factory.getExternalTorqueQuadraticCost(i));
  }

  // Pre-computation
  problemPtr_->preComputationPtr.reset(
      new HumanoidPreComputation(*pinocchioInterfacePtr_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), *mpcRobotModelPtr_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extendNormalizedMomentum = true;  // 变量说明：extendNormalizedMomentum 表示extend、normalized、momentum。
  initializerPtr_.reset(
      new CentroidalWeightCompInitializer(centroidalModelInfo_, *referenceManagerPtr_, *mpcRobotModelPtr_, extendNormalizedMomentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputConstraint> CentroidalMpcInterface::getStanceFootConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,  // 变量说明：CentroidalMpcInterface 表示质心动力学、MPC、interface。
                                                                                      size_t contactPointIndex) {  // 变量说明：contactPointIndex 表示接触、point、索引。
  auto eeZeroVelConConfig = [](scalar_t positionErrorGain, scalar_t orientationErrorGain) {  // 变量说明：eeZeroVelConConfig 表示ee、zero、速度、con、config。
    EndEffectorKinematicsTwistConstraint::Config config;  // 变量说明：config 表示从配置文件读取出的参数集合。
    config.b.setZero(6);
    config.Ax.setZero(6, 6);
    config.Av.setIdentity(6, 6);
    if (!numerics::almost_eq(positionErrorGain, 0.0)) {
      config.Ax(2, 2) = positionErrorGain;
    }
    if (!numerics::almost_eq(orientationErrorGain, 0.0)) {
      config.Ax.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * orientationErrorGain;
    }

    return config;
  };

  return std::unique_ptr<StateInputConstraint>(
      new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                      eeZeroVelConConfig(modelSettings_.footConstraintConfig.positionErrorGain_z,
                                                         modelSettings_.footConstraintConfig.orientationErrorGain)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> CentroidalMpcInterface::getNormalVelocityConstraint(
    const EndEffectorKinematics<scalar_t>& eeKinematics, size_t contactPointIndex) {  // 变量说明：eeKinematics 表示ee、kinematics。
  return std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> CentroidalMpcInterface::getJointMimicConstraint(size_t mimicIndex) {  // 变量说明：CentroidalMpcInterface 表示质心动力学、MPC、interface。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile_, pt);
  std::string prefix;  // 变量说明：prefix 表示读取 task.info 时使用的字段前缀。
  if (mimicIndex == 0) {
    prefix = "mimicJoints.left_knee.";
  } else if (mimicIndex == 1) {
    prefix = "mimicJoints.right_knee.";
  } else {
    throw std::runtime_error("No mimic joint for index: " + std::to_string(mimicIndex));
  }

  std::string parentJointName;  // 变量说明：parentJointName 表示parent、关节、名称。
  std::string childJointName;  // 变量说明：childJointName 表示child、关节、名称。
  scalar_t multiplier;  // q_child = multiplier* q_parent
  scalar_t positionGain;  // 变量说明：positionGain 表示位置、增益。

  if (verbose_) {
    std::cerr << "\n #### Joint Mimic Kinematic Constraint Config: ";
    std::cerr << "\n #### "
                 "============================================================="
                 "================\n";
  }
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, parentJointName, prefix + "parentJointName", verbose_);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, childJointName, prefix + "childJointName", verbose_);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, multiplier, prefix + "multiplier", verbose_);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, positionGain, prefix + "positionGain", verbose_);
  if (verbose_) {
    std::cerr << " #### "
                 "============================================================="
                 "================\n";
  }

  JointMimicKinematicConstraint::Config config(*mpcRobotModelPtr_, parentJointName, childJointName, multiplier, positionGain);

  return std::unique_ptr<StateInputConstraint>(new JointMimicKinematicConstraint(*mpcRobotModelPtr_, config));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalMpcInterface::addTaskSpaceKinematicsCosts(
    const CentroidalModelPinocchioMappingCppAd& pinocchioMappingCppAd,  // 变量说明：pinocchioMappingCppAd 表示Pinocchio、mapping、cpp、自动微分。
    const PinocchioEndEffectorKinematicsCppAd::update_pinocchio_interface_callback& velocityUpdateCallback) {  // 变量说明：velocityUpdateCallback 表示速度、update、callback。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile_, pt);

  boost::property_tree::ptree task_space_costs_pt = pt.get_child("task_space_costs");  // 变量说明：task_space_costs_pt 表示task、space、costs、pt。

  for (auto& task_space_cost : task_space_costs_pt) {
    std::string costName = task_space_cost.first;  // 变量说明：costName 表示自动微分代价库名称。
    std::string linkName;  // 变量说明：linkName 表示link、名称。

    // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
    loadData::loadPtreeValue(task_space_costs_pt, linkName, costName + ".link_name", verbose_);

    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;  // 变量说明：eeKinematicsPtr 表示ee、kinematics、指针。

    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {linkName},
                                                                  centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                  velocityUpdateCallback, linkName, modelSettings_.modelFolderCppAd,
                                                                  modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

    EndEffectorKinematicsWeights weights =  // 变量说明：weights 表示代价/任务权重。
        // 函数说明：获取weights，连接当前模块的数据流和控制逻辑。
        EndEffectorKinematicsWeights::getWeights(taskFile_, "task_space_costs." + costName + ".weights.", verbose_);

    std::unique_ptr<StateInputCost> cost = std::make_unique<EndEffectorKinematicsQuadraticCost>(  // 变量说明：cost 表示代价。
        weights, *pinocchioInterfacePtr_, *eeKinematicsPtr, *mpcRobotModelADPtr_, linkName, modelSettings_);

    problemPtr_->costPtr->add(costName + "_TaskSpaceKinematicsCost", std::move(cost));

    std::cout << "Initialized Task Space Kinematics Cost for link: " << linkName << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getCostNames() const {  // 变量说明：CentroidalMpcInterface 表示质心动力学、MPC、interface。
  std::vector<std::string> costNames;  // 变量说明：costNames 表示代价、names。
  for (const auto& [costName, index] : problemPtr_->costPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getTerminalCostNames() const {  // 变量说明：CentroidalMpcInterface 表示质心动力学、MPC、interface。
  std::vector<std::string> terminalCostNames;  // 变量说明：terminalCostNames 表示terminal、代价、names。
  for (const auto& [costName, index] : problemPtr_->finalCostPtr->getTermNameMap()) {
    terminalCostNames.emplace_back(costName);
  }
  return terminalCostNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getStateSoftConstraintNames() const {  // 变量说明：CentroidalMpcInterface 表示质心动力学、MPC、interface。
  std::vector<std::string> costNames;  // 变量说明：costNames 表示代价、names。
  for (const auto& [costName, index] : problemPtr_->stateSoftConstraintPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getSoftConstraintNames() const {  // 变量说明：CentroidalMpcInterface 表示质心动力学、MPC、interface。
  std::vector<std::string> costNames;  // 变量说明：costNames 表示代价、names。
  for (const auto& [costName, index] : problemPtr_->softConstraintPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> CentroidalMpcInterface::getEqualityConstraintNames() const {  // 变量说明：CentroidalMpcInterface 表示质心动力学、MPC、interface。
  std::vector<std::string> costNames;  // 变量说明：costNames 表示代价、names。
  for (const auto& [costName, index] : problemPtr_->equalityConstraintPtr->getTermNameMap()) {
    costNames.emplace_back(costName);
  }
  return costNames;
}

}  // namespace ocs2::humanoid

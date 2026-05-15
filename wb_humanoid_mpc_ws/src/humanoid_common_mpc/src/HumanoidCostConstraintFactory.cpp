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

// 自动中文注释：src/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/HumanoidCostConstraintFactory.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/penalties/penalties/PieceWisePolynomialBarrierPenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include <humanoid_common_mpc/constraint/FrictionForceConeConstraint.h>
#include <humanoid_common_mpc/constraint/ZeroWrenchConstraint.h>
#include <humanoid_common_mpc/contact/ContactRectangle.h>
#include <humanoid_common_mpc/cost/StateInputQuadraticCost.h>
#include <humanoid_common_mpc/pinocchio_model/pinocchioUtils.h>
#include "humanoid_common_mpc/constraint/ContactMomentXYConstraintCppAd.h"
#include "humanoid_common_mpc/constraint/FootCollisionConstraint.h"
#include "humanoid_common_mpc/constraint/JointLimitsSoftConstraint.h"
#include "humanoid_common_mpc/cost/ExternalTorqueQuadraticCostAD.h"

namespace ocs2::humanoid {

// 函数说明：处理humanoid、代价、约束、factory，连接当前模块的数据流和控制逻辑。
HumanoidCostConstraintFactory::HumanoidCostConstraintFactory(const std::string& taskFile,
                                                             const std::string& referenceFile,  // 变量说明：referenceFile 表示参考、file。
                                                             const SwitchedModelReferenceManager& referenceManager,  // 变量说明：referenceManager 表示参考、manager。
                                                             const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                             const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                                             const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,  // 变量说明：mpcRobotModelAD 表示自动微分版本的 MPC 机器人模型。
                                                             const ModelSettings& modelSettings,  // 变量说明：modelSettings 表示机器人关节和接触配置。
                                                             bool verbose)
    // 函数说明：处理task、file，连接当前模块的数据流和控制逻辑。
    : taskFile_(taskFile),
      referenceFile_(referenceFile),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfacePtr_(&pinocchioInterface),
      mpcRobotModelPtr_(&mpcRobotModel),
      mpcRobotModelADPtr_(&mpcRobotModelAD),
      modelSettings_(modelSettings),
      verbose_(verbose) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getStateInputQuadraticCost() const {  // 变量说明：HumanoidCostConstraintFactory 表示humanoid、代价、约束、factory。
  matrix_t Q(mpcRobotModelADPtr_->getStateDim(), mpcRobotModelADPtr_->getStateDim());
  // 函数说明：加载eigen、矩阵，连接当前模块的数据流和控制逻辑。
  loadData::loadEigenMatrix(taskFile_, "Q", Q);
  matrix_t R(mpcRobotModelADPtr_->getInputDim(), mpcRobotModelADPtr_->getInputDim());
  // 函数说明：加载eigen、矩阵，连接当前模块的数据流和控制逻辑。
  loadData::loadEigenMatrix(taskFile_, "R", R);

  if (verbose_) {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(
      new StateInputQuadraticCost(std::move(Q), std::move(R), *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getFootCollisionConstraint() const {  // 变量说明：HumanoidCostConstraintFactory 表示humanoid、代价、约束、factory。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile_, pt);
  const std::string prefix = "collision_constraint.";  // 变量说明：prefix 表示读取 task.info 时使用的字段前缀。

  FootCollisionConstraint::Config collisionConfig = FootCollisionConstraint::loadFootCollisionConstraintConfig(taskFile_, verbose_);  // 变量说明：collisionConfig 表示collision、config。
  PieceWisePolynomialBarrierPenalty::Config barrierPenaltyConfig;  // 变量说明：barrierPenaltyConfig 表示barrier、penalty、config。

  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose_);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose_);

  std::unique_ptr<PieceWisePolynomialBarrierPenalty> penalty(new PieceWisePolynomialBarrierPenalty(barrierPenaltyConfig));

  std::unique_ptr<FootCollisionConstraint> footCollisionConstraintPtr(
      new FootCollisionConstraint(*referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, std::move(collisionConfig),
                                  "FootCollisionConstraint", modelSettings_));

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(footCollisionConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getJointLimitsConstraint() const {  // 变量说明：HumanoidCostConstraintFactory 表示humanoid、代价、约束、factory。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile_, pt);
  const std::string prefix = "jointLimits.";  // 变量说明：prefix 表示读取 task.info 时使用的字段前缀。

  PieceWisePolynomialBarrierPenalty::Config barrierPenaltyConfig;  // 变量说明：barrierPenaltyConfig 表示barrier、penalty、config。

  if (verbose_) {
    std::cerr << "\n #### Joint Limit Barrier Penalty Config: ";
    std::cerr << "\n #### "
                 "============================================================="
                 "================\n";
  }
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose_);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose_);
  if (verbose_) {
    std::cerr << " #### "
                 "============================================================="
                 "================\n";
  }

  std::cout << "Initialized joint limits constraint with zero crossing cost " << barrierPenaltyConfig.getZeroCrossingValue() << "."
            << std::endl;

  std::pair<vector_t, vector_t> jointLimits = readPinocchioJointLimits(*pinocchioInterfacePtr_, mpcRobotModelPtr_->modelSettings);

  return std::unique_ptr<StateCost>(new JointLimitsSoftConstraint(jointLimits, barrierPenaltyConfig, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getContactMomentXYConstraint(size_t contactPointIndex,  // 变量说明：HumanoidCostConstraintFactory 表示humanoid、代价、约束、factory。
                                                                                            const std::string& name) const {  // 变量说明：name 表示名称。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile_, pt);
  const std::string prefix = "contacts.contactMomentXYSoftConstraint.";  // 变量说明：prefix 表示读取 task.info 时使用的字段前缀。

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;  // 变量说明：barrierPenaltyConfig 表示barrier、penalty、config。
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose_);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose_);

  std::unique_ptr<ContactMomentXYConstraintCppAd> contactMomentXYConstraintPtr(new ContactMomentXYConstraintCppAd(
      *referenceManagerPtr_, ContactRectangle::loadContactRectangle(taskFile_, mpcRobotModelPtr_->modelSettings, contactPointIndex),
      contactPointIndex, *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, name, modelSettings_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(contactMomentXYConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputConstraint> HumanoidCostConstraintFactory::getZeroWrenchConstraint(size_t contactPointIndex) const {  // 变量说明：HumanoidCostConstraintFactory 表示humanoid、代价、约束、factory。
  return std::unique_ptr<StateInputConstraint>(new ZeroWrenchConstraint(*referenceManagerPtr_, contactPointIndex, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getFrictionForceConeConstraint(size_t contactPointIndex) const {  // 变量说明：HumanoidCostConstraintFactory 表示humanoid、代价、约束、factory。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile_, pt);
  const std::string prefix = "contacts.frictionForceConeSoftConstraint.";  // 变量说明：prefix 表示读取 task.info 时使用的字段前缀。

  scalar_t frictionCoefficient = 1.0;  // 变量说明：frictionCoefficient 表示摩擦、coefficient。
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;  // 变量说明：barrierPenaltyConfig 表示barrier、penalty、config。
  if (verbose_) {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose_);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose_);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose_);
  if (verbose_) {
    std::cerr << " #### =============================================================================\n";
  }

  FrictionForceConeConstraint::Config frictionConeConConfig(frictionCoefficient);
  std::unique_ptr<FrictionForceConeConstraint> frictionForceConeConstraintPtr(
      new FrictionForceConeConstraint(*referenceManagerPtr_, std::move(frictionConeConConfig), contactPointIndex, *mpcRobotModelPtr_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(frictionForceConeConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getTerminalCost() const {  // 变量说明：HumanoidCostConstraintFactory 表示humanoid、代价、约束、factory。
  scalar_t terminalCostScaling;  // 变量说明：terminalCostScaling 表示terminal、代价、scaling。
  loadData::loadCppDataType<scalar_t>(taskFile_, "terminalCostScaling", terminalCostScaling);
  std::cout << "terminalCostScaling: " << terminalCostScaling << std::endl;

  matrix_t Qf(mpcRobotModelPtr_->getStateDim(), mpcRobotModelPtr_->getStateDim());
  // 函数说明：加载eigen、矩阵，连接当前模块的数据流和控制逻辑。
  loadData::loadEigenMatrix(taskFile_, "Q_final", Qf);
  Qf *= terminalCostScaling;
  if (verbose_) std::cerr << "Q_final:\n" << Qf << std::endl;
  return std::unique_ptr<StateCost>(new QuadraticStateCost(Qf));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getExternalTorqueQuadraticCost(size_t contactPointIndex) const {  // 变量说明：HumanoidCostConstraintFactory 表示humanoid、代价、约束、factory。
  std::string fieldName;  // 变量说明：fieldName 表示field、名称。
  if (contactPointIndex == 0) {
    fieldName = "left_leg_torque_cost.";
  }
  if (contactPointIndex == 1) {
    fieldName = "right_leg_torque_cost.";
  }
  ExternalTorqueQuadraticCostAD::Config config = ExternalTorqueQuadraticCostAD::loadConfigFromFile(taskFile_, fieldName, verbose_);  // 变量说明：config 表示从配置文件读取出的参数集合。
  return std::make_unique<ExternalTorqueQuadraticCostAD>(contactPointIndex, config, *referenceManagerPtr_, *pinocchioInterfacePtr_,
                                                         *mpcRobotModelADPtr_, modelSettings_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

}  // namespace ocs2::humanoid

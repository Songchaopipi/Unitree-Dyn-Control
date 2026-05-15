#include "humanoid_centroidal_mpc/constraint/NormalVelocityConstraintCppAd.h"
#include "humanoid_common_mpc/HumanoidPreComputation.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics,  // 变量说明：endEffectorKinematics 表示end、effector、kinematics。
                                                             size_t contactPointIndex)
    // 函数说明：处理状态、输入、约束，连接当前模块的数据流和控制逻辑。
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      eeLinearConstraintPtr_(new EndEffectorKinematicsLinearVelConstraint(endEffectorKinematics, 1)),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd& rhs)
    // 函数说明：处理状态、输入、约束，连接当前模块的数据流和控制逻辑。
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool NormalVelocityConstraintCppAd::isActive(scalar_t time) const {  // 变量说明：NormalVelocityConstraintCppAd 表示法向、速度、约束、cpp、自动微分。
  if (!isActive_) return false;
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t NormalVelocityConstraintCppAd::getValue(scalar_t time,  // 变量说明：NormalVelocityConstraintCppAd 表示法向、速度、约束、cpp、自动微分。
                                                 const vector_t& state,  // 变量说明：state 表示状态。
                                                 const vector_t& input,  // 变量说明：input 表示输入。
                                                 const PreComputation& preComp) const {  // 变量说明：preComp 表示pre、comp。
  const auto& humanoidPreComp = cast<HumanoidPreComputation>(preComp);  // 变量说明：humanoidPreComp 表示humanoid、pre、comp。
  eeLinearConstraintPtr_->configure(humanoidPreComp.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);
  return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation NormalVelocityConstraintCppAd::getLinearApproximation(scalar_t time,  // 变量说明：NormalVelocityConstraintCppAd 表示法向、速度、约束、cpp、自动微分。
                                                                                        const vector_t& state,  // 变量说明：state 表示状态。
                                                                                        const vector_t& input,  // 变量说明：input 表示输入。
                                                                                        const PreComputation& preComp) const {  // 变量说明：preComp 表示pre、comp。
  const auto& humanoidPreComp = cast<HumanoidPreComputation>(preComp);  // 变量说明：humanoidPreComp 表示humanoid、pre、comp。
  eeLinearConstraintPtr_->configure(humanoidPreComp.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);
  return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

}  // namespace ocs2::humanoid

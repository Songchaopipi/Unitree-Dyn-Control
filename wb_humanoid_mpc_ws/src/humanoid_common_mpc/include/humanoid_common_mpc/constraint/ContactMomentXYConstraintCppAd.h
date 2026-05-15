
#pragma once

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/contact/ContactRectangle.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

/**
 * Implements the constraint h(t,x,u) >= 0 to constrain the contact moment in the x-y plane.
 */

class ContactMomentXYConstraintCppAd final : public StateInputConstraintCppAd {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ContactMomentXYConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                 const ContactRectangle& contactRectangle,  // 变量说明：contactRectangle 表示接触、rectangle。
                                 size_t contactPointIndex,  // 变量说明：contactPointIndex 表示接触、point、索引。
                                 const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                 const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                 std::string costName,  // 变量说明：costName 表示自动微分代价库名称。
                                 const ModelSettings& modelSettings);  // 变量说明：modelSettings 表示机器人关节和接触配置。

  ~ContactMomentXYConstraintCppAd() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  ContactMomentXYConstraintCppAd* clone() const override { return new ContactMomentXYConstraintCppAd(*this); }

  // 函数说明：判断主动关节，连接当前模块的数据流和控制逻辑。
  bool isActive(scalar_t time) const override;
  // 函数说明：设置主动关节，连接当前模块的数据流和控制逻辑。
  void setActive(bool isActive) override { isActive_ = isActive; }
  // 函数说明：获取主动关节，连接当前模块的数据流和控制逻辑。
  bool getActive() const override { return isActive_; }
  // 函数说明：获取num、constraints，连接当前模块的数据流和控制逻辑。
  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; };

 private:
  ContactMomentXYConstraintCppAd(const ContactMomentXYConstraintCppAd& other);

  // 函数说明：处理约束、function，连接当前模块的数据流和控制逻辑。
  ad_vector_t constraintFunction(ad_scalar_t time,
                                 const ad_vector_t& state,  // 变量说明：state 表示状态。
                                 const ad_vector_t& input,  // 变量说明：input 表示输入。
                                 const ad_vector_t& parameters) const override;  // 变量说明：parameters 表示传给自动微分代价的参数向量。

  const SwitchedModelReferenceManager* referenceManagerPtr_;  // 变量说明：referenceManagerPtr_ 表示切换模型参考管理器指针。
  const MpcRobotModelBase<ad_scalar_t>* mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。
  const ContactRectangle contactRectangle_;  // 变量说明：contactRectangle_ 表示接触、rectangle。
  const size_t contactPointIndex_;  // 变量说明：contactPointIndex_ 表示接触、point、索引。
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;  // 变量说明：pinocchioInterfaceCppAd_ 表示自动微分版本的 Pinocchio 接口。

  const static size_t numConstraints_ = 4;  // 变量说明：numConstraints_ 表示num、constraints。
  bool isActive_ = true;  // 变量说明：isActive_ 表示该足端代价是否启用。
};

}  // namespace ocs2::humanoid

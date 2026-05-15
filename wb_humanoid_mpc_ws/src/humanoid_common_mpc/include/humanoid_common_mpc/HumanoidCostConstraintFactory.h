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

#pragma once
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/HumanoidCostConstraintFactory.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/contact/ContactRectangle.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

/**
 * Implements the constraint h(t,x,u) >= 0 to constrain the contact moment in the x-y plane.
 */

class HumanoidCostConstraintFactory {
 public:
  HumanoidCostConstraintFactory(const std::string& taskFile,
                                const std::string& referenceFile,  // 变量说明：referenceFile 表示参考、file。
                                const SwitchedModelReferenceManager& referenceManager,  // 变量说明：referenceManager 表示参考、manager。
                                const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,  // 变量说明：mpcRobotModelAD 表示自动微分版本的 MPC 机器人模型。
                                const ModelSettings& modelSettings,  // 变量说明：modelSettings 表示机器人关节和接触配置。
                                bool verbose = false);  // 变量说明：verbose 表示是否打印配置和初始化信息。

  ~HumanoidCostConstraintFactory() = default;
  HumanoidCostConstraintFactory(const HumanoidCostConstraintFactory& other) = delete;

  std::unique_ptr<StateInputCost> getStateInputQuadraticCost() const;

  std::unique_ptr<StateCost> getTerminalCost() const;

  std::unique_ptr<StateCost> getFootCollisionConstraint() const;

  std::unique_ptr<StateCost> getJointLimitsConstraint() const;

  std::unique_ptr<StateInputCost> getContactMomentXYConstraint(size_t contactPointIndex, const std::string& name) const;

  std::unique_ptr<StateInputConstraint> getZeroWrenchConstraint(size_t contactPointIndex) const;

  std::unique_ptr<StateInputCost> getFrictionForceConeConstraint(size_t contactPointIndex) const;

  std::unique_ptr<StateInputCost> getExternalTorqueQuadraticCost(size_t contactPointIndex) const;

 private:
  std::string taskFile_;  // 变量说明：taskFile_ 表示task、file。
  std::string referenceFile_;  // 变量说明：referenceFile_ 表示参考、file。
  const SwitchedModelReferenceManager* referenceManagerPtr_;  // 变量说明：referenceManagerPtr_ 表示切换模型参考管理器指针。
  const PinocchioInterface* pinocchioInterfacePtr_;  // 变量说明：pinocchioInterfacePtr_ 表示Pinocchio、interface、指针。
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。
  const MpcRobotModelBase<ad_scalar_t>* mpcRobotModelADPtr_;  // 变量说明：mpcRobotModelADPtr_ 表示MPC、机器人、模型、adptr。
  const ModelSettings& modelSettings_;  // 变量说明：modelSettings_ 表示模型、配置。
  const bool verbose_;  // 变量说明：verbose_ 表示本对象是否打印详细初始化信息。
};

}  // namespace ocs2::humanoid

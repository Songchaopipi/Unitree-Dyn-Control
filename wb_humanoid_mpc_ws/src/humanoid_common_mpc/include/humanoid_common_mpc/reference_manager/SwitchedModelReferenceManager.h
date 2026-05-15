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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/gait/GaitSchedule.h"
#include "humanoid_common_mpc/gait/MotionPhaseDefinition.h"
#include "humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h"

namespace ocs2::humanoid {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,  // 变量说明：swingTrajectoryPtr 表示摆动脚、轨迹、指针。
                                const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                const MpcRobotModelBase<scalar_t>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。

  ~SwitchedModelReferenceManager() override = default;

  /** Disable copy / move */
  SwitchedModelReferenceManager& operator=(const SwitchedModelReferenceManager&) = delete;  // 变量说明：重载运算符或禁用赋值操作，控制对象拷贝/输出行为。
  SwitchedModelReferenceManager(const SwitchedModelReferenceManager&) = delete;
  SwitchedModelReferenceManager& operator=(SwitchedModelReferenceManager&&) = delete;  // 变量说明：重载运算符或禁用赋值操作，控制对象拷贝/输出行为。
  SwitchedModelReferenceManager(SwitchedModelReferenceManager&&) = delete;

  // 函数说明：获取接触、flags，连接当前模块的数据流和控制逻辑。
  contact_flag_t getContactFlags(scalar_t time) const;

  // 函数说明：判断in、支撑脚/站立、步态相位，连接当前模块的数据流和控制逻辑。
  bool isInStancePhase(scalar_t time) const { return (getContactFlags(time)[0] && getContactFlags(time)[1]); }

  // 函数说明：判断in、接触，连接当前模块的数据流和控制逻辑。
  bool isInContact(scalar_t time, size_t contactIndex) const { return getContactFlags(time)[contactIndex]; };

  // 函数说明：设置arm、摆动脚、参考、主动关节，连接当前模块的数据流和控制逻辑。
  void setArmSwingReferenceActive(bool armSwingReferenceActive) { armSwingReferenceActive_ = armSwingReferenceActive; }

  // 函数说明：获取步态、时序表，连接当前模块的数据流和控制逻辑。
  const std::shared_ptr<GaitSchedule>& getGaitSchedule() const { return gaitSchedulePtr_; }

  // 函数说明：获取摆动脚、轨迹、planner，连接当前模块的数据流和控制逻辑。
  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() const { return swingTrajectoryPtr_; }

  // 函数说明：获取步态相位、variable，连接当前模块的数据流和控制逻辑。
  scalar_t getPhaseVariable(scalar_t time) const;

  // 函数说明：获取期望、状态，连接当前模块的数据流和控制逻辑。
  vector_t getDesiredState(const TargetTrajectories& targetTrajectories, const vector_t& state, scalar_t time) const;

 protected:
  // 函数说明：处理modify、references，连接当前模块的数据流和控制逻辑。
  virtual void modifyReferences(scalar_t initTime,  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。
                                scalar_t finalTime,  // 变量说明：finalTime 表示最终、时间。
                                const vector_t& initState,  // 变量说明：initState 表示init、状态。
                                size_t initMode,  // 变量说明：initMode 表示init、接触模式。
                                TargetTrajectories& targetTrajectories,  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
                                ModeSchedule& modeSchedule) override;  // 变量说明：modeSchedule 表示接触模式、时序表。

  // Adjusts the height of the target trajectories to current terrain height and returns that height.
  scalar_t adaptToCurrentGroundHeight(TargetTrajectories& targetTrajectories, const vector_t& initState, size_t initMode);
  scalar_t previousGroundHeightEstimate_{0.0};  // 变量说明：previousGroundHeightEstimate_ 表示previous、ground、height、estimate。

  PinocchioInterface pinocchioInterface_;  // 变量说明：pinocchioInterface_ 表示Pinocchio、interface。
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。
  ModeSchedule modeSchedule_;  // 变量说明：modeSchedule_ 表示接触模式、时序表。

  bool armSwingReferenceActive_{false};  // 变量说明：armSwingReferenceActive_ 表示arm、摆动脚、参考、主动关节。

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;  // 变量说明：gaitSchedulePtr_ 表示步态、时序表、指针。
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;  // 变量说明：swingTrajectoryPtr_ 表示摆动脚、轨迹、指针。
};

}  // namespace ocs2::humanoid

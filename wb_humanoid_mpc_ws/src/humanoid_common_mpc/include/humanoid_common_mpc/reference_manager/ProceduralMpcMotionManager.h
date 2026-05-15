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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <functional>
#include <string>
#include <vector>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <humanoid_common_mpc/command/WalkingVelocityCommand.h>
#include <humanoid_common_mpc/gait/GaitSchedule.h>
#include <humanoid_common_mpc/gait/ModeSequenceTemplate.h>

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/reference_manager/BreakFrequencyAlphaFilter.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

// ProceduralMpcMotionManager 是速度指令到 MPC 参考的在线桥接模块。
// 每次 SQP 求解前，它根据 vx/vy/yaw/height 命令选择合适 gait，并刷新 TargetTrajectories。
class ProceduralMpcMotionManager : public SolverSynchronizedModule {
 public:
  using VelocityTargetToTargetTrajectories =
      std::function<TargetTrajectories(const vector4_t& velocityTarget, scalar_t initTime, scalar_t finalTime, const vector_t& initState)>;

  struct GaitModeStateConfig {
    // 一个速度区间对应一个 gaitCommand，例如 stance/slow_walk/walk。
    std::string gaitCommand = "stance";  // 变量说明：gaitCommand 表示步态、命令。
    scalar_t minLinVelCmd;  // 变量说明：minLinVelCmd 表示min、lin、速度、cmd。
    scalar_t maxLinVelCmd;  // 变量说明：maxLinVelCmd 表示max、lin、速度、cmd。
    scalar_t minAngVelCmd;  // 变量说明：minAngVelCmd 表示min、ang、速度、cmd。
    scalar_t maxAngVelCmd;  // 变量说明：maxAngVelCmd 表示max、ang、速度、cmd。
    scalar_t linVelErrorThresh;  // 变量说明：linVelErrorThresh 表示lin、速度、误差、thresh。
    scalar_t angVelErrorThresh;  // 变量说明：angVelErrorThresh 表示ang、速度、误差、thresh。
  };

  /**
   * Constructor
   *
   * @param [in] gaitFile: The file path that contains the different gait patterns.
   * @param [in] referenceFile: The file path containing the default references and velocity limits.
   * @param [in] velocityTargetToTargetTrajectories: A function which transforms the commanded velocities to TargetTrajectories.
   * @param [in] switchedModelReferenceManagerPtr: A pointer to the switched model reference manager used to update gait and references
   */
  ProceduralMpcMotionManager(const std::string& gaitFile,
                             const std::string& referenceFile,  // 变量说明：referenceFile 表示参考、file。
                             std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,  // 变量说明：switchedModelReferenceManagerPtr 表示switched、模型、参考、manager、指针。
                             const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                             VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories);  // 变量说明：velocityTargetToTargetTrajectories 表示速度、目标、to、目标、trajectories。

  ProceduralMpcMotionManager(const ProceduralMpcMotionManager& mpcMotionManager) = delete;

  /**
   * Method called right before the solver runs
   *
   * @param initTime : start time of the MPC horizon
   * @param finalTime : Final time of the MPC horizon
   * @param initState : State at the start of the MPC horizon
   * @param referenceManager : The ReferenceManager which manages both ModeSchedule and TargetTrajectories.
   */
  void preSolverRun(scalar_t initTime,
                    scalar_t finalTime,  // 变量说明：finalTime 表示最终、时间。
                    const vector_t& initState,  // 变量说明：initState 表示init、状态。
                    const ReferenceManagerInterface& referenceManager) override;  // 变量说明：referenceManager 表示参考、manager。

  /**
   * Method called right after the solver runs
   *
   * @param primalSolution : primalSolution
   */
  void postSolverRun(const PrimalSolution& primalSolution) override {};

  virtual void setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand);  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。

  // 函数说明：处理transition、to、faster、步态，连接当前模块的数据流和控制逻辑。
  static bool transitionToFasterGait(const vector4_t& velCommandVec, const vector6_t& baseVelocity, const GaitModeStateConfig& cfg);

  // 函数说明：处理transition、to、slower、步态，连接当前模块的数据流和控制逻辑。
  static bool transitionToSlowerGait(const vector4_t& velCommandVec, const vector6_t& baseVelocity, const GaitModeStateConfig& cfg);

 protected:
  // clang-format off
  // 从慢到快的步态切换表。速度命令或当前速度误差越大，越可能切到更快 gait。
  const std::vector<GaitModeStateConfig> gaitModeStates_ {  // 变量说明：gaitModeStates_ 表示步态、接触模式、states。
    { "stance",       -0.1,  0.1, -0.1,  0.1,  10.0,   10.0 }, // Large threshold allows switching aw3ay from stance purely command based. 
    { "slow_walk",     0.05,  0.3,  0.05,  0.2,    0.05,  0.05},
    { "walk",          0.25,  0.5, 0.15,  0.35,    0.05,  0.05},
    { "slower_trot",   0.45, 0.7, 0.3,  0.55,      0.1,  0.1},
    { "slow_trot",     0.65, 0.9,  0.5,  0.7,      0.2,  0.2},
    { "trot",          0.8,  1.3,  0.65,  10.0,    0.2,  0.2},
    { "run",           1.2,  10.0,  0.65,  10.0,   0.2,  0.2}  
  };  // clang-format on

  // 当前 gaitModeStates_ 下标。
  size_t currentGaitMode_{0};  // 变量说明：currentGaitMode_ 表示当前、步态、接触模式。

  // 函数说明：获取scaled、walking、速度、命令，连接当前模块的数据流和控制逻辑。
  virtual WalkingVelocityCommand getScaledWalkingVelocityCommand() { return velocityCommand_; }  // 变量说明：WalkingVelocityCommand 表示walking、速度、命令。

  // 函数说明：处理scale、walking、速度、命令，连接当前模块的数据流和控制逻辑。
  WalkingVelocityCommand scaleWalkingVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) const;

  // referenceManager 负责把新的 mode schedule 和目标轨迹推给 OCS2；gaitSchedule 负责时间轴上的接触模式。
  std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr_;  // 变量说明：switchedModelReferenceManagerPtr_ 表示switched、模型、参考、manager、指针。
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;  // 变量说明：gaitSchedulePtr_ 表示步态、时序表、指针。
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。

  // reference.info 中的速度/高度/角速度限制，scaleWalkingVelocityCommand 会用它做归一化。
  const vector_t targetCommandLimits_;  // 变量说明：targetCommandLimits_ 表示目标、命令、limits。
  VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectoriesFun_;  // 变量说明：velocityTargetToTargetTrajectoriesFun_ 表示速度、目标、to、目标、trajectories、fun。

  std::vector<std::string> gaitList_;  // 变量说明：gaitList_ 表示步态、list。
  std::map<std::string, ModeSequenceTemplate> gaitMap_;

  // For velocity control mode
  scalar_t maxDisplacementVelocityX_ = 0.6;  // 变量说明：maxDisplacementVelocityX_ 表示max、displacement、速度、x。
  scalar_t maxDisplacementVelocityY_ = 0.3;  // 变量说明：maxDisplacementVelocityY_ 表示max、displacement、速度、y。
  scalar_t maxDeltaPelvisHeight_ = 0.3;  // 变量说明：maxDeltaPelvisHeight_ 表示max、delta、pelvis、height。
  scalar_t maxRotationVelocity_ = 0.6;  // 变量说明：maxRotationVelocity_ 表示max、旋转、速度。

  // 命令滤波，避免用户速度输入突变直接打到 MPC 目标。
  BreakFrequencyAlphaFilter velocityCommandFilter;  // 变量说明：velocityCommandFilter 表示速度、命令、滤波器。
  WalkingVelocityCommand velocityCommand_;  // 变量说明：velocityCommand_ 表示速度、命令。

  // 当前和上一次 gait 字符串，用于检测切换并重铺 ModeSchedule。
  std::string currentGaitCommand_{"stance"};  // 变量说明：currentGaitCommand_ 表示当前、步态、命令。
  std::string lastGaitCommand_{"stance"};  // 变量说明：lastGaitCommand_ 表示last、步态、命令。
  scalar_t lastGaitChangeTime_{0.0};  // 变量说明：lastGaitChangeTime_ 表示last、步态、change、时间。
};

}  // namespace ocs2::humanoid

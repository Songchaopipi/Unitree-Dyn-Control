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

// 自动中文注释：src/humanoid_common_mpc/src/reference_manager/SwitchedModelReferenceManager.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,  // 变量说明：swingTrajectoryPtr 表示摆动脚、轨迹、指针。
                                                             const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                             const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    // 函数说明：处理参考、manager，连接当前模块的数据流和控制逻辑。
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      // The reference manager gets a copy of the pinocchio model to use for initializing the ground height
      pinocchioInterface_(pinocchioInterface),
      mpcRobotModelPtr_(&mpcRobotModel) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {  // 变量说明：SwitchedModelReferenceManager 表示switched、模型、参考、manager。
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t SwitchedModelReferenceManager::getPhaseVariable(scalar_t time) const {  // 变量说明：SwitchedModelReferenceManager 表示switched、模型、参考、manager。
  const auto it = std::upper_bound(modeSchedule_.eventTimes.begin(), modeSchedule_.eventTimes.end(), time);  // 变量说明：it 表示名称查找迭代器。
  scalar_t nextEventTime = *it;  // 变量说明：nextEventTime 表示next、事件、时间。
  scalar_t prevEventTime = *(it - 1);  // 变量说明：prevEventTime 表示prev、事件、时间。

  if (modeSchedule_.modeAtTime(time) == LF) {
    return (0.5 * (time - prevEventTime) / (nextEventTime - prevEventTime));
  } else if (modeSchedule_.modeAtTime(time) == RF) {
    return (0.5 + 0.5 * (time - prevEventTime) / (nextEventTime - prevEventTime));
  } else {
    if (modeSchedule_.modeAtTime(prevEventTime - 0.01) == LF) {
      return 0.5;
    } else {
      return 0;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwitchedModelReferenceManager::adaptToCurrentGroundHeight(TargetTrajectories& targetTrajectories,  // 变量说明：SwitchedModelReferenceManager 表示switched、模型、参考、manager。
                                                                   const vector_t& initState,  // 变量说明：initState 表示init、状态。
                                                                   size_t initMode) {  // 变量说明：initMode 表示init、接触模式。
  scalar_t terrainHeight = computeGroundHeightEstimate(pinocchioInterface_, *mpcRobotModelPtr_,  // 变量说明：terrainHeight 表示terrain、height。
                                                       mpcRobotModelPtr_->getGeneralizedCoordinates(initState), initMode);

  terrainHeight = 0.0;

  // adapt target Trajectories to current terrain height
  // Since they are published in the past the current observations ground height might have drifted.

  // Adapt the ground height difference for every state in the target Trajectories.
  // The height difference between last update and the current update is applied here
  // to prevent applying the same difference twice in case the trajectories have not been updated.
  for (size_t i = 0; i < targetTrajectories.stateTrajectory.size(); i++) {
    vector_t& targetState = targetTrajectories.stateTrajectory[i];  // 变量说明：targetState 表示目标、状态。
    scalar_t heightDifference = terrainHeight - previousGroundHeightEstimate_;  // 变量说明：heightDifference 表示height、difference。
    mpcRobotModelPtr_->adaptBasePoseHeight(targetState, heightDifference);
  }
  previousGroundHeightEstimate_ = terrainHeight;
  return terrainHeight;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t SwitchedModelReferenceManager::getDesiredState(const TargetTrajectories& targetTrajectories,  // 变量说明：SwitchedModelReferenceManager 表示switched、模型、参考、manager。
                                                        const vector_t& state,  // 变量说明：state 表示状态。
                                                        scalar_t time) const {  // 变量说明：time 表示时间。
  vector_t xNominal = targetTrajectories.getDesiredState(time);  // 变量说明：xNominal 表示x、nominal。

  if (armSwingReferenceActive_) {
    scalar_t phaseVariable = this->getPhaseVariable(time);  // 变量说明：phaseVariable 表示步态相位、variable。
    vector_t desiredJointAngles = mpcRobotModelPtr_->getJointAngles(xNominal);  // 变量说明：desiredJointAngles 表示期望、关节、angles。

    vector3_t linVelCommand = mpcRobotModelPtr_->getBaseComLinearVelocity(xNominal);  // 变量说明：linVelCommand 表示lin、速度、命令。
    scalar_t currentEulerZ = mpcRobotModelPtr_->getBasePose(state)[3];  // 变量说明：currentEulerZ 表示当前、欧拉角、z。

    const scalar_t localVelXCommand = (std::cos(currentEulerZ) * linVelCommand[0] + std::sin(currentEulerZ) * linVelCommand[1]);  // 变量说明：localVelXCommand 表示local、速度、xcommand。

    const ModelSettings& modelSettings = mpcRobotModelPtr_->modelSettings;  // 变量说明：modelSettings 表示机器人关节和接触配置。

    scalar_t gaitCycleFactor = std::sin(2 * M_PI * (phaseVariable - 0.15)) * localVelXCommand;  // 变量说明：gaitCycleFactor 表示步态、cycle、factor。
    desiredJointAngles[modelSettings.j_l_shoulder_y_index] += -0.15 * gaitCycleFactor;
    desiredJointAngles[modelSettings.j_r_shoulder_y_index] += 0.15 * gaitCycleFactor;
    desiredJointAngles[modelSettings.j_l_elbow_y_index] += -0.15 * gaitCycleFactor;
    desiredJointAngles[modelSettings.j_r_elbow_y_index] += 0.15 * gaitCycleFactor;

    mpcRobotModelPtr_->setJointAngles(xNominal, desiredJointAngles);
  }
  return xNominal;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime,  // 变量说明：SwitchedModelReferenceManager 表示switched、模型、参考、manager。
                                                     scalar_t finalTime,  // 变量说明：finalTime 表示最终、时间。
                                                     const vector_t& initState,  // 变量说明：initState 表示init、状态。
                                                     size_t initMode,  // 变量说明：initMode 表示init、接触模式。
                                                     TargetTrajectories& targetTrajectories,  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
                                                     ModeSchedule& modeSchedule) {  // 变量说明：modeSchedule 表示接触模式、时序表。
  const auto timeHorizon = finalTime - initTime;  // 变量说明：timeHorizon 表示时间、预测时域。
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  scalar_t terrainHeight = adaptToCurrentGroundHeight(targetTrajectories, initState, initMode);  // 变量说明：terrainHeight 表示terrain、height。

  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);

  modeSchedule_ = modeSchedule;
}

}  // namespace ocs2::humanoid

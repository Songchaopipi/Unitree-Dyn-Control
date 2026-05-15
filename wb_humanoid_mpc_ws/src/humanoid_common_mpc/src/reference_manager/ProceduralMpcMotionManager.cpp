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

// 自动中文注释：src/humanoid_common_mpc/src/reference_manager/ProceduralMpcMotionManager.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"

#include <ocs2_core/misc/LoadData.h>

#include <cmath>
#include "humanoid_common_mpc/gait/GaitScheduleUpdater.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ProceduralMpcMotionManager::ProceduralMpcMotionManager(const std::string& gaitFile,
                                                       const std::string& referenceFile,  // 变量说明：referenceFile 表示参考、file。
                                                       std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,  // 变量说明：switchedModelReferenceManagerPtr 表示switched、模型、参考、manager、指针。
                                                       const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                                       VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories)
    // 函数说明：处理速度、目标、to、目标、trajectories、fun，连接当前模块的数据流和控制逻辑。
    : velocityTargetToTargetTrajectoriesFun_(std::move(velocityTargetToTargetTrajectories)),
      switchedModelReferenceManagerPtr_(switchedModelReferenceManagerPtr),
      gaitSchedulePtr_(switchedModelReferenceManagerPtr_->getGaitSchedule()),
      mpcRobotModelPtr_(&mpcRobotModel),
      velocityCommandFilter(5, vector4_t::Zero()) {
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityX", maxDisplacementVelocityX_);
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityY", maxDisplacementVelocityY_);
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "maxDeltaPelvisHeight", maxDeltaPelvisHeight_);
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "maxRotationVelocity", maxRotationVelocity_);

  gaitMap_ = getGaitMap(gaitFile);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void ProceduralMpcMotionManager::setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) {  // 变量说明：ProceduralMpcMotionManager 表示procedural、MPC、motion、manager。
  velocityCommand_ = scaleWalkingVelocityCommand(rawVelocityCommand);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

WalkingVelocityCommand ProceduralMpcMotionManager::scaleWalkingVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) const {  // 变量说明：ProceduralMpcMotionManager 表示procedural、MPC、motion、manager。
  WalkingVelocityCommand scaledCommand = rawVelocityCommand;  // 变量说明：scaledCommand 表示scaled、命令。
  scaledCommand.linear_velocity_x *= maxDisplacementVelocityX_;
  scaledCommand.linear_velocity_y *= maxDisplacementVelocityY_;
  scaledCommand.angular_velocity_z *= maxRotationVelocity_;
  return scaledCommand;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool ProceduralMpcMotionManager::transitionToFasterGait(const vector4_t& velCommandVec,  // 变量说明：ProceduralMpcMotionManager 表示procedural、MPC、motion、manager。
                                                        const vector6_t& baseVelocity,  // 变量说明：baseVelocity 表示浮动基/机身、速度。
                                                        const GaitModeStateConfig& cfg) {  // 变量说明：cfg 表示当前步态模式的阈值配置。
  bool fasterGaitRequested = (std::abs(velCommandVec(0)) > cfg.maxLinVelCmd || std::abs(velCommandVec(1)) > cfg.maxLinVelCmd ||  // 变量说明：fasterGaitRequested 表示faster、步态、requested。
                              std::abs(velCommandVec(3)) > cfg.maxAngVelCmd);

  bool withinMaxSpeedErrorThreshold = (std::abs(baseVelocity(0)) > cfg.maxLinVelCmd - cfg.linVelErrorThresh ||  // 变量说明：withinMaxSpeedErrorThreshold 表示within、max、speed、误差、threshold。
                                       std::abs(baseVelocity(1)) > cfg.maxLinVelCmd - cfg.linVelErrorThresh ||
                                       std::abs(baseVelocity(3)) > cfg.maxAngVelCmd - cfg.angVelErrorThresh);
  return fasterGaitRequested && withinMaxSpeedErrorThreshold;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool ProceduralMpcMotionManager::transitionToSlowerGait(const vector4_t& velCommandVec,  // 变量说明：ProceduralMpcMotionManager 表示procedural、MPC、motion、manager。
                                                        const vector6_t& baseVelocity,  // 变量说明：baseVelocity 表示浮动基/机身、速度。
                                                        const GaitModeStateConfig& cfg) {  // 变量说明：cfg 表示当前步态模式的阈值配置。
  bool slowerGaitRequested = (std::abs(velCommandVec(0)) < cfg.minLinVelCmd && std::abs(velCommandVec(1)) < cfg.minLinVelCmd &&  // 变量说明：slowerGaitRequested 表示slower、步态、requested。
                              std::abs(velCommandVec(3)) < cfg.minAngVelCmd);

  bool baseSpeedSlowEnough = (std::abs(baseVelocity(0)) < cfg.minLinVelCmd + cfg.linVelErrorThresh &&  // 变量说明：baseSpeedSlowEnough 表示浮动基/机身、speed、slow、enough。
                              std::abs(baseVelocity(1)) < cfg.minLinVelCmd + cfg.linVelErrorThresh &&
                              std::abs(velCommandVec(3)) < cfg.minAngVelCmd + cfg.angVelErrorThresh);

  return slowerGaitRequested && baseSpeedSlowEnough;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void ProceduralMpcMotionManager::preSolverRun(scalar_t initTime,  // 变量说明：ProceduralMpcMotionManager 表示procedural、MPC、motion、manager。
                                              scalar_t finalTime,  // 变量说明：finalTime 表示最终、时间。
                                              const vector_t& initState,  // 变量说明：initState 表示init、状态。
                                              const ReferenceManagerInterface& referenceManager) {  // 变量说明：referenceManager 表示参考、manager。
  WalkingVelocityCommand incommingVelCommand = getScaledWalkingVelocityCommand();  // 变量说明：incommingVelCommand 表示incomming、速度、命令。
  vector4_t filteredVelCommand = velocityCommandFilter.getFilteredVector(incommingVelCommand.toVector());  // 变量说明：filteredVelCommand 表示filtered、速度、命令。

  // Update TargetTrajectories
  TargetTrajectories targetTrajectories = velocityTargetToTargetTrajectoriesFun_(filteredVelCommand, initTime, finalTime, initState);  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
  switchedModelReferenceManagerPtr_->setTargetTrajectories(targetTrajectories);

  static GaitModeStateConfig currentCfg = gaitModeStates_[currentGaitMode_];  // 变量说明：currentCfg 表示当前、cfg。
  vector6_t baseVelocity = mpcRobotModelPtr_->getBaseComVelocity(initState);  // 变量说明：baseVelocity 表示浮动基/机身、速度。

  // Do not change the gait pattern for at least 0.5s
  if (initTime > lastGaitChangeTime_ + 0.2) {
    if (transitionToFasterGait(filteredVelCommand, baseVelocity, currentCfg)) {
      std::cout << "filteredVelCommand: " << filteredVelCommand.transpose() << std::endl;
      std::cout << "Linear limits: " << currentCfg.minLinVelCmd << ", " << currentCfg.maxLinVelCmd << std::endl;
      currentGaitMode_++;
      currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      std::cout << "ProceduralMpcMotionManager: Increasing to gait:" << currentCfg.gaitCommand << std::endl;
      lastGaitChangeTime_ = initTime;
    } else if (transitionToSlowerGait(filteredVelCommand, baseVelocity, currentCfg)) {
      std::cout << "filteredVelCommand: " << filteredVelCommand.transpose() << std::endl;
      std::cout << "Linear limits: " << currentCfg.minLinVelCmd << ", " << currentCfg.maxLinVelCmd << std::endl;
      currentGaitMode_--;
      currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      std::cout << "ProceduralMpcMotionManager: Decreasing to gait:" << currentCfg.gaitCommand << std::endl;
      lastGaitChangeTime_ = initTime;
    }
  }

  if (currentGaitCommand_ != lastGaitCommand_) {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(currentGaitCommand_);  // 变量说明：modeSequenceTemplate 表示接触模式、sequence、template。

    // 函数说明：更新步态、时序表，连接当前模块的数据流和控制逻辑。
    GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr_, modeSequenceTemplate, initTime, finalTime);
    lastGaitCommand_ = currentGaitCommand_;
  }
}

}  // namespace ocs2::humanoid

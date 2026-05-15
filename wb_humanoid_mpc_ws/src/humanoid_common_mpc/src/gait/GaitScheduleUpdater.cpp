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

// 自动中文注释：src/humanoid_common_mpc/src/gait/GaitScheduleUpdater.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/gait/GaitScheduleUpdater.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitScheduleUpdater::GaitScheduleUpdater(std::shared_ptr<GaitSchedule> gaitSchedulePtr)
    // 函数说明：处理步态、时序表、指针，连接当前模块的数据流和控制逻辑。
    : gaitSchedulePtr_(std::move(gaitSchedulePtr)), receivedGait_({0.0, 1.0}, {ModeNumber::STANCE}), gaitUpdated_(false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void GaitScheduleUpdater::updateGaitSchedule(std::shared_ptr<GaitSchedule>& gaitSchedulePtr,  // 变量说明：GaitScheduleUpdater 表示步态、时序表、updater。
                                             const ModeSequenceTemplate& updatedGait,  // 变量说明：updatedGait 表示updated、步态。
                                             scalar_t initTime,  // 变量说明：initTime 表示init、时间。
                                             scalar_t finalTime) {  // 变量说明：finalTime 表示最终、时间。
  std::cerr << updatedGait;
  const scalar_t timeHorizon = finalTime - initTime;  // 变量说明：timeHorizon 表示时间、预测时域。
  const scalar_t earliestSwitchingTime = (0.7 * finalTime + 0.3 * initTime);  // This is a heuristic
  std::cerr << "[GaitScheduleUpdater]: Setting new gait after time " << earliestSwitchingTime << "\n";
  // Find the first time that is greater than current_time
  const auto& modeSchedule = gaitSchedulePtr->getModeSchedule(initTime, finalTime + timeHorizon);  // 变量说明：modeSchedule 表示接触模式、时序表。

  const auto it = std::upper_bound(modeSchedule.eventTimes.begin(), modeSchedule.eventTimes.end(), earliestSwitchingTime);  // 变量说明：it 表示名称查找迭代器。
  scalar_t nextEventTime;  // 变量说明：nextEventTime 表示next、事件、时间。
  if (it == modeSchedule.eventTimes.end()) {
    nextEventTime = finalTime;
  } else {
    if (modeSchedule.modeAtTime(*it) == LF) {
      nextEventTime = *(it - 1);
    } else {
      nextEventTime = *it;
    }
  }

  gaitSchedulePtr->insertModeSequenceTemplate(updatedGait, nextEventTime, 1.5 * timeHorizon);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void GaitScheduleUpdater::preSolverRun(scalar_t initTime,  // 变量说明：GaitScheduleUpdater 表示步态、时序表、updater。
                                       scalar_t finalTime,  // 变量说明：finalTime 表示最终、时间。
                                       const vector_t& currentState,  // 变量说明：currentState 表示当前、状态。
                                       const ReferenceManagerInterface& referenceManager) {  // 变量说明：referenceManager 表示参考、manager。
  if (gaitUpdated_) {
    updateGaitSchedule(gaitSchedulePtr_, receivedGait_, initTime, finalTime);
    gaitUpdated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitScheduleUpdater::updateModeSequence(const ModeSequenceTemplate& modeSequenceTemplate) {  // 变量说明：GaitScheduleUpdater 表示步态、时序表、updater。
  receivedGait_ = modeSequenceTemplate;
  gaitUpdated_ = true;
}

}  // namespace ocs2::humanoid

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

// 自动中文注释：src/humanoid_common_mpc/src/gait/GaitSchedule.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/gait/GaitSchedule.h"

#include <ocs2_core/misc/Lookup.h>

#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitSchedule::GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate, scalar_t phaseTransitionStanceTime)
    // 函数说明：处理接触模式、时序表，连接当前模块的数据流和控制逻辑。
    : modeSchedule_(std::move(initModeSchedule)),
      modeSequenceTemplate_(std::move(initModeSequenceTemplate)),
      phaseTransitionStanceTime_(phaseTransitionStanceTime) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void GaitSchedule::insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime, scalar_t finalTime) {  // 变量说明：GaitSchedule 表示步态、时序表。
  modeSequenceTemplate_ = modeSequenceTemplate;
  auto& eventTimes = modeSchedule_.eventTimes;  // 变量说明：eventTimes 表示事件、times。
  auto& modeSequence = modeSchedule_.modeSequence;  // 变量说明：modeSequence 表示接触模式、sequence。

  // find the index on which the new gait should be added
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), startTime) - eventTimes.begin();  // 变量说明：index 表示索引。

  // delete the old logic from the index
  if (index < eventTimes.size()) {
    eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
    modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
  }

  // add an intermediate stance phase
  scalar_t phaseTransitionStanceTime = phaseTransitionStanceTime_;  // 变量说明：phaseTransitionStanceTime 表示步态相位、transition、支撑脚/站立、时间。
  if (!modeSequence.empty() && modeSequence.back() == ModeNumber::STANCE) {
    phaseTransitionStanceTime = 0.0;
  }

  if (phaseTransitionStanceTime > 0.0) {
    eventTimes.push_back(startTime);
    modeSequence.push_back(ModeNumber::STANCE);
  }

  // tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
  tileModeSequenceTemplate(startTime + phaseTransitionStanceTime, finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime) {  // 变量说明：GaitSchedule 表示步态、时序表。
  auto& eventTimes = modeSchedule_.eventTimes;  // 变量说明：eventTimes 表示事件、times。
  auto& modeSequence = modeSchedule_.modeSequence;  // 变量说明：modeSequence 表示接触模式、sequence。
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), lowerBoundTime) - eventTimes.begin();  // 变量说明：index 表示索引。

  if (index > 0) {
    // delete the old logic from index and set the default start phase to stance
    eventTimes.erase(eventTimes.begin(),
                     eventTimes.begin() + index - 1);  // keep the one before the last to make it stance
    modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);

    // set the default initial phase
    modeSequence.front() = ModeNumber::STANCE;
  }

  // Start tiling at time
  const auto tilingStartTime = eventTimes.empty() ? upperBoundTime : eventTimes.back();  // 变量说明：tilingStartTime 表示tiling、start、时间。

  // delete the last default stance phase
  eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
  modeSequence.erase(modeSequence.end() - 1, modeSequence.end());

  // tile the template logic
  tileModeSequenceTemplate(tilingStartTime, upperBoundTime);
  return modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime) {  // 变量说明：GaitSchedule 表示步态、时序表。
  auto& eventTimes = modeSchedule_.eventTimes;  // 变量说明：eventTimes 表示事件、times。
  auto& modeSequence = modeSchedule_.modeSequence;  // 变量说明：modeSequence 表示接触模式、sequence。
  const auto& templateTimes = modeSequenceTemplate_.switchingTimes;  // 变量说明：templateTimes 表示template、times。
  const auto& templateModeSequence = modeSequenceTemplate_.modeSequence;  // 变量说明：templateModeSequence 表示template、接触模式、sequence。
  const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size();  // 变量说明：numTemplateSubsystems 表示num、template、subsystems。

  // If no template subsystem is defined, the last subsystem should continue for ever
  if (numTemplateSubsystems == 0) {
    return;
  }

  if (!eventTimes.empty() && startTime <= eventTimes.back()) {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  eventTimes.push_back(startTime);

  // concatenate from index
  while (eventTimes.back() < finalTime) {
    for (size_t i = 0; i < templateModeSequence.size(); i++) {
      modeSequence.push_back(templateModeSequence[i]);
      scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];  // 变量说明：deltaTime 表示delta、时间。
      eventTimes.push_back(eventTimes.back() + deltaTime);
    }  // end of i loop
  }  // end of while loop

  // default final phase
  modeSequence.push_back(ModeNumber::STANCE);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::shared_ptr<GaitSchedule> GaitSchedule::loadGaitSchedule(const std::string& referenceFile,  // 变量说明：GaitSchedule 表示步态、时序表。
                                                             const ModelSettings& modelSettings,  // 变量说明：modelSettings 表示机器人关节和接触配置。
                                                             bool verbose) {  // 变量说明：verbose 表示是否打印配置和初始化信息。
  const auto initModeSchedule = loadModeSchedule(referenceFile, "initialModeSchedule", false);  // 变量说明：initModeSchedule 表示init、接触模式、时序表。
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(referenceFile, "defaultModeSequenceTemplate", false);  // 变量说明：defaultModeSequenceTemplate 表示默认、接触模式、sequence、template。

  const auto defaultGait = [&] {  // 变量说明：defaultGait 表示默认、步态。
    Gait gait{};  // 变量说明：gait 表示步态。
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  // display
  if (verbose) {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
    std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void GaitSchedule::updateModeSchedule(const ModeSchedule& modeSchedule) {  // 变量说明：GaitSchedule 表示步态、时序表。
  modeSchedule_ = modeSchedule;
}

}  // namespace ocs2::humanoid

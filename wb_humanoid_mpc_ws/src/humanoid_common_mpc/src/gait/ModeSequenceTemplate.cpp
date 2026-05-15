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

// 自动中文注释：src/humanoid_common_mpc/src/gait/ModeSequenceTemplate.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& stream, const ModeSequenceTemplate& modeSequenceTemplate) {  // 变量说明：重载运算符或禁用赋值操作，控制对象拷贝/输出行为。
  stream << "Template switching times: {" << toDelimitedString(modeSequenceTemplate.switchingTimes) << "}\n";
  stream << "Template mode sequence:   {" << toDelimitedString(modeSequenceTemplate.modeSequence) << "}\n";
  return stream;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSequenceTemplate loadModeSequenceTemplate(const std::string& filename, const std::string& topicName, bool verbose) {
  std::vector<scalar_t> switchingTimes;  // 变量说明：switchingTimes 表示switching、times。
  // 函数说明：加载std、向量，连接当前模块的数据流和控制逻辑。
  loadData::loadStdVector(filename, topicName + ".switchingTimes", switchingTimes, verbose);

  std::vector<std::string> modeSequenceString;  // 变量说明：modeSequenceString 表示接触模式、sequence、string。
  // 函数说明：加载std、向量，连接当前模块的数据流和控制逻辑。
  loadData::loadStdVector(filename, topicName + ".modeSequence", modeSequenceString, verbose);

  if (switchingTimes.empty() || modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSequenceTemplate] failed to load : " + topicName + " from " + filename);
  }

  // convert the mode name to mode enum
  std::vector<size_t> modeSequence;  // 变量说明：modeSequence 表示接触模式、sequence。
  modeSequence.reserve(modeSequenceString.size());
  for (const auto& modeName : modeSequenceString) {
    modeSequence.push_back(string2ModeNumber(modeName));
  }

  return {switchingTimes, modeSequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Gait toGait(const ModeSequenceTemplate& modeSequenceTemplate) {
  const auto startTime = modeSequenceTemplate.switchingTimes.front();  // 变量说明：startTime 表示start、时间。
  const auto endTime = modeSequenceTemplate.switchingTimes.back();  // 变量说明：endTime 表示end、时间。
  Gait gait;  // 变量说明：gait 表示步态。
  gait.duration = endTime - startTime;
  // Events: from time -> phase
  gait.eventPhases.reserve(modeSequenceTemplate.switchingTimes.size());
  std::for_each(modeSequenceTemplate.switchingTimes.begin() + 1, modeSequenceTemplate.switchingTimes.end() - 1,
                [&](scalar_t eventTime) { gait.eventPhases.push_back((eventTime - startTime) / gait.duration); });
  // Modes:
  gait.modeSequence = modeSequenceTemplate.modeSequence;
  assert(isValidGait(gait));
  return gait;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule loadModeSchedule(const std::string& filename, const std::string& topicName, bool verbose) {
  std::vector<scalar_t> eventTimes;  // 变量说明：eventTimes 表示事件、times。
  // 函数说明：加载std、向量，连接当前模块的数据流和控制逻辑。
  loadData::loadStdVector(filename, topicName + ".eventTimes", eventTimes, verbose);

  std::vector<std::string> modeSequenceString;  // 变量说明：modeSequenceString 表示接触模式、sequence、string。
  // 函数说明：加载std、向量，连接当前模块的数据流和控制逻辑。
  loadData::loadStdVector(filename, topicName + ".modeSequence", modeSequenceString, verbose);

  if (modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSchedule] failed to load : " + topicName + " from " + filename);
  }

  // convert the mode name to mode enum
  std::vector<size_t> modeSequence;  // 变量说明：modeSequence 表示接触模式、sequence。
  modeSequence.reserve(modeSequenceString.size());
  for (const auto& modeName : modeSequenceString) {
    modeSequence.push_back(string2ModeNumber(modeName));
  }

  return {eventTimes, modeSequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

// returns the gait map for a gait file
std::map<std::string, ModeSequenceTemplate> getGaitMap(const std::string& gaitFile, bool verbose) {
  std::vector<std::string> gaitList;  // 变量说明：gaitList 表示步态、list。
  std::map<std::string, ModeSequenceTemplate> gaitMap;

  // 函数说明：加载std、向量，连接当前模块的数据流和控制逻辑。
  loadData::loadStdVector(gaitFile, "list", gaitList, verbose);

  gaitMap.clear();
  for (const auto& gaitName : gaitList) {
    gaitMap.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  return gaitMap;
}

}  // namespace ocs2::humanoid

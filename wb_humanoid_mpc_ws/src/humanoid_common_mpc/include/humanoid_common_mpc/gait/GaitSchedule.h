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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/gait/GaitSchedule.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <mutex>

#include <ocs2_core/reference/ModeSchedule.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"

namespace ocs2::humanoid {

// GaitSchedule 把离散的步态模板铺到实际时间轴上，生成 OCS2 的 ModeSchedule。
// ModeSchedule 决定某个时间段是双脚支撑、左脚支撑、右脚支撑还是腾空。
class GaitSchedule {
 public:
  GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate, scalar_t phaseTransitionStanceTime);

  /**
   * @param [in] lowerBoundTime: The smallest time for which the ModeSchedule should be defined.
   * @param [in] upperBoundTime: The greatest time for which the ModeSchedule should be defined.
   */
  ModeSchedule getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime);

  // 函数说明：获取当前、接触模式、时序表，连接当前模块的数据流和控制逻辑。
  ModeSchedule getCurrentModeSchedule() const { return modeSchedule_; };

  /**
   * Used to insert a new user defined logic in the given time period.
   *
   * @param [in] startTime: The initial time from which the new mode sequence template should start.
   * @param [in] finalTime: The final time until when the new mode sequence needs to be defined.
   */
  void insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime, scalar_t finalTime);

  // 函数说明：加载步态、时序表，连接当前模块的数据流和控制逻辑。
  static std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& referenceFile,
                                                        const ModelSettings& modelSettings,  // 变量说明：modelSettings 表示机器人关节和接触配置。
                                                        bool verbose = false);  // 变量说明：verbose 表示是否打印配置和初始化信息。

  void updateModeSchedule(const ModeSchedule& modeSchedule);

 private:
  /**
   * Extends the switch information from lowerBoundTime to upperBoundTime based on the template mode sequence.
   *
   * @param [in] startTime: The initial time from which the mode schedule should be appended with the template.
   * @param [in] finalTime: The final time to which the mode schedule should be appended with the template.
   */
  void tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime);

 private:
  // 当前已经展开到时间轴上的模式序列。
  ModeSchedule modeSchedule_;  // 变量说明：modeSchedule_ 表示接触模式、时序表。
  // 默认重复铺设的周期模板。
  ModeSequenceTemplate modeSequenceTemplate_;  // 变量说明：modeSequenceTemplate_ 表示接触模式、sequence、template。
  // 模式切换时额外保留站立相的时间。
  scalar_t phaseTransitionStanceTime_;  // 变量说明：phaseTransitionStanceTime_ 表示步态相位、transition、支撑脚/站立、时间。
};

}  // namespace ocs2::humanoid

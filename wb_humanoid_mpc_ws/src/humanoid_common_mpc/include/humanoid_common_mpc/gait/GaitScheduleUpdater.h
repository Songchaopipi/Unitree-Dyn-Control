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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/gait/GaitScheduleUpdater.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <mutex>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <humanoid_common_mpc/gait/GaitSchedule.h>
#include <humanoid_common_mpc/gait/ModeSequenceTemplate.h>
#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>

namespace ocs2::humanoid {

// Lock free implementation of the gait scheduler updater.

class GaitScheduleUpdater : public SolverSynchronizedModule {
 public:
  GaitScheduleUpdater(std::shared_ptr<GaitSchedule> gaitSchedulePtr);

  // 函数说明：处理pre、求解器、run，连接当前模块的数据流和控制逻辑。
  virtual void preSolverRun(scalar_t initTime,  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。
                            scalar_t finalTime,  // 变量说明：finalTime 表示最终、时间。
                            const vector_t& currentState,  // 变量说明：currentState 表示当前、状态。
                            const ReferenceManagerInterface& referenceManager) override;  // 变量说明：referenceManager 表示参考、manager。

  // 函数说明：处理post、求解器、run，连接当前模块的数据流和控制逻辑。
  void postSolverRun(const PrimalSolution&) override {};

  // Override this function in case you need to e.g. access the received gait in a mutex protected way.
  virtual ModeSequenceTemplate getReceivedGait() { return receivedGait_; }  // 变量说明：ModeSequenceTemplate 表示接触模式、sequence、template。

  // make sure this function is not called in paralell to the presolver run without proper protection against race condition.
  void updateModeSequence(const ModeSequenceTemplate& modeSequenceTemplate);

  // 函数说明：更新步态、时序表，连接当前模块的数据流和控制逻辑。
  static void updateGaitSchedule(std::shared_ptr<GaitSchedule>& gaitSchedulePtr,
                                 const ModeSequenceTemplate& updatedGait,  // 变量说明：updatedGait 表示updated、步态。
                                 scalar_t initTime,  // 变量说明：initTime 表示init、时间。
                                 scalar_t finalTime);  // 变量说明：finalTime 表示最终、时间。

 protected:
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;  // 变量说明：gaitSchedulePtr_ 表示步态、时序表、指针。
  bool gaitUpdated_;  // 变量说明：gaitUpdated_ 表示步态、updated。
  ModeSequenceTemplate receivedGait_;  // 变量说明：receivedGait_ 表示received、步态。
};

}  // namespace ocs2::humanoid

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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/swing_foot_planner/ImpactProximityFactorBase.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_core/Types.h>

namespace ocs2::humanoid {

class ImpactProximityFactorBase {
 public:
  ImpactProximityFactorBase(scalar_t startTime, scalar_t endTime) : startTime_(startTime), endTime_(endTime){};

  // 函数说明：设置时间、interval，连接当前模块的数据流和控制逻辑。
  virtual void setTimeInterval(scalar_t startTime, scalar_t endTime) {  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。
    startTime_ = startTime;
    endTime_ = endTime;
  }

  // 函数说明：获取数值，连接当前模块的数据流和控制逻辑。
  virtual scalar_t getValue(scalar_t time) const = 0;  // 变量说明：scalar_t 表示scalar、t。

 protected:
  scalar_t startTime_;  // 变量说明：startTime_ 表示start、时间。
  scalar_t endTime_;  // 变量说明：endTime_ 表示end、时间。
};

}  // namespace ocs2::humanoid

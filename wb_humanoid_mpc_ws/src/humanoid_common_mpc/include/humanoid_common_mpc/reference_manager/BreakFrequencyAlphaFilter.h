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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/reference_manager/BreakFrequencyAlphaFilter.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <humanoid_common_mpc/common/Types.h>
#include <cmath>

namespace ocs2::humanoid {

class BreakFrequencyAlphaFilter final {
 public:
  /**
   * Constructor
   *
   * @param [in] breakFrequency: Break frequency (cut-off frequency) in Hz.
   */
  BreakFrequencyAlphaFilter(scalar_t breakFrequency, const vector_t& y_init)
      // 函数说明：处理break、delta、t，连接当前模块的数据流和控制逻辑。
      : breakDeltaT_(1 / (2 * M_PI * breakFrequency)), y_last_(y_init) {
    lastTimeFilterCalled = std::chrono::steady_clock::now();
  };

  // 函数说明：获取filtered、向量，连接当前模块的数据流和控制逻辑。
  vector_t getFilteredVector(const vector_t& x) {
    assert(x.size() == y_last_.size());
    scalar_t alpha = computeAlpha();  // 变量说明：alpha 表示滤波系数。
    return (alpha * x + (1 - alpha) * y_last_);
  }

 private:
  // 函数说明：计算滤波系数，连接当前模块的数据流和控制逻辑。
  scalar_t computeAlpha() {
    auto now = std::chrono::steady_clock::now();  // 变量说明：now 表示当前墙钟时间。
    std::chrono::duration<double> durationInSeconds = now - lastTimeFilterCalled;  // 变量说明：durationInSeconds 表示duration、in、seconds。
    scalar_t delta_t = durationInSeconds.count();  // 变量说明：delta_t 表示delta、t。
    return (delta_t / (delta_t + breakDeltaT_));
  }

  scalar_t breakDeltaT_;  // 变量说明：breakDeltaT_ 表示break、delta、t。
  vector_t y_last_;  // 变量说明：y_last_ 表示y、last。
  std::chrono::time_point<std::chrono::steady_clock> lastTimeFilterCalled;  // 变量说明：lastTimeFilterCalled 表示last、时间、滤波器、called。
};

}  // namespace ocs2::humanoid

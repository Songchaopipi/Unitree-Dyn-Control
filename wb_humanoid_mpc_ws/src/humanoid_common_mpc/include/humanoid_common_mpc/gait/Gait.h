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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/gait/Gait.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ostream>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2::humanoid {

/**
 * A gait is a periodic mode schedule parameterized by a "phase" variable.
 * 中文说明：Gait 是“一个周期内接触模式如何切换”的模板。
 * duration 是一个完整步态周期时长；eventPhases 是周期内切换点；modeSequence 是每段对应的接触模式。
 *
 * The eventPhases only indicate switches of modes, i.e. phase = 0 and phase = 1 are not part of the eventPhases.
 * The number of modes is therefore number of phases + 1
 *
 * The conversion to time is regulated by a duration
 */
struct Gait {
  /** time for one gait cycle*/
  scalar_t duration;  // 变量说明：duration 表示duration。
  /** points in (0.0, 1.0) along the gait cycle where the contact mode changes, size N-1 */
  std::vector<scalar_t> eventPhases;  // 变量说明：eventPhases 表示事件、phases。
  /** sequence of contact modes, size: N */
  std::vector<size_t> modeSequence;  // 变量说明：modeSequence 表示接触模式、sequence。
};

/**
 * isValidGait checks the following properties
 * - positive duration
 * - eventPhases are all in (0.0, 1.0)
 * - eventPhases are sorted
 * - the size of the modeSequences is 1 more than the eventPhases.
 */
bool isValidGait(const Gait& gait);

/** Check is if the phase is in [0.0, 1.0) */
bool isValidPhase(scalar_t phase);

/** Wraps a phase to [0.0, 1.0) */
scalar_t wrapPhase(scalar_t phase);

/** The modes are selected with a closed-open interval: [ ) */
int getModeIndexFromPhase(scalar_t phase, const Gait& gait);

/** Gets the active mode from the phase variable */
size_t getModeFromPhase(scalar_t phase, const Gait& gait);

/** Returns the time left in the gait based on the phase variable */
scalar_t timeLeftInGait(scalar_t phase, const Gait& gait);

/** Returns the time left in the current based on the phase variable */
scalar_t timeLeftInMode(scalar_t phase, const Gait& gait);

/** Print gait */
std::ostream& operator<<(std::ostream& stream, const Gait& gait);  // 变量说明：重载运算符或禁用赋值操作，控制对象拷贝/输出行为。

}  // namespace ocs2::humanoid

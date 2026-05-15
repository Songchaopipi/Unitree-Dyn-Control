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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_core/reference/ModeSchedule.h>

#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/swing_foot_planner/SplineCpg.h"

namespace ocs2::humanoid {

// 根据 ModeSchedule 生成摆动脚 z 方向轨迹。MPC 的足端约束会查询这里的 z/zdot/zddot，
// 让离地脚抬高、落脚时降低，并在接触相保持足底高度。
class SwingTrajectoryPlanner {
 public:
  struct Config {
    // 离地/触地速度和摆动高度，用于生成三次样条足高曲线。
    scalar_t liftOffVelocity = 0.0;  // 变量说明：liftOffVelocity 表示lift、off、速度。
    scalar_t touchDownVelocity = 0.0;  // 变量说明：touchDownVelocity 表示touch、down、速度。
    scalar_t swingHeight = 0.1;  // 变量说明：swingHeight 表示摆动脚、height。
    scalar_t swingTimeScale = 0.15;  // swing phases shorter than this time will be scaled down in height and velocity
    scalar_t touchDownHeightOffset = 0.0;  // 变量说明：touchDownHeightOffset 表示touch、down、height、offset。

    scalar_t impactProximityFactorLiftOffVelocity = 0;    // should lesser or equal 0
    scalar_t impactProximityFactorTouchDownVelocity = 0;  // should be greater or equal to 0
    scalar_t impactProximityFactorMidPointValue = 0.1;    // should be between 0 and 1
  };

  SwingTrajectoryPlanner(Config config, size_t numFeet);

  void update(const ModeSchedule& modeSchedule, scalar_t terrainHeight);

  // 函数说明：更新update，连接当前模块的数据流和控制逻辑。
  void update(const ModeSchedule& modeSchedule,
              const feet_array_t<scalar_array_t>& liftOffHeightSequence,  // 变量说明：liftOffHeightSequence 表示lift、off、height、sequence。
              const feet_array_t<scalar_array_t>& touchDownHeightSequence);  // 变量说明：touchDownHeightSequence 表示touch、down、height、sequence。

  // 函数说明：获取zacceleration、约束，连接当前模块的数据流和控制逻辑。
  scalar_t getZaccelerationConstraint(size_t leg, scalar_t time) const;

  // 函数说明：获取zvelocity、约束，连接当前模块的数据流和控制逻辑。
  scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

  // 函数说明：获取zposition、约束，连接当前模块的数据流和控制逻辑。
  scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;

  // 函数说明：获取impact、proximity、factor，连接当前模块的数据流和控制逻辑。
  scalar_t getImpactProximityFactor(size_t leg, scalar_t time) const;

 private:
  /**
   * Extracts for each leg the contact sequence over the motion phase sequence.
   * @param phaseIDsStock
   * @return contactFlagStock
   */
  feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

  /**
   * Finds the take-off and touch-down times indices for a specific leg.
   *
   * @param index
   * @param contactFlagStock
   * @return {The take-off time index for swing legs, touch-down time index for swing legs}
   */
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);  // 变量说明：该对象使用标准库容器/数组保存相关数据。

  /**
   * based on the input phaseIDsStock finds the start subsystem and final subsystem of the swing
   * phases of the a foot in each subsystem.
   *
   * startTimeIndexStock: eventTimes[startTimesIndex] will be the take-off time for the requested leg.
   * finalTimeIndexStock: eventTimes[finalTimesIndex] will be the touch-down time for the requested leg.
   *
   * @param [in] footIndex: Foot index
   * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
   * @param [in] contactFlagStock: The sequence of the contact status for the requested leg.
   * @return { startTimeIndexStock, finalTimeIndexStock}
   */
  static std::pair<std::vector<int>, std::vector<int>> updateFootSchedule(const std::vector<bool>& contactFlagStock);  // 变量说明：该对象使用标准库容器/数组保存相关数据。

  /**
   * Check if event time indices are valid
   * @param leg
   * @param index : phase index
   * @param startIndex : liftoff event time index
   * @param finalIndex : touchdown event time index
   * @param phaseIDsStock : mode sequence
   */
  static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex, const std::vector<size_t>& phaseIDsStock);

  // 函数说明：处理摆动脚、轨迹、scaling，连接当前模块的数据流和控制逻辑。
  static scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale);

  const Config config_;  // 变量说明：config_ 表示当前对象持有的约束/代价配置。
  const size_t numFeet_;  // 变量说明：numFeet_ 表示num、双足。

  // impactProximityTrajectories 表示离触地事件多近，常用于把约束/代价在切换附近平滑开关。
  feet_array_t<std::vector<SplineCpg>> impactProximityTrajectories_;  // 变量说明：impactProximityTrajectories_ 表示impact、proximity、trajectories。
  // feetHeightTrajectories 保存每只脚每段摆动的 z 样条。
  feet_array_t<std::vector<SplineCpg>> feetHeightTrajectories_;  // 变量说明：feetHeightTrajectories_ 表示双足、height、trajectories。
  feet_array_t<std::vector<scalar_t>> feetHeightTrajectoriesEvents_;  // 变量说明：feetHeightTrajectoriesEvents_ 表示双足、height、trajectories、events。
};

// 函数说明：加载摆动脚、轨迹、配置，连接当前模块的数据流和控制逻辑。
SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName,
                                                           const std::string& fieldName = "swing_trajectory_config",  // 变量说明：fieldName 表示field、名称。
                                                           bool verbose = true);  // 变量说明：verbose 表示是否打印配置和初始化信息。

}  // namespace ocs2::humanoid

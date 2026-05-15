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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/constraint/JointLimitsSoftConstraint.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <memory>

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/penalties/PieceWisePolynomialBarrierPenalty.h>

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

class JointLimitsSoftConstraint final : public StateCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   *
   * @param positionlimits : {lower bounds, upper bounds} joint position limits.
   * @param barrierSettings : Settings for the position barrier penalty function.
   */
  JointLimitsSoftConstraint(std::pair<vector_t, vector_t> positionlimits,
                            ocs2::PieceWisePolynomialBarrierPenalty::Config barrierSettings,  // 变量说明：barrierSettings 表示barrier、配置。
                            const MpcRobotModelBase<scalar_t>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。

  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  JointLimitsSoftConstraint* clone() const override { return new JointLimitsSoftConstraint(*this); }

  // 函数说明：获取数值，连接当前模块的数据流和控制逻辑。
  scalar_t getValue(scalar_t time,
                    const vector_t& state,  // 变量说明：state 表示状态。
                    const ocs2::TargetTrajectories& targetTrajectories,  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
                    const ocs2::PreComputation& preComp) const override;  // 变量说明：preComp 表示pre、comp。

  // 函数说明：获取quadratic、approximation，连接当前模块的数据流和控制逻辑。
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time,
                                                                 const vector_t& state,  // 变量说明：state 表示状态。
                                                                 const ocs2::TargetTrajectories& targetTrajectories,  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
                                                                 const ocs2::PreComputation& preComp) const override;  // 变量说明：preComp 表示pre、comp。

  // 函数说明：获取数值，连接当前模块的数据流和控制逻辑。
  scalar_t getValue(const vector_t& jointPositions) const;
  // 函数说明：获取quadratic、approximation，连接当前模块的数据流和控制逻辑。
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(const vector_t& jointPositions) const;

  // 函数说明：设置gains，连接当前模块的数据流和控制逻辑。
  void setGains(const scalar_t& mu, const scalar_t& delta);
  // 函数说明：获取gains，连接当前模块的数据流和控制逻辑。
  void getGains(scalar_t& mu, scalar_t& delta) const;

  // 函数说明：判断主动关节，连接当前模块的数据流和控制逻辑。
  bool isActive(scalar_t time) const override { return isActive_; }
  // 函数说明：设置主动关节，连接当前模块的数据流和控制逻辑。
  void setActive(bool isActive) { isActive_ = isActive; }
  // 函数说明：获取主动关节，连接当前模块的数据流和控制逻辑。
  bool getActive() const { return isActive_; }

 private:
  JointLimitsSoftConstraint(const JointLimitsSoftConstraint& rhs);

  std::unique_ptr<PieceWisePolynomialBarrierPenalty> jointPositionPenaltyPtr_;  // 变量说明：jointPositionPenaltyPtr_ 表示关节、位置、penalty、指针。
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。
  std::pair<vector_t, vector_t> positionLimits_;
  scalar_t offset_;  // 变量说明：offset_ 表示关节软限位向内收缩的安全余量。
  bool isActive_ = true;  // 变量说明：isActive_ 表示该足端代价是否启用。
};

}  // namespace ocs2::humanoid

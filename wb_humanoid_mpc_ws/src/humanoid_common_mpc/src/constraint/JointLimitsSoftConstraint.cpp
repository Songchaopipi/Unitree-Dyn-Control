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

// 自动中文注释：src/humanoid_common_mpc/src/constraint/JointLimitsSoftConstraint.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/constraint/JointLimitsSoftConstraint.h"

#include "humanoid_common_mpc/common/ModelSettings.h"

#include <iostream>

namespace ocs2::humanoid {

// 函数说明：处理关节、limits、soft、约束，连接当前模块的数据流和控制逻辑。
JointLimitsSoftConstraint::JointLimitsSoftConstraint(std::pair<vector_t, vector_t> positionlimits,
                                                     ocs2::PieceWisePolynomialBarrierPenalty::Config barrierSettings,  // 变量说明：barrierSettings 表示barrier、配置。
                                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    // 函数说明：处理关节、位置、penalty、指针，连接当前模块的数据流和控制逻辑。
    : jointPositionPenaltyPtr_(new ocs2::PieceWisePolynomialBarrierPenalty(barrierSettings)),
      positionLimits_(positionlimits),
      mpcRobotModelPtr_(&mpcRobotModel),
      offset_(0.0) {
  // Obtain the offset at the middle joint angles. Just to compensate high negative costs when being far away from an infinite joint limit
  // offset_ = -getValue(0.5 * (positionLimits_.first + positionLimits_.second));
  std::cout << "joint limit offset: " << offset_ << std::endl;
}

// 函数说明：处理关节、limits、soft、约束，连接当前模块的数据流和控制逻辑。
JointLimitsSoftConstraint::JointLimitsSoftConstraint(const JointLimitsSoftConstraint& rhs)
    // 函数说明：处理关节、位置、penalty、指针，连接当前模块的数据流和控制逻辑。
    : jointPositionPenaltyPtr_(rhs.jointPositionPenaltyPtr_->clone()),
      positionLimits_(rhs.positionLimits_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_),
      offset_(rhs.offset_) {}

// 函数说明：获取数值，连接当前模块的数据流和控制逻辑。
scalar_t JointLimitsSoftConstraint::getValue(scalar_t time,  // 变量说明：JointLimitsSoftConstraint 表示关节、limits、soft、约束。
                                             const vector_t& state,  // 变量说明：state 表示状态。
                                             const ocs2::TargetTrajectories& targetTrajectories,  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
                                             const ocs2::PreComputation& preComp) const {  // 变量说明：preComp 表示pre、comp。
  return getValue(mpcRobotModelPtr_->getJointAngles(state));
}

// 函数说明：获取quadratic、approximation，连接当前模块的数据流和控制逻辑。
ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories, const ocs2::PreComputation& preComp) const {  // 变量说明：time 表示时间。
  return getQuadraticApproximation(mpcRobotModelPtr_->getJointAngles(state));
}

// 函数说明：获取数值，连接当前模块的数据流和控制逻辑。
scalar_t JointLimitsSoftConstraint::getValue(const vector_t& jointPositions) const {  // 变量说明：JointLimitsSoftConstraint 表示关节、limits、soft、约束。
  const vector_t upperBoundPositionOffset = positionLimits_.second - jointPositions;  // 变量说明：upperBoundPositionOffset 表示upper、边界、位置、offset。
  const vector_t lowerBoundPositionOffset = jointPositions - positionLimits_.first;  // 变量说明：lowerBoundPositionOffset 表示lower、边界、位置、offset。

  return upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getValue(0.0, hi); }).sum() +
         lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getValue(0.0, hi); }).sum() + offset_;
}

// 函数说明：获取quadratic、approximation，连接当前模块的数据流和控制逻辑。
ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(const vector_t& jointPositions) const {  // 变量说明：JointLimitsSoftConstraint 表示关节、limits、soft、约束。
  const vector_t upperBoundPositionOffset = positionLimits_.second - jointPositions;  // 变量说明：upperBoundPositionOffset 表示upper、边界、位置、offset。
  const vector_t lowerBoundPositionOffset = jointPositions - positionLimits_.first;  // 变量说明：lowerBoundPositionOffset 表示lower、边界、位置、offset。

  const size_t stateDim = mpcRobotModelPtr_->getStateDim();  // 变量说明：stateDim 表示状态、维度。
  const size_t jointDim = mpcRobotModelPtr_->getJointDim();  // 变量说明：jointDim 表示关节、维度。
  const size_t jointStartIndex = mpcRobotModelPtr_->getJointStartindex();  // 变量说明：jointStartIndex 表示关节、start、索引。

  ScalarFunctionQuadraticApproximation cost;  // 变量说明：cost 表示代价。
  cost.f = upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getValue(0.0, hi); }).sum() +
           lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getValue(0.0, hi); }).sum() + offset_;

  cost.dfdx = vector_t::Zero(stateDim);
  cost.dfdx.segment(jointStartIndex, jointDim) = lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) {
    return jointPositionPenaltyPtr_->getDerivative(0.0, hi);
  }) - upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getDerivative(0.0, hi); });

  cost.dfdxx = matrix_t::Zero(stateDim, stateDim);
  cost.dfdxx.block(jointStartIndex, jointStartIndex, jointDim, jointDim).diagonal() = lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) {
    return jointPositionPenaltyPtr_->getSecondDerivative(0.0, hi);
  }) + upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getSecondDerivative(0.0, hi); });

  return cost;
}

// 函数说明：设置gains，连接当前模块的数据流和控制逻辑。
void JointLimitsSoftConstraint::setGains(const scalar_t& mu, const scalar_t& delta) {  // 变量说明：JointLimitsSoftConstraint 表示关节、limits、soft、约束。
  jointPositionPenaltyPtr_->setConfig(ocs2::PieceWisePolynomialBarrierPenalty::Config(mu, delta));
}

// 函数说明：获取gains，连接当前模块的数据流和控制逻辑。
void JointLimitsSoftConstraint::getGains(scalar_t& mu, scalar_t& delta) const {  // 变量说明：JointLimitsSoftConstraint 表示关节、limits、soft、约束。
  ocs2::PieceWisePolynomialBarrierPenalty::Config config;  // 变量说明：config 表示从配置文件读取出的参数集合。
  jointPositionPenaltyPtr_->getConfig(config);
  mu = config.mu;
  delta = config.delta;
}

}  // namespace ocs2::humanoid

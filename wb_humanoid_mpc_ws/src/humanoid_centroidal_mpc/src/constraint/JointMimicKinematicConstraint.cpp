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

// 自动中文注释：src/humanoid_centroidal_mpc/src/constraint/JointMimicKinematicConstraint.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_centroidal_mpc/constraint/JointMimicKinematicConstraint.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

JointMimicKinematicConstraint::JointMimicKinematicConstraint(const MpcRobotModelBase<scalar_t>& mpcRobotModel, Config config)
    // 函数说明：处理状态、输入、约束，连接当前模块的数据流和控制逻辑。
    : StateInputConstraint(ConstraintOrder::Linear), mpcRobotModelPtr_(&mpcRobotModel), config_(config) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

JointMimicKinematicConstraint::JointMimicKinematicConstraint(const JointMimicKinematicConstraint& rhs)
    // 函数说明：处理状态、输入、约束，连接当前模块的数据流和控制逻辑。
    : StateInputConstraint(rhs), mpcRobotModelPtr_(rhs.mpcRobotModelPtr_), config_(rhs.config_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool JointMimicKinematicConstraint::isActive(scalar_t time) const {  // 变量说明：JointMimicKinematicConstraint 表示关节、mimic、kinematic、约束。
  return isActive_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t JointMimicKinematicConstraint::getValue(scalar_t time,  // 变量说明：JointMimicKinematicConstraint 表示关节、mimic、kinematic、约束。
                                                 const vector_t& state,  // 变量说明：state 表示状态。
                                                 const vector_t& input,  // 变量说明：input 表示输入。
                                                 const PreComputation& preComp) const {  // 变量说明：preComp 表示pre、comp。
  vector_t jointAngles = mpcRobotModelPtr_->getJointAngles(state);  // 变量说明：jointAngles 表示关节、angles。
  vector_t jointVelocities = mpcRobotModelPtr_->getJointVelocities(state, input);  // 变量说明：jointVelocities 表示关节、velocities。
  scalar_t posError = config_.multiplier * jointAngles[config_.parentJointIndex] - jointAngles[config_.childJointIndex];  // 变量说明：posError 表示位置、误差。
  scalar_t velError = config_.multiplier * jointVelocities[config_.parentJointIndex] - jointVelocities[config_.childJointIndex];  // 变量说明：velError 表示速度、误差。

  vector_t value(1);
  value << (config_.positionGain * posError + velError);

  return value;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation JointMimicKinematicConstraint::getLinearApproximation(scalar_t time,  // 变量说明：JointMimicKinematicConstraint 表示关节、mimic、kinematic、约束。
                                                                                        const vector_t& state,  // 变量说明：state 表示状态。
                                                                                        const vector_t& input,  // 变量说明：input 表示输入。
                                                                                        const PreComputation& preComp) const {  // 变量说明：preComp 表示pre、comp。
  VectorFunctionLinearApproximation linearApproximation =  // 变量说明：linearApproximation 表示linear、approximation。
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());

  linearApproximation.f = getValue(time, state, input, preComp);

  linearApproximation.dfdx(0, mpcRobotModelPtr_->getJointStartindex() + config_.parentJointIndex) =
      config_.positionGain * config_.multiplier;
  linearApproximation.dfdx(0, mpcRobotModelPtr_->getJointStartindex() + config_.childJointIndex) = -config_.positionGain;

  linearApproximation.dfdu(0, mpcRobotModelPtr_->getJointStartindex() + config_.parentJointIndex) = config_.multiplier;
  linearApproximation.dfdu(0, mpcRobotModelPtr_->getJointStartindex() + config_.childJointIndex) = -1;

  return linearApproximation;
}

}  // namespace ocs2::humanoid

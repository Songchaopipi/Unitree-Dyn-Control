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

// 自动中文注释：src/humanoid_common_mpc/src/constraint/ZeroWrenchConstraint.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/constraint/ZeroWrenchConstraint.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroWrenchConstraint::ZeroWrenchConstraint(const SwitchedModelReferenceManager& referenceManager,
                                           size_t contactPointIndex,  // 变量说明：contactPointIndex 表示接触、point、索引。
                                           const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    // 函数说明：处理状态、输入、约束，连接当前模块的数据流和控制逻辑。
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      contactPointIndex_(contactPointIndex),
      mpcRobotModelPtr_(&mpcRobotModel) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ZeroWrenchConstraint::ZeroWrenchConstraint(const ZeroWrenchConstraint& rhs)
    // 函数说明：处理状态、输入、约束，连接当前模块的数据流和控制逻辑。
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      contactPointIndex_(rhs.contactPointIndex_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ZeroWrenchConstraint::isActive(scalar_t time) const {  // 变量说明：ZeroWrenchConstraint 表示zero、力和力矩、约束。
  if (!isActive_) return false;
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ZeroWrenchConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {  // 变量说明：ZeroWrenchConstraint 表示zero、力和力矩、约束。
  return mpcRobotModelPtr_->getContactWrench(input, contactPointIndex_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ZeroWrenchConstraint::getLinearApproximation(scalar_t time,  // 变量说明：ZeroWrenchConstraint 表示zero、力和力矩、约束。
                                                                               const vector_t& state,  // 变量说明：state 表示状态。
                                                                               const vector_t& input,  // 变量说明：input 表示输入。
                                                                               const PreComputation& preComp) const {  // 变量说明：preComp 表示pre、comp。
  VectorFunctionLinearApproximation approx;  // 变量说明：approx 表示approx。
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(n_constraints, mpcRobotModelPtr_->getStateDim());
  approx.dfdu = matrix_t::Zero(n_constraints, mpcRobotModelPtr_->getInputDim());
  approx.dfdu.middleCols<n_constraints>(n_constraints * contactPointIndex_).diagonal() = vector_t::Ones(n_constraints);
  return approx;
}

}  // namespace ocs2::humanoid

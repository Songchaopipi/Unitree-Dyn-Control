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

// 自动中文注释：src/humanoid_common_mpc/src/cost/StateInputQuadraticCost.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/cost/StateInputQuadraticCost.h"

#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

#include <cmath>
#include <numbers>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputQuadraticCost::StateInputQuadraticCost(matrix_t Q,
                                                 matrix_t R,  // 变量说明：R 表示输入二次代价权重矩阵。
                                                 const SwitchedModelReferenceManager& referenceManager,  // 变量说明：referenceManager 表示参考、manager。
                                                 const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    // 函数说明：处理quadratic、状态、输入、代价，连接当前模块的数据流和控制逻辑。
    : QuadraticStateInputCost(std::move(Q), std::move(R)),
      referenceManagerPtr_(&referenceManager),
      pinInterface_(pinocchioInterface),
      mpcRobotModelPtr_(&mpcRobotModel) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

StateInputQuadraticCost::StateInputQuadraticCost(const StateInputQuadraticCost& rhs)
    // 函数说明：处理quadratic、状态、输入、代价，连接当前模块的数据流和控制逻辑。
    : QuadraticStateInputCost(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      pinInterface_(rhs.pinInterface_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::pair<vector_t, vector_t> StateInputQuadraticCost::getStateInputDeviation(scalar_t time,
                                                                              const vector_t& state,  // 变量说明：state 表示状态。
                                                                              const vector_t& input,  // 变量说明：input 表示输入。
                                                                              const TargetTrajectories& targetTrajectories) const {  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);  // 变量说明：contactFlags 表示接触、flags。
  vector_t xNominal = referenceManagerPtr_->getDesiredState(targetTrajectories, state, time);  // 变量说明：xNominal 表示x、nominal。

  // All reference stuff should eventually be moved out of here.
  const vector_t uNominal = weightCompensatingInput(pinInterface_, contactFlags, *mpcRobotModelPtr_);  // 变量说明：uNominal 表示u、nominal。

  return {state - xNominal, input - uNominal};
}

}  // namespace ocs2::humanoid

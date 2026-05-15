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

// 自动中文注释：src/humanoid_centroidal_mpc/src/initialization/CentroidalWeightCompInitializer.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_centroidal_mpc/initialization/CentroidalWeightCompInitializer.h"

#include "humanoid_centroidal_mpc/dynamics/DynamicsHelperFunctions.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalWeightCompInitializer::CentroidalWeightCompInitializer(CentroidalModelInfo info,
                                                                 const SwitchedModelReferenceManager& referenceManager,  // 变量说明：referenceManager 表示参考、manager。
                                                                 const CentroidalMpcRobotModel<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                                                 bool extendNormalizedMomentum)
    // 函数说明：处理模型信息，连接当前模块的数据流和控制逻辑。
    : info_(std::move(info)),
      referenceManagerPtr_(&referenceManager),
      mpcRobotModelPtr_(&mpcRobotModel),
      extendNormalizedMomentum_(extendNormalizedMomentum) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalWeightCompInitializer::CentroidalWeightCompInitializer(const CentroidalWeightCompInitializer& rhs)
    // 函数说明：处理模型信息，连接当前模块的数据流和控制逻辑。
    : info_(rhs.info_),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_),
      extendNormalizedMomentum_(rhs.extendNormalizedMomentum_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalWeightCompInitializer* CentroidalWeightCompInitializer::clone() const {  // 变量说明：CentroidalWeightCompInitializer 表示质心动力学、权重、comp、initializer。
  return new CentroidalWeightCompInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void CentroidalWeightCompInitializer::compute(
    scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {  // 变量说明：time 表示时间。
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);  // 变量说明：contactFlags 表示接触、flags。
  input = weightCompensatingInput(info_, contactFlags, *mpcRobotModelPtr_);
  nextState = state;
  if (!extendNormalizedMomentum_) {
    // 函数说明：获取normalized、momentum，连接当前模块的数据流和控制逻辑。
    centroidal_model::getNormalizedMomentum(nextState, info_).setZero();
  }
}

}  // namespace ocs2::humanoid

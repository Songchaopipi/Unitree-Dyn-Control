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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/initialization/WeightCompInitializer.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

// This class is used to initialize the input policy

class WeightCompInitializer final : public Initializer {
 public:
  /*
   * Constructor
   * @param [in] pinocchioInterface : The pinocchio model interface
   * @param [in] referenceManager : Switched system reference manager.
   * @param [in] extendNormalizedMomentum: If true, it extrapolates the normalized momenta; otherwise sets them to zero.
   */
  WeightCompInitializer(const PinocchioInterface& pinocchioInterface,
                        const SwitchedModelReferenceManager& referenceManager,  // 变量说明：referenceManager 表示参考、manager。
                        const MpcRobotModelBase<scalar_t>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。

  ~WeightCompInitializer() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  WeightCompInitializer* clone() const override;

  // 函数说明：计算compute，连接当前模块的数据流和控制逻辑。
  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  WeightCompInitializer(const WeightCompInitializer& rhs);

  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。
  const PinocchioInterface& pinocchioInterface_;  // 变量说明：pinocchioInterface_ 表示Pinocchio、interface。
  const SwitchedModelReferenceManager* referenceManagerPtr_;  // 变量说明：referenceManagerPtr_ 表示切换模型参考管理器指针。
};

}  // namespace ocs2::humanoid

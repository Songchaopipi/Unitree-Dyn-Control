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
// 自动中文注释：src/humanoid_centroidal_mpc/include/humanoid_centroidal_mpc/constraint/NormalVelocityConstraintCppAd.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "humanoid_common_mpc/constraint/EndEffectorKinematicsLinearVelConstraint.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

/**
 * Specializes the CppAd version of normal velocity constraint on an end-effector position and linear velocity.
 * Constructs the member EndEffectorKinematicsLinearVelConstraint object with number of constraints of 1.
 *
 * See also EndEffectorKinematicsLinearVelConstraint for the underlying computation.
 */
class NormalVelocityConstraintCppAd final : public StateInputConstraint {
 public:
  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager
   * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   */
  NormalVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                const EndEffectorKinematics<scalar_t>& endEffectorKinematics,  // 变量说明：endEffectorKinematics 表示end、effector、kinematics。
                                size_t contactPointIndex);  // 变量说明：contactPointIndex 表示接触、point、索引。

  ~NormalVelocityConstraintCppAd() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  NormalVelocityConstraintCppAd* clone() const override { return new NormalVelocityConstraintCppAd(*this); }

  // 函数说明：判断主动关节，连接当前模块的数据流和控制逻辑。
  bool isActive(scalar_t time) const override;
  // 函数说明：设置主动关节，连接当前模块的数据流和控制逻辑。
  void setActive(bool isActive) override { isActive_ = isActive; }
  // 函数说明：获取主动关节，连接当前模块的数据流和控制逻辑。
  bool getActive() const override { return isActive_; }
  // 函数说明：获取num、constraints，连接当前模块的数据流和控制逻辑。
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  // 函数说明：获取数值，连接当前模块的数据流和控制逻辑。
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  // 函数说明：获取linear、approximation，连接当前模块的数据流和控制逻辑。
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time,
                                                           const vector_t& state,  // 变量说明：state 表示状态。
                                                           const vector_t& input,  // 变量说明：input 表示输入。
                                                           const PreComputation& preComp) const override;  // 变量说明：preComp 表示pre、comp。

 private:
  NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;  // 变量说明：referenceManagerPtr_ 表示切换模型参考管理器指针。
  std::unique_ptr<EndEffectorKinematicsLinearVelConstraint> eeLinearConstraintPtr_;  // 变量说明：eeLinearConstraintPtr_ 表示ee、linear、约束、指针。
  const size_t contactPointIndex_;  // 变量说明：contactPointIndex_ 表示接触、point、索引。
  bool isActive_ = true;  // 变量说明：isActive_ 表示该足端代价是否启用。
};

}  // namespace ocs2::humanoid

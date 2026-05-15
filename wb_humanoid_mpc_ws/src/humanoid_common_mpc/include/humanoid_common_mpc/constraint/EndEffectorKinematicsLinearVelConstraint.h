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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/constraint/EndEffectorKinematicsLinearVelConstraint.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace ocs2::humanoid {

/**
 * Defines a linear constraint on an end-effector position (xee) and linear velocity (vee).
 * g(xee, vee) = Ax * xee + Av * vee + b
 * - For defining constraint of type g(xee), set Av to matrix_t(0, 0)
 * - For defining constraint of type g(vee), set Ax to matrix_t(0, 0)
 */
class EndEffectorKinematicsLinearVelConstraint final : public StateInputConstraint {
 public:
  /**
   * Coefficients of the linear constraints of the form:
   * g(xee, vee) = Ax * xee + Av * vee + b
   */
  struct Config {
    vector_t b;  // 变量说明：b 表示b。
    matrix_t Ax;  // 变量说明：Ax 表示ax。
    matrix_t Av;  // 变量说明：Av 表示av。
  };

  /**
   * Constructor
   * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
   * @param [in] numConstraints: The number of constraints {1, 2, 3}
   * @param [in] config: The constraint coefficients, g(xee, vee) = Ax * xee + Av * vee + b
   */
  EndEffectorKinematicsLinearVelConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                           size_t numConstraints,  // 变量说明：numConstraints 表示num、constraints。
                                           Config config = Config());  // 变量说明：config 表示从配置文件读取出的参数集合。

  ~EndEffectorKinematicsLinearVelConstraint() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  EndEffectorKinematicsLinearVelConstraint* clone() const override { return new EndEffectorKinematicsLinearVelConstraint(*this); }

  /** Sets a new constraint coefficients. */
  void configure(Config&& config);
  /** Sets a new constraint coefficients. */
  void configure(const Config& config) { this->configure(Config(config)); }

  /** Gets the underlying end-effector kinematics interface. */
  EndEffectorKinematics<scalar_t>& getEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }

  // 函数说明：获取num、constraints，连接当前模块的数据流和控制逻辑。
  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }
  // 函数说明：获取数值，连接当前模块的数据流和控制逻辑。
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  // 函数说明：获取linear、approximation，连接当前模块的数据流和控制逻辑。
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time,
                                                           const vector_t& state,  // 变量说明：state 表示状态。
                                                           const vector_t& input,  // 变量说明：input 表示输入。
                                                           const PreComputation& preComp) const override;  // 变量说明：preComp 表示pre、comp。

 private:
  EndEffectorKinematicsLinearVelConstraint(const EndEffectorKinematicsLinearVelConstraint& rhs);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;  // 变量说明：endEffectorKinematicsPtr_ 表示end、effector、kinematics、指针。
  const size_t numConstraints_;  // 变量说明：numConstraints_ 表示num、constraints。
  Config config_;  // 变量说明：config_ 表示当前对象持有的约束/代价配置。
};

}  // namespace ocs2::humanoid

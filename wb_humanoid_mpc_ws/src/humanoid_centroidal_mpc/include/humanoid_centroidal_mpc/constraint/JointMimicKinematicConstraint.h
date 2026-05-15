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
// 自动中文注释：src/humanoid_centroidal_mpc/include/humanoid_centroidal_mpc/constraint/JointMimicKinematicConstraint.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

class JointMimicKinematicConstraint final : public StateInputConstraint {
 public:
  struct Config {
    Config() = delete;
    // 函数说明：处理config，连接当前模块的数据流和控制逻辑。
    explicit Config(const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                    std::string parentJointNameParam,  // 变量说明：parentJointNameParam 表示parent、关节、名称、param。
                    std::string childJointNameParam,  // 变量说明：childJointNameParam 表示child、关节、名称、param。
                    scalar_t multiplierParam,  // 变量说明：multiplierParam 表示multiplier、param。
                    scalar_t positionGainParam)
        // 函数说明：处理parent、关节、名称，连接当前模块的数据流和控制逻辑。
        : parentJointName(parentJointNameParam),
          childJointName(childJointNameParam),
          parentJointIndex(mpcRobotModel.getJointIndex(parentJointNameParam)),
          childJointIndex(mpcRobotModel.getJointIndex(childJointNameParam)),
          multiplier(multiplierParam),
          positionGain(positionGainParam) {
      assert(positionGain > 0.0);
    }

    const std::string parentJointName;  // 变量说明：parentJointName 表示parent、关节、名称。
    const std::string childJointName;  // 变量说明：childJointName 表示child、关节、名称。
    const size_t parentJointIndex;  // 变量说明：parentJointIndex 表示parent、关节、索引。
    const size_t childJointIndex;  // 变量说明：childJointIndex 表示child、关节、索引。
    const scalar_t multiplier;  // q_child = multiplier* q_parent
    scalar_t positionGain;  // 变量说明：positionGain 表示位置、增益。
  };

  /**
   * @param [in] contactPointIndex : The 3 DoF contact index.
   */
  JointMimicKinematicConstraint(const MpcRobotModelBase<scalar_t>& mpcRobotModel, Config config);

  ~JointMimicKinematicConstraint() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  JointMimicKinematicConstraint* clone() const override { return new JointMimicKinematicConstraint(*this); }

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
  JointMimicKinematicConstraint(const JointMimicKinematicConstraint& rhs);

  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。
  const Config config_;  // 变量说明：config_ 表示当前对象持有的约束/代价配置。

  bool isActive_ = true;  // 变量说明：isActive_ 表示该足端代价是否启用。
};

}  // namespace ocs2::humanoid

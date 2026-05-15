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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/HumanoidPreComputation.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/constraint/EndEffectorKinematicsLinearVelConstraint.h"

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h"

namespace ocs2::humanoid {

/** Callback for caching and reference update */
class HumanoidPreComputation : public PreComputation {
 public:
  HumanoidPreComputation(PinocchioInterface pinocchioInterface,
                         const SwingTrajectoryPlanner& swingTrajectoryPlanner,  // 变量说明：swingTrajectoryPlanner 表示摆动脚、轨迹、planner。
                         const MpcRobotModelBase<scalar_t>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
  // 函数说明：处理~humanoid、pre、computation，连接当前模块的数据流和控制逻辑。
  virtual ~HumanoidPreComputation() override = default;

  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  virtual HumanoidPreComputation* clone() const override;  // 变量说明：HumanoidPreComputation 表示humanoid、pre、computation。

  // 函数说明：处理request，连接当前模块的数据流和控制逻辑。
  virtual void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。
  // 函数说明：获取rworld、to、contacts，连接当前模块的数据流和控制逻辑。
  const matrix3_t& getRWorldToContacts(size_t contactIndex) const { return R_world_to_contacts_[contactIndex]; }

  // 函数说明：获取ee、法向、速度、约束、configs，连接当前模块的数据流和控制逻辑。
  const std::vector<EndEffectorKinematicsLinearVelConstraint::Config>& getEeNormalVelocityConstraintConfigs() const {
    return eeNormalVelConConfigs_;
  }
  // 函数说明：获取足端、参考、height，连接当前模块的数据流和控制逻辑。
  scalar_t getFootReferenceHeight(size_t contactIndex) const { return footHeightReferences_[contactIndex]; }

  // 函数说明：获取Pinocchio、interface，连接当前模块的数据流和控制逻辑。
  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  // 函数说明：获取Pinocchio、interface，连接当前模块的数据流和控制逻辑。
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 protected:
  HumanoidPreComputation(const HumanoidPreComputation& rhs);

  void updatePinocchioModelKinematics(const vector_t& generalizedCoordinates);

  PinocchioInterface pinocchioInterface_;  // 变量说明：pinocchioInterface_ 表示Pinocchio、interface。
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;  // 变量说明：swingTrajectoryPlannerPtr_ 表示摆动脚、轨迹、planner、指针。
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。

  std::vector<matrix3_t> R_world_to_contacts_;  // 变量说明：R_world_to_contacts_ 表示r、world、to、contacts。

  std::vector<EndEffectorKinematicsLinearVelConstraint::Config> eeNormalVelConConfigs_;  // 变量说明：eeNormalVelConConfigs_ 表示ee、法向、速度、con、configs。
  std::vector<scalar_t> footHeightReferences_;  // 变量说明：footHeightReferences_ 表示足端、height、references。
};

}  // namespace ocs2::humanoid

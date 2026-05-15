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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/constraint/FootCollisionConstraint.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <pinocchio/fwd.hpp>

#include <ocs2_core/constraint/StateConstraintCppAd.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/algorithm/frames.hpp>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

/**
 * Implements the constraint h(t,x,u) >= 0 to prevent collisions of the feet.
 */

class FootCollisionConstraint final : public StateConstraintCppAd {
 public:
  struct Config {
    // Foot and ankle
    std::string leftAnkleFrame;  // 变量说明：leftAnkleFrame 表示左侧、ankle、坐标系/帧。
    std::string rightAnkleFrame;  // 变量说明：rightAnkleFrame 表示右侧、ankle、坐标系/帧。

    std::string leftFootCenterFrame{"foot_l_contact"};  // 变量说明：leftFootCenterFrame 表示左侧、足端、center、坐标系/帧。
    std::string rightFootCenterFrame{"foot_r_contact"};  // 变量说明：rightFootCenterFrame 表示右侧、足端、center、坐标系/帧。

    std::string leftFootFrame1{"foot_l_contact_collision_p_1"};  // 变量说明：leftFootFrame1 表示左侧、足端、frame1。
    std::string rightFootFrame1{"foot_r_contact_collision_p_1"};  // 变量说明：rightFootFrame1 表示右侧、足端、frame1。

    std::string leftFootFrame2{"foot_l_contact_collision_p_2"};  // 变量说明：leftFootFrame2 表示左侧、足端、frame2。
    std::string rightFootFrame2{"foot_r_contact_collision_p_2"};  // 变量说明：rightFootFrame2 表示右侧、足端、frame2。

    scalar_t footCollisionSphereRadius;  // 变量说明：footCollisionSphereRadius 表示足端、collision、sphere、radius。

    // Knee
    std::string leftKneeFrame;  // 变量说明：leftKneeFrame 表示左侧、knee、坐标系/帧。
    std::string rightKneeFrame;  // 变量说明：rightKneeFrame 表示右侧、knee、坐标系/帧。
    scalar_t kneeCollisionSphereRadius;  // 变量说明：kneeCollisionSphereRadius 表示knee、collision、sphere、radius。
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FootCollisionConstraint(const SwitchedModelReferenceManager& referenceManager,
                          const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                          const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                          const Config& config,  // 变量说明：config 表示从配置文件读取出的参数集合。
                          std::string costName,  // 变量说明：costName 表示自动微分代价库名称。
                          const ModelSettings& modelSettings);  // 变量说明：modelSettings 表示机器人关节和接触配置。

  ~FootCollisionConstraint() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  FootCollisionConstraint* clone() const override { return new FootCollisionConstraint(*this); }

  // 函数说明：判断主动关节，连接当前模块的数据流和控制逻辑。
  bool isActive(scalar_t time) const override;
  // 函数说明：获取主动关节，连接当前模块的数据流和控制逻辑。
  bool getActive() const { return isActive_; }
  // 函数说明：设置主动关节，连接当前模块的数据流和控制逻辑。
  void setActive(bool active) { isActive_ = active; }

  // 函数说明：获取num、constraints，连接当前模块的数据流和控制逻辑。
  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; };

  // 函数说明：获取parameters，连接当前模块的数据流和控制逻辑。
  vector_t getParameters(scalar_t time, const PreComputation& preComputation) const override {
    vector_t parameters(2);
    parameters << cfg_.footCollisionSphereRadius, cfg_.kneeCollisionSphereRadius;
    return parameters;
  };

  // 函数说明：设置sphere、radii，连接当前模块的数据流和控制逻辑。
  void setSphereRadii(scalar_t footCollisionSphereRadius, scalar_t kneeCollisionSphereRadius) {
    cfg_.footCollisionSphereRadius = footCollisionSphereRadius;
    cfg_.kneeCollisionSphereRadius = kneeCollisionSphereRadius;
  }

  // 函数说明：获取sphere、radii，连接当前模块的数据流和控制逻辑。
  void getSphereRadii(scalar_t& footCollisionSphereRadius, scalar_t& kneeCollisionSphereRadius) const {
    footCollisionSphereRadius = cfg_.footCollisionSphereRadius;
    kneeCollisionSphereRadius = cfg_.kneeCollisionSphereRadius;
  }

  static Config loadFootCollisionConstraintConfig(const std::string taskFile, bool verbose = false);

 private:
  // 函数说明：处理约束、function，连接当前模块的数据流和控制逻辑。
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override;

  FootCollisionConstraint(const FootCollisionConstraint& other);

  const SwitchedModelReferenceManager* referenceManagerPtr_;  // 变量说明：referenceManagerPtr_ 表示切换模型参考管理器指针。
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;  // 变量说明：pinocchioInterfaceCppAd_ 表示自动微分版本的 Pinocchio 接口。
  const MpcRobotModelBase<ad_scalar_t>* const mpcRobotModelPtr_;  // 变量说明：const 表示const。
  Config cfg_;  // 变量说明：cfg_ 表示cfg。

  const size_t numConstraints_ = 16;  // 变量说明：numConstraints_ 表示num、constraints。
  bool isActive_ = true;  // 变量说明：isActive_ 表示该足端代价是否启用。
};

}  // namespace ocs2::humanoid

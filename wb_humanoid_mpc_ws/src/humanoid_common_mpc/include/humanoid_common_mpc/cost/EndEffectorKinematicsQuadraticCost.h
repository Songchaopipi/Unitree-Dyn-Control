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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/cost/EndEffectorKinematicsQuadraticCost.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <pinocchio/algorithm/frames.hpp>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"

#include "humanoid_common_mpc/cost/EndEffectorKinematicCostHelpers.h"

namespace ocs2::humanoid {

class EndEffectorKinematicsQuadraticCost : public ocs2::StateInputCostGaussNewtonAd {
 public:
  EndEffectorKinematicsQuadraticCost(EndEffectorKinematicsWeights weights,
                                     const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                     const EndEffectorKinematics<scalar_t>& endEffectorKinematics,  // 变量说明：endEffectorKinematics 表示end、effector、kinematics。
                                     const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,  // 变量说明：mpcRobotModelAD 表示自动微分版本的 MPC 机器人模型。
                                     std::string endEffectorName,  // 变量说明：endEffectorName 表示end、effector、名称。
                                     const ModelSettings& modelSettings);  // 变量说明：modelSettings 表示机器人关节和接触配置。

  ~EndEffectorKinematicsQuadraticCost() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  EndEffectorKinematicsQuadraticCost* clone() const override { return new EndEffectorKinematicsQuadraticCost(*this); }

  // 函数说明：获取parameters，连接当前模块的数据流和控制逻辑。
  virtual vector_t getParameters(scalar_t time,  // 变量说明：vector_t 表示向量、t。
                                 const TargetTrajectories& targetTrajectories,  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
                                 const PreComputation& preComputation) const override;  // 变量说明：preComputation 表示 OCS2 预计算缓存，本函数当前不直接使用。

  // 函数说明：判断主动关节，连接当前模块的数据流和控制逻辑。
  bool isActive(scalar_t time) const override { return isActive_; }
  // 函数说明：设置主动关节，连接当前模块的数据流和控制逻辑。
  void setActive(bool active) { isActive_ = active; }
  // 函数说明：获取主动关节，连接当前模块的数据流和控制逻辑。
  bool getActive() const { return isActive_; }

  // 函数说明：获取weights，连接当前模块的数据流和控制逻辑。
  static EndEffectorKinematicsWeights getWeights(const std::string& taskFile, const std::string prefix, bool verbose = false);

  // 函数说明：获取weights，连接当前模块的数据流和控制逻辑。
  void getWeights(vector12_t& weights) const { weights = sqrtWeights_.cwiseProduct(sqrtWeights_); }
  // 函数说明：设置weights，连接当前模块的数据流和控制逻辑。
  void setWeights(const vector12_t& weights) { sqrtWeights_ = weights.cwiseSqrt(); }

 protected:
  EndEffectorKinematicsQuadraticCost(const EndEffectorKinematicsQuadraticCost& other);

  // 函数说明：获取参考、代价、element，连接当前模块的数据流和控制逻辑。
  static EndEffectorKinematicsCostElement<scalar_t> getReferenceCostElement(const vector_t& state,
                                                                            const vector_t& input,  // 变量说明：input 表示输入。
                                                                            const EndEffectorKinematics<scalar_t>& endEffectorKinematics);  // 变量说明：endEffectorKinematics 表示end、effector、kinematics。

  // 函数说明：处理代价、向量、function，连接当前模块的数据流和控制逻辑。
  ad_vector_t costVectorFunction(ad_scalar_t time,
                                 const ad_vector_t& state,  // 变量说明：state 表示状态。
                                 const ad_vector_t& input,  // 变量说明：input 表示输入。
                                 const ad_vector_t& parameters) override;  // 变量说明：parameters 表示传给自动微分代价的参数向量。

  vector12_t sqrtWeights_;  // 变量说明：sqrtWeights_ 表示代价权重的平方根，用于 Gauss-Newton residual 缩放。
  size_t n_parameters_ = 25;  // 变量说明：n_parameters_ 表示n、parameters。
  pinocchio::FrameIndex frameID_;  // 变量说明：frameID_ 表示 Pinocchio 中足端 contact frame 的编号。
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;  // 变量说明：pinocchioInterfaceCppAd_ 表示自动微分版本的 Pinocchio 接口。
  const std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;  // 变量说明：endEffectorKinematicsPtr_ 表示end、effector、kinematics、指针。
  std::unique_ptr<MpcRobotModelBase<ad_scalar_t>> mpcRobotModelADPtr;  // 变量说明：mpcRobotModelADPtr 表示MPC、机器人、模型、adptr。
  bool isActive_ = true;  // 变量说明：isActive_ 表示该足端代价是否启用。
};

// 函数说明：加载weights、from、file，连接当前模块的数据流和控制逻辑。
EndEffectorKinematicsWeights loadWeightsFromFile(const std::string& filename, const std::string& fieldname, bool verbose = true);

}  // namespace ocs2::humanoid

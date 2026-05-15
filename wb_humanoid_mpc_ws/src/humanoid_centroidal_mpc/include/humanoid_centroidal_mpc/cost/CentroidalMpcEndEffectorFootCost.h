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
// 自动中文注释：src/humanoid_centroidal_mpc/include/humanoid_centroidal_mpc/cost/CentroidalMpcEndEffectorFootCost.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <pinocchio/algorithm/frames.hpp>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>

#include "humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/cost/EndEffectorKinematicCostHelpers.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

class CentroidalMpcEndEffectorFootCost final : public StateInputCostGaussNewtonAd {
 public:
  CentroidalMpcEndEffectorFootCost(const SwitchedModelReferenceManager& referenceManager,
                                   EndEffectorKinematicsWeights weights,  // 变量说明：weights 表示代价/任务权重。
                                   const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                   const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,  // 变量说明：mpcRobotModelAD 表示自动微分版本的 MPC 机器人模型。
                                   size_t contactIndex,  // 变量说明：contactIndex 表示左右脚接触点编号。
                                   std::string costName,  // 变量说明：costName 表示自动微分代价库名称。
                                   const ModelSettings& modelSettings);  // 变量说明：modelSettings 表示机器人关节和接触配置。

  ~CentroidalMpcEndEffectorFootCost() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  CentroidalMpcEndEffectorFootCost* clone() const override { return new CentroidalMpcEndEffectorFootCost(*this); }

  // 函数说明：获取parameters，连接当前模块的数据流和控制逻辑。
  vector_t getParameters(scalar_t time, const TargetTrajectories& targetTrajectories, const PreComputation& preComputation) const override;

  // 函数说明：判断主动关节，连接当前模块的数据流和控制逻辑。
  bool isActive(scalar_t time) const override {
    if (!isActive_) return false;
    return !referenceManagerPtr_->isInContact(time, contactIndex_);
  }

  // 函数说明：设置主动关节，连接当前模块的数据流和控制逻辑。
  void setActive(bool active) { isActive_ = active; }
  // 函数说明：获取主动关节，连接当前模块的数据流和控制逻辑。
  bool getActive() const { return isActive_; }

  // 函数说明：设置weights，连接当前模块的数据流和控制逻辑。
  void setWeights(const vector12_t& weights) { sqrtWeights_ = weights.cwiseSqrt(); }
  // 函数说明：获取weights，连接当前模块的数据流和控制逻辑。
  void getWeights(vector12_t& weights) const { weights = sqrtWeights_.cwiseProduct(sqrtWeights_); }

 private:
  CentroidalMpcEndEffectorFootCost(const CentroidalMpcEndEffectorFootCost& other);

  // 函数说明：处理代价、向量、function，连接当前模块的数据流和控制逻辑。
  ad_vector_t costVectorFunction(ad_scalar_t time,
                                 const ad_vector_t& state,  // 变量说明：state 表示状态。
                                 const ad_vector_t& input,  // 变量说明：input 表示输入。
                                 const ad_vector_t& parameters) override;  // 变量说明：parameters 表示传给自动微分代价的参数向量。

  const SwitchedModelReferenceManager* referenceManagerPtr_;  // 变量说明：referenceManagerPtr_ 表示切换模型参考管理器指针。

  vector12_t sqrtWeights_;  // 变量说明：sqrtWeights_ 表示代价权重的平方根，用于 Gauss-Newton residual 缩放。
  bool isActive_ = true;  // 变量说明：isActive_ 表示该足端代价是否启用。

  size_t contactIndex_;  // 变量说明：contactIndex_ 表示当前足端代价对应的接触点编号。
  const pinocchio::FrameIndex frameID_;  // 变量说明：frameID_ 表示 Pinocchio 中足端 contact frame 的编号。
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;  // 变量说明：pinocchioInterfaceCppAd_ 表示自动微分版本的 Pinocchio 接口。
  std::unique_ptr<MpcRobotModelBase<ad_scalar_t>> mpcRobotModelAdPtr_;  // 变量说明：mpcRobotModelAdPtr_ 表示自动微分机器人模型指针。
};

}  // namespace ocs2::humanoid

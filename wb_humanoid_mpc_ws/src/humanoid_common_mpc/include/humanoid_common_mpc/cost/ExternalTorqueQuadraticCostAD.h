/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/cost/ExternalTorqueQuadraticCostAD.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <pinocchio/algorithm/frames.hpp>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

#include "humanoid_common_mpc/cost/EndEffectorKinematicCostHelpers.h"

namespace ocs2::humanoid {

class ExternalTorqueQuadraticCostAD : public ocs2::StateInputCostGaussNewtonAd {
 public:
  struct Config {
    std::vector<std::string> activeJointNames;  // 变量说明：activeJointNames 表示主动关节、关节、names。
    vector_t weights;  // 变量说明：weights 表示代价/任务权重。
  };

  ExternalTorqueQuadraticCostAD(size_t endEffectorIndex,
                                Config config,  // 变量说明：config 表示从配置文件读取出的参数集合。
                                const SwitchedModelReferenceManager& referenceManager,  // 变量说明：referenceManager 表示参考、manager。
                                const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,  // 变量说明：mpcRobotModelAD 表示自动微分版本的 MPC 机器人模型。
                                const ModelSettings& modelSettings);  // 变量说明：modelSettings 表示机器人关节和接触配置。

  ~ExternalTorqueQuadraticCostAD() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  ExternalTorqueQuadraticCostAD* clone() const override { return new ExternalTorqueQuadraticCostAD(*this); }

  // 函数说明：获取parameters，连接当前模块的数据流和控制逻辑。
  virtual vector_t getParameters(scalar_t time,  // 变量说明：vector_t 表示向量、t。
                                 const TargetTrajectories& targetTrajectories,  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。
                                 const PreComputation& preComputation) const override;  // 变量说明：preComputation 表示 OCS2 预计算缓存，本函数当前不直接使用。

  // 函数说明：判断主动关节，连接当前模块的数据流和控制逻辑。
  bool isActive(scalar_t time) const override;
  // 函数说明：设置主动关节，连接当前模块的数据流和控制逻辑。
  void setActive(bool active) { isActive_ = active; }
  // 函数说明：获取主动关节，连接当前模块的数据流和控制逻辑。
  bool getActive() const { return isActive_; }

  // 函数说明：获取weights，连接当前模块的数据流和控制逻辑。
  void getWeights(vector_t& weights) const { weights = sqrtWeights_.cwiseProduct(sqrtWeights_); }
  // 函数说明：设置weights，连接当前模块的数据流和控制逻辑。
  void setWeights(const vector_t& weights) { sqrtWeights_ = weights.cwiseSqrt(); }

  // 函数说明：加载config、from、file，连接当前模块的数据流和控制逻辑。
  static ExternalTorqueQuadraticCostAD::Config loadConfigFromFile(const std::string& filename,
                                                                  const std::string& fieldname,  // 变量说明：fieldname 表示配置文件中对应代价/约束块的字段名。
                                                                  bool verbose = true);  // 变量说明：verbose 表示是否打印配置和初始化信息。

 protected:
  ExternalTorqueQuadraticCostAD(const ExternalTorqueQuadraticCostAD& other);

  // 函数说明：处理代价、向量、function，连接当前模块的数据流和控制逻辑。
  ad_vector_t costVectorFunction(ad_scalar_t time,
                                 const ad_vector_t& state,  // 变量说明：state 表示状态。
                                 const ad_vector_t& input,  // 变量说明：input 表示输入。
                                 const ad_vector_t& parameters) override;  // 变量说明：parameters 表示传给自动微分代价的参数向量。

  const size_t contactPointIndex_;  // 变量说明：contactPointIndex_ 表示接触、point、索引。
  const pinocchio::FrameIndex frameID_;  // 变量说明：frameID_ 表示 Pinocchio 中足端 contact frame 的编号。
  const size_t n_parameters_;  // 变量说明：n_parameters_ 表示n、parameters。
  vector_t sqrtWeights_;  // 变量说明：sqrtWeights_ 表示代价权重的平方根，用于 Gauss-Newton residual 缩放。
  const std::vector<std::string> activeJointNames_;  // 变量说明：activeJointNames_ 表示主动关节、关节、names。
  const SwitchedModelReferenceManager* referenceManagerPtr_;  // 变量说明：referenceManagerPtr_ 表示切换模型参考管理器指针。
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;  // 变量说明：pinocchioInterfaceCppAd_ 表示自动微分版本的 Pinocchio 接口。
  const std::unique_ptr<MpcRobotModelBase<ad_scalar_t>> mpcRobotModelADPtr;  // 变量说明：mpcRobotModelADPtr 表示MPC、机器人、模型、adptr。
  bool isActive_ = true;  // 变量说明：isActive_ 表示该足端代价是否启用。
};

}  // namespace ocs2::humanoid

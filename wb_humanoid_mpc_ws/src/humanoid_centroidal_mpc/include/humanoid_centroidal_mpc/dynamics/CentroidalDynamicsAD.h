
#pragma once

#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/ModelSettings.h"

namespace ocs2::humanoid {

class CentroidalDynamicsAD final : public SystemDynamicsBase {
 public:
  CentroidalDynamicsAD(const PinocchioInterface& pinocchioInterface,
                       const CentroidalModelInfo& info,  // 变量说明：info 表示模型信息。
                       const std::string& modelName,  // 变量说明：modelName 表示模型、名称。
                       const ModelSettings& modelSettings);  // 变量说明：modelSettings 表示机器人关节和接触配置。

  ~CentroidalDynamicsAD() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  CentroidalDynamicsAD* clone() const override { return new CentroidalDynamicsAD(*this); }

  // 函数说明：计算flow、map，连接当前模块的数据流和控制逻辑。
  vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) override;
  // 函数说明：处理linear、approximation，连接当前模块的数据流和控制逻辑。
  VectorFunctionLinearApproximation linearApproximation(scalar_t time,
                                                        const vector_t& state,  // 变量说明：state 表示状态。
                                                        const vector_t& input,  // 变量说明：input 表示输入。
                                                        const PreComputation& preComp) override;  // 变量说明：preComp 表示pre、comp。

 private:
  CentroidalDynamicsAD(const CentroidalDynamicsAD& rhs) = default;

  PinocchioCentroidalDynamicsAD pinocchioCentroidalDynamicsAd_;  // 变量说明：pinocchioCentroidalDynamicsAd_ 表示Pinocchio、质心动力学、dynamics、自动微分。
};

}  // namespace ocs2::humanoid

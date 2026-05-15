#include "humanoid_centroidal_mpc/dynamics/CentroidalDynamicsAD.h"

namespace ocs2::humanoid {

CentroidalDynamicsAD::CentroidalDynamicsAD(const PinocchioInterface& pinocchioInterface,
                                           const CentroidalModelInfo& info,  // 变量说明：info 表示模型信息。
                                           const std::string& modelName,  // 变量说明：modelName 表示模型、名称。
                                           const ModelSettings& modelSettings)
    // 函数说明：处理Pinocchio、质心动力学、dynamics、自动微分，连接当前模块的数据流和控制逻辑。
    : pinocchioCentroidalDynamicsAd_(pinocchioInterface,
                                     info,
                                     modelName,
                                     modelSettings.modelFolderCppAd,
                                     modelSettings.recompileLibrariesCppAd,
                                     modelSettings.verboseCppAd) {}


vector_t CentroidalDynamicsAD::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) {  // 变量说明：CentroidalDynamicsAD 表示质心动力学、dynamics、自动微分。
  return pinocchioCentroidalDynamicsAd_.getValue(time, state, input);
}

VectorFunctionLinearApproximation CentroidalDynamicsAD::linearApproximation(scalar_t time,  // 变量说明：CentroidalDynamicsAD 表示质心动力学、dynamics、自动微分。
                                                                            const vector_t& state,  // 变量说明：state 表示状态。
                                                                            const vector_t& input,  // 变量说明：input 表示输入。
                                                                            const PreComputation& preComp) {  // 变量说明：preComp 表示pre、comp。
  return pinocchioCentroidalDynamicsAd_.getLinearApproximation(time, state, input);
}

}  // namespace ocs2::humanoid

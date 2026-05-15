#pragma once
#include <functional>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "ocs2_centroidal_model/CentroidalModelInfo.h"

#include <humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h>
#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/common/MpcRobotModelBase.h>
#include <humanoid_common_mpc/common/Types.h>

namespace ocs2::humanoid {

class CentroidalMpcTargetTrajectoriesCalculator : public TargetTrajectoriesCalculatorBase {
 public:
  CentroidalMpcTargetTrajectoriesCalculator(const std::string& referenceFile,
                                            const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                            PinocchioInterface pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                            const CentroidalModelInfo& info,  // 变量说明：info 表示模型信息。
                                            scalar_t mpcHorizon);  // 变量说明：mpcHorizon 表示MPC、预测时域。

  CentroidalMpcTargetTrajectoriesCalculator(const CentroidalMpcTargetTrajectoriesCalculator& rhs) = delete;

  /**
   * Converts command line to TargetTrajectories.
   * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw] defined in pelvis frame
   * @param [in] observation : the current observation
   */
  TargetTrajectories commandedPositionToTargetTrajectories(const vector4_t& commadLineTarget,
                                                           scalar_t initTime,  // 变量说明：initTime 表示init、时间。
                                                           const vector_t& initState) override;  // 变量说明：initState 表示init、状态。

  /**
   * Converts desired velocities to TargetTrajectories.
   * @param [in] commandedVelocities : [v_x, v_y, v_yaw] defined in pelvis frame
   * @param [in] observation : the current observation
   */
  TargetTrajectories commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,
                                                           scalar_t initTime,  // 变量说明：initTime 表示init、时间。
                                                           const vector_t& initState) override;  // 变量说明：initState 表示init、状态。

 private:
  PinocchioInterface pinocchioInterface_;  // 变量说明：pinocchioInterface_ 表示Pinocchio、interface。
  const CentroidalModelInfo& info_;  // 变量说明：info_ 表示模型信息。
  const scalar_t mass_;  // 变量说明：mass_ 表示 Pinocchio 模型计算出的机器人总质量。
};

}  // namespace ocs2::humanoid

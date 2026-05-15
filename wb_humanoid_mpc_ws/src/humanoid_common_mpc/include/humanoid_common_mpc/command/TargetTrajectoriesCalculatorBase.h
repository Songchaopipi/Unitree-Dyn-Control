
#pragma once
#include <functional>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/common/Types.h>
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"

namespace ocs2::humanoid {

class TargetTrajectoriesCalculatorBase {
 public:
  TargetTrajectoriesCalculatorBase(const std::string& referenceFile,

                                   const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                   scalar_t mpcHorizon);  // 变量说明：mpcHorizon 表示MPC、预测时域。

  TargetTrajectoriesCalculatorBase(const TargetTrajectoriesCalculatorBase& rhs) = delete;

  // 函数说明：设置目标、displacement、速度，连接当前模块的数据流和控制逻辑。
  void setTargetDisplacementVelocity(scalar_t targetDisplacementVelocity) { targetDisplacementVelocity_ = targetDisplacementVelocity; }
  // 函数说明：设置目标、旋转、速度，连接当前模块的数据流和控制逻辑。
  void setTargetRotationVelocity_(scalar_t targetRotationVelocity) { targetRotationVelocity = targetRotationVelocity; }
  void setTargetJointState(const vector_t targetJointState);

  /**
   * Converts command line to TargetTrajectories.
   * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw] defined in pelvis frame
   * @param [in] observation : the current observation
   */
  virtual TargetTrajectories commandedPositionToTargetTrajectories(const vector4_t& commandedVelocities,  // 变量说明：TargetTrajectories 表示目标、trajectories。
                                                                   scalar_t initTime,  // 变量说明：initTime 表示init、时间。
                                                                   const vector_t& initState) = 0;  // 变量说明：initState 表示init、状态。

  /**
   * Converts desired velocities to TargetTrajectories.
   * @param [in] commandedVelocities : [v_x, v_y, v_yaw] defined in pelvis frame
   * @param [in] observation : the current observation
   */
  virtual TargetTrajectories commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,  // 变量说明：TargetTrajectories 表示目标、trajectories。
                                                                   scalar_t initTime,  // 变量说明：initTime 表示init、时间。
                                                                   const vector_t& initState) = 0;  // 变量说明：initState 表示init、状态。

 protected:
  // 函数说明：处理estimate、时间、to、目标，连接当前模块的数据流和控制逻辑。
  scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) const;

  // 函数说明：获取当前、浮动基/机身、pose、目标，连接当前模块的数据流和控制逻辑。
  virtual vector6_t getCurrentBasePoseTarget(const vector_t& state) const;  // 变量说明：vector6_t 表示vector6、t。

  // 函数说明：获取delta、浮动基/机身、目标，连接当前模块的数据流和控制逻辑。
  vector6_t getDeltaBaseTarget(const vector4_t& commadLinePoseTarget, const vector6_t& currentPoseTarget) const;

  // 函数说明：处理滤波器、and、transform、速度、命令、to、local，连接当前模块的数据流和控制逻辑。
  vector4_t filterAndTransformVelCommandToLocal(const vector4_t& commandedVelLocal,
                                                const scalar_t& currentEulerZ,  // 变量说明：currentEulerZ 表示当前、欧拉角、z。
                                                scalar_t filterAlpha) const;  // 变量说明：filterAlpha 表示滤波器、滤波系数。

  // 函数说明：处理integrate、目标、浮动基/机身、pose，连接当前模块的数据流和控制逻辑。
  vector6_t integrateTargetBasePose(const vector6_t& currentPose,
                                    const vector3_t& averageVel,  // 变量说明：averageVel 表示积分目标位姿时使用的平均基座速度。
                                    scalar_t deltaPelvisHeight,  // 变量说明：deltaPelvisHeight 表示delta、pelvis、height。
                                    scalar_t deltaT) const;  // 变量说明：deltaT 表示delta、t。

  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。

  // For pose control mode
  scalar_t targetDisplacementVelocity_;  // 变量说明：targetDisplacementVelocity_ 表示目标、displacement、速度。
  scalar_t targetRotationVelocity_;  // 变量说明：targetRotationVelocity_ 表示目标、旋转、速度。

  // For velocity control mode
  scalar_t maxDisplacementVelocityX_ = 0.6;  // 变量说明：maxDisplacementVelocityX_ 表示max、displacement、速度、x。
  scalar_t maxDisplacementVelocityY_ = 0.3;  // 变量说明：maxDisplacementVelocityY_ 表示max、displacement、速度、y。
  scalar_t maxDeltaPelvisHeight_ = 0.3;  // 变量说明：maxDeltaPelvisHeight_ 表示max、delta、pelvis、height。
  scalar_t maxRotationVelocity_ = 0.6;  // 变量说明：maxRotationVelocity_ 表示max、旋转、速度。

  scalar_t defaultBaseHeight_;  // 变量说明：defaultBaseHeight_ 表示默认、浮动基/机身、height。
  vector_t targetJointState_;  // 变量说明：targetJointState_ 表示目标、关节、状态。
  scalar_t mpcHorizon_;  // 变量说明：mpcHorizon_ 表示MPC、预测时域。
};

}  // namespace ocs2::humanoid

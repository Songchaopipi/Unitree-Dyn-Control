
#include "humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h"

#include <algorithm>  // For std::clamp

#include <ocs2_core/misc/LoadData.h>

#include <cmath>
#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {

// 函数说明：调用基类读取参考配置，并缓存目标轨迹积分所需的模型句柄。
TargetTrajectoriesCalculatorBase::TargetTrajectoriesCalculatorBase(const std::string& referenceFile,
                                                                   const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                                                   scalar_t mpcHorizon)
    // 函数说明：处理MPC、机器人、模型、指针，连接当前模块的数据流和控制逻辑。
    : mpcRobotModelPtr_(mpcRobotModel.clone()), mpcHorizon_(mpcHorizon) {
  std::cerr << "Loading reference file: " << referenceFile << std::endl;
  targetJointState_.resize(mpcRobotModel.getJointDim());
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "defaultBaseHeight", defaultBaseHeight_);
  // 函数说明：加载eigen、矩阵，连接当前模块的数据流和控制逻辑。
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", targetJointState_);
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", targetRotationVelocity_);
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", targetDisplacementVelocity_);
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityX", maxDisplacementVelocityX_);
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityY", maxDisplacementVelocityY_);
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "maxDeltaPelvisHeight", maxDeltaPelvisHeight_);
  // 函数说明：加载cpp、数据、type，连接当前模块的数据流和控制逻辑。
  loadData::loadCppDataType(referenceFile, "maxRotationVelocity", maxRotationVelocity_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void TargetTrajectoriesCalculatorBase::setTargetJointState(const vector_t targetJointState) {  // 变量说明：TargetTrajectoriesCalculatorBase 表示目标、trajectories、calculator、浮动基/机身。
  assert(targetJointState.size() == mpcRobotModelPtr_->getJointDim());
  targetJointState_ = targetJointState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector6_t TargetTrajectoriesCalculatorBase::getDeltaBaseTarget(const vector4_t& commadLinePoseTarget,  // 变量说明：TargetTrajectoriesCalculatorBase 表示目标、trajectories、calculator、浮动基/机身。
                                                               const vector6_t& currentPoseTarget) const {  // 变量说明：currentPoseTarget 表示从当前状态读取出的基座位姿目标起点。
  vector_t target(6);

  // X facing forward to the robot, Y to the left side in the baseFrame
  const scalar_t baseFrameDeltaX = commadLinePoseTarget(0);  // 变量说明：baseFrameDeltaX 表示浮动基/机身、坐标系/帧、delta、x。
  const scalar_t baseFrameDeltaY = commadLinePoseTarget(1);  // 变量说明：baseFrameDeltaY 表示浮动基/机身、坐标系/帧、delta、y。
  const scalar_t currentEulerZ = currentPoseTarget(3);  // 变量说明：currentEulerZ 表示当前、欧拉角、z。

  const scalar_t globalFrameDeltaX = std::cos(currentEulerZ) * baseFrameDeltaX - std::sin(currentEulerZ) * baseFrameDeltaY;  // 变量说明：globalFrameDeltaX 表示global、坐标系/帧、delta、x。
  const scalar_t globalFrameDeltaY = std::sin(currentEulerZ) * baseFrameDeltaX + std::cos(currentEulerZ) * baseFrameDeltaY;  // 变量说明：globalFrameDeltaY 表示global、坐标系/帧、delta、y。

  // base p_x, p_y are relative to current state
  target(0) = currentPoseTarget(0) + globalFrameDeltaX;
  target(1) = currentPoseTarget(1) + globalFrameDeltaY;
  // base z relative to the default height
  scalar_t deltaPelvisHeight = std::clamp(commadLinePoseTarget(2), -maxDeltaPelvisHeight_, maxDeltaPelvisHeight_);  // 变量说明：deltaPelvisHeight 表示delta、pelvis、height。
  target(2) = defaultBaseHeight_ + deltaPelvisHeight;
  // theta_z relative to current
  target(3) = currentPoseTarget(3) + commadLinePoseTarget(3) * M_PI / 180.0;
  target(4) = 0.0;
  target(5) = 0.0;

  return target;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector6_t TargetTrajectoriesCalculatorBase::getCurrentBasePoseTarget(const vector_t& state) const {  // 变量说明：TargetTrajectoriesCalculatorBase 表示目标、trajectories、calculator、浮动基/机身。
  vector_t currentPoseTarget = mpcRobotModelPtr_->getBasePose(state);  // 变量说明：currentPoseTarget 表示从当前状态读取出的基座位姿目标起点。
  // Zero out roll and pitch of the torso since target trajectories starts from current state
  currentPoseTarget(4) = 0.0;
  currentPoseTarget(5) = 0.0;

  return currentPoseTarget;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector4_t TargetTrajectoriesCalculatorBase::filterAndTransformVelCommandToLocal(const vector4_t& commandedVelLocal,  // 变量说明：TargetTrajectoriesCalculatorBase 表示目标、trajectories、calculator、浮动基/机身。
                                                                                const scalar_t& currentEulerZ,  // 变量说明：currentEulerZ 表示当前、欧拉角、z。
                                                                                scalar_t filterAlpha) const {  // 变量说明：filterAlpha 表示滤波器、滤波系数。
  static vector4_t commVelFiltered = vector4_t::Zero();  // 变量说明：commVelFiltered 表示comm、速度、filtered。

  commVelFiltered = commVelFiltered * filterAlpha + commandedVelLocal * (1 - filterAlpha);

  vector4_t globalTargetVel = commVelFiltered;  // 变量说明：globalTargetVel 表示global、目标、速度。

  globalTargetVel(0) = std::cos(currentEulerZ) * commVelFiltered[0] - std::sin(currentEulerZ) * commVelFiltered[1];
  globalTargetVel(1) = std::sin(currentEulerZ) * commVelFiltered[0] + std::cos(currentEulerZ) * commVelFiltered[1];

  return globalTargetVel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector6_t TargetTrajectoriesCalculatorBase::integrateTargetBasePose(const vector6_t& currentPose,  // 变量说明：TargetTrajectoriesCalculatorBase 表示目标、trajectories、calculator、浮动基/机身。
                                                                    const vector3_t& averageVel,  // 变量说明：averageVel 表示积分目标位姿时使用的平均基座速度。
                                                                    scalar_t deltaPelvisHeight,  // 变量说明：deltaPelvisHeight 表示delta、pelvis、height。
                                                                    scalar_t deltaT) const {  // 变量说明：deltaT 表示delta、t。
  vector6_t targetPose = currentPose;  // 变量说明：targetPose 表示位置命令换算后的最终基座位姿。

  targetPose[0] += averageVel[0] * deltaT;
  targetPose[1] += averageVel[1] * deltaT;
  targetPose[2] = deltaPelvisHeight;
  targetPose[3] += averageVel[2] * deltaT;
  targetPose[4] = 0.0;
  targetPose[5] = 0.0;
  return targetPose;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t TargetTrajectoriesCalculatorBase::estimateTimeToTarget(const vector_t& desiredBaseDisplacement) const {  // 变量说明：TargetTrajectoriesCalculatorBase 表示目标、trajectories、calculator、浮动基/机身。
  const scalar_t& dx = desiredBaseDisplacement(0);  // 变量说明：dx 表示 x 方向差值/位移。
  const scalar_t& dy = desiredBaseDisplacement(1);  // 变量说明：dy 表示 y 方向差值/位移。
  const scalar_t& dyaw = desiredBaseDisplacement(3);  // 变量说明：dyaw 表示偏航角增量。
  const scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity_;  // 变量说明：rotationTime 表示旋转、时间。
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);  // 变量说明：displacement 表示仿真开始到结束的机身平移量。
  const scalar_t displacementTime = displacement / targetDisplacementVelocity_;  // 变量说明：displacementTime 表示displacement、时间。
  return std::max(rotationTime, displacementTime);
}

}  // namespace ocs2::humanoid

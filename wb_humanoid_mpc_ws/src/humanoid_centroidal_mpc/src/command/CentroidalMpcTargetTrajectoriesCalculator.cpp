#include "humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h"

#include <boost/proto/proto_fwd.hpp>
#include <cmath>

#include <pinocchio/algorithm/center-of-mass.hpp>

#include <ocs2_core/misc/LoadData.h>
#include "ocs2_centroidal_model/ModelHelperFunctions.h"

namespace ocs2::humanoid {

// == 把外部给定的command转化成OCS2需要的TargetTrajectories, 给x_ref(t), u_ref(t) == 
CentroidalMpcTargetTrajectoriesCalculator::CentroidalMpcTargetTrajectoriesCalculator(const std::string& referenceFile,
                                                                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,  
                                                                                     PinocchioInterface pinocchioInterface,  
                                                                                     const CentroidalModelInfo& info,  
                                                                                     scalar_t mpcHorizon)
    // 函数说明：调用基类读取参考配置，并缓存目标轨迹积分所需的模型句柄。
    : TargetTrajectoriesCalculatorBase(referenceFile, mpcRobotModel, mpcHorizon),
      pinocchioInterface_(pinocchioInterface),
      info_(info),
      mass_(pinocchio::computeTotalMass(pinocchioInterface.getModel())) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories CentroidalMpcTargetTrajectoriesCalculator::commandedPositionToTargetTrajectories(const vector4_t& commadLinePoseTarget,  // 变量说明：CentroidalMpcTargetTrajectoriesCalculator 表示质心动力学、MPC、目标、trajectories、calculator。
                                                                                                    scalar_t initTime,  // 变量说明：initTime 表示init、时间。
                                                                                                    const vector_t& initState) {  // 变量说明：initState 表示init、状态。
  vector_t currentPoseTarget = getCurrentBasePoseTarget(initState);  // * currentPoseTarget 表示从当前状态读取出的基座位姿目标起点

  const vector_t targetPose = getDeltaBaseTarget(commadLinePoseTarget, currentPoseTarget);  // 变量说明：targetPose 表示位置命令换算后的最终基座位姿。

  scalar_t targetReachingTime = initTime + estimateTimeToTarget(targetPose - currentPoseTarget);  // 变量说明：targetReachingTime 表示预计到达位置目标的时间。

  // desired time trajectory
  const scalar_array_t timeTrajectory{initTime, targetReachingTime};  // 变量说明：timeTrajectory 表示目标轨迹各节点对应的时间戳。

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  stateTrajectory[0] << vector_t::Zero(6), currentPoseTarget, targetJointState_;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, targetJointState_;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));

  TargetTrajectories targetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};  // 变量说明：targetTrajectories 表示 OCS2 使用的目标状态/输入轨迹。

  return targetTrajectories;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// == 把外部给定的速度command转化成OCS2需要的TargetTrajectories, 给x_ref(t), u_ref(t) (当前应该只用到了这个函数)==
// == 1. commandedVelocities = [v_x, v_y, z_target, yaw_rate]
TargetTrajectories CentroidalMpcTargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,  
                                                                                                    scalar_t initTime,  
                                                                                                    const vector_t& initState) {  

  vector_t currentPoseTarget = getCurrentBasePoseTarget(initState); // * 从当前状态中读取基座位姿目标起点qb = [x,y,z,yaw,pitch,roll]
  vector4_t commVelTargetGlobal = filterAndTransformVelCommandToLocal(commandedVelocities, currentPoseTarget(3), 0.8); // * 对速度命令进行滤波,并从浮动基/机身坐标系转换到全局坐标系

  /////////////////////////
  // Intermediate Target //
  /////////////////////////

  vector6_t targetBaseTwist;  // * 表示期望基座线速度和角速度。
  targetBaseTwist << commVelTargetGlobal[0], commVelTargetGlobal[1], 0.0, 0.0, 0.0, commVelTargetGlobal[3];

  updateCentroidalDynamics(pinocchioInterface_, info_, mpcRobotModelPtr_->getGeneralizedCoordinates(initState));
  const Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>& A = getCentroidalMomentumMatrix(pinocchioInterface_);  // * A: centroidal momentum matrix,用来在基座速度和质心动量间转换

  vector6_t targetMomentum;  // 变量说明：targetMomentum 表示 MPC 状态前 6 维的目标归一化动量。

  const Eigen::Matrix<scalar_t, 6, 6> Ab = A.leftCols<6>();  // 变量说明：Ab 表示动量矩阵中浮动基对应的 6x6 子块。
  const Eigen::Matrix<scalar_t, 6, 6> Ab_inv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);  // 变量说明：Ab_inv 表示浮动基动量子块的稳定逆。


  targetMomentum << commVelTargetGlobal[0], commVelTargetGlobal[1], 0.0, 0.0, 0.0, commVelTargetGlobal[3] / mass_;

  vector6_t baseVel = Ab_inv * initState.head(6);  // * 近似估计当前时刻的基座线速度/角速度
  vector3_t averageVel;  // 变量说明：averageVel 表示积分目标位姿时使用的平均基座速度。
  averageVel(0) = (baseVel[0] + commVelTargetGlobal[0]) / 2;
  averageVel(1) = (baseVel[1] + commVelTargetGlobal[1]) / 2;
  averageVel(2) = (baseVel[5] + commVelTargetGlobal[3]) / 2;

  currentPoseTarget[2] = commVelTargetGlobal[2];
  scalar_t intermediateTargetTime = 0.7 * mpcHorizon_;  // 变量说明：intermediateTargetTime 表示中间目标点在预测时域中的时间位置。
  vector6_t intermediateTargetPose = integrateTargetBasePose(currentPoseTarget, averageVel, commVelTargetGlobal(2), intermediateTargetTime);  // 变量说明：intermediateTargetPose 表示速度命令积分出的中间基座位姿。

  //////////////////
  // Final Target //
  //////////////////

  averageVel(0) = (commVelTargetGlobal[0]);
  averageVel(1) = (commVelTargetGlobal[1]);
  averageVel(2) = (commVelTargetGlobal[3]);
  vector6_t finalTargetPose =  // 变量说明：finalTargetPose 表示预测时域末端的基座位姿目标。
      integrateTargetBasePose(intermediateTargetPose, averageVel, commVelTargetGlobal(2), (mpcHorizon_ - intermediateTargetTime));
  // desired time trajectory
  const scalar_array_t timeTrajectory{initTime, initTime + intermediateTargetTime, initTime + mpcHorizon_};  // 变量说明：timeTrajectory 表示目标轨迹各节点对应的时间戳。
  // desired state trajectory
  vector_array_t stateTrajectory(3, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  stateTrajectory[0] << targetMomentum, currentPoseTarget, targetJointState_;   // * targetJointState_直接用了reference.info配置文件读取出来的默认关节姿态
  stateTrajectory[1] << targetMomentum, intermediateTargetPose, targetJointState_;
  stateTrajectory[2] << targetMomentum, finalTargetPose, targetJointState_;
  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(3, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));
  TargetTrajectories targetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};  

  // === 最终返回的轨迹形式 ===
  // - 1. t0​=initTime t1​=initTime+0.7T t2​=initTime+T(T=mpcHorizon_)
  // - 2. x_ref(t0) = [targetMomentum, currentPoseTarget, targetJointState_]
  // - 3. x_ref(t1) = [targetMomentum, intermediateTargetPose, targetJointState_]
  // - 4. x_ref(t2) = [targetMomentum, finalTargetPose, targetJointState_]
  // - 5. u_ref(t) = 0(用于占位, 之后用weightCompensatingInput()替代)
  return targetTrajectories; 
}

}  // namespace ocs2::humanoid

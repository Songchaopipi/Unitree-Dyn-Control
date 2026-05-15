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

// 自动中文注释：src/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include <pinocchio/fwd.hpp>

#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

// Pinnochio
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>

namespace ocs2::humanoid {

template <typename SCALAR_T>
// 函数说明：更新坐标系/帧、placements，连接当前模块的数据流和控制逻辑。
void updateFramePlacements(const VECTOR_T<SCALAR_T>& q, PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface) {
  const auto& model = pinocchioInterface.getModel();  // 变量说明：model 表示模型。
  auto& data = pinocchioInterface.getData();  // 变量说明：data 表示数据。
  updateFramePlacements(q, model, data);
}
template void updateFramePlacements(const ad_vector_t& q, PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface);
template void updateFramePlacements(const vector_t& q, PinocchioInterfaceTpl<scalar_t>& pinocchioInterface);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
// 函数说明：更新坐标系/帧、placements，连接当前模块的数据流和控制逻辑。
void updateFramePlacements(const VECTOR_T<SCALAR_T>& q, const pinocchio::ModelTpl<SCALAR_T>& model, pinocchio::DataTpl<SCALAR_T>& data) {
  pinocchio::forwardKinematics(model, data, q);
  updateFramePlacements(model, data);
}
// 函数说明：更新坐标系/帧、placements，连接当前模块的数据流和控制逻辑。
template void updateFramePlacements(const ad_vector_t& q,
                                    const pinocchio::ModelTpl<ad_scalar_t>& model,  // 变量说明：model 表示模型。
                                    pinocchio::DataTpl<ad_scalar_t>& data);  // 变量说明：data 表示数据。
// 函数说明：更新坐标系/帧、placements，连接当前模块的数据流和控制逻辑。
template void updateFramePlacements(const vector_t& q, const pinocchio::ModelTpl<scalar_t>& model, pinocchio::DataTpl<scalar_t>& data);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> computeContactPositions(const VECTOR_T<SCALAR_T>& q,
                                                         PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                         const MpcRobotModelBase<SCALAR_T>& mpcRobotModel) {  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
  updateFramePlacements<SCALAR_T>(q, pinocchioInterface);
  return getContactPositions<SCALAR_T>(pinocchioInterface, mpcRobotModel);
}
// 函数说明：计算接触、positions，连接当前模块的数据流和控制逻辑。
template std::vector<VECTOR3_T<ad_scalar_t>> computeContactPositions(const VECTOR_T<ad_scalar_t>& q,
                                                                     PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                                     const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
// 函数说明：计算接触、positions，连接当前模块的数据流和控制逻辑。
template std::vector<VECTOR3_T<scalar_t>> computeContactPositions(const VECTOR_T<scalar_t>& q,
                                                                  PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                                  const MpcRobotModelBase<scalar_t>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> getContactPositions(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                     const MpcRobotModelBase<SCALAR_T>& mpcRobotModel) {  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
  assert(mpcRobotModel.modelSettings.contactNames.size() == N_CONTACTS);
  std::vector<VECTOR3_T<SCALAR_T>> footPositions;  // 变量说明：footPositions 表示足端、positions。
  footPositions.reserve(N_CONTACTS);
  const auto& data = pinocchioInterface.getData();  // 变量说明：data 表示数据。
  std::vector<pinocchio::FrameIndex> contactFrameIndices = getContactFrameIndices(pinocchioInterface, mpcRobotModel);  // 变量说明：contactFrameIndices 表示接触、坐标系/帧、indices。

  for (size_t i = 0; i < N_CONTACTS; i++) {
    const VECTOR3_T<SCALAR_T>& footPosition = data.oMf[getContactFrameIndex(pinocchioInterface, mpcRobotModel, i)].translation();  // 变量说明：footPosition 表示足端、位置。
    footPositions.emplace_back(footPosition);
  }
  return footPositions;
}
// 函数说明：获取接触、positions，连接当前模块的数据流和控制逻辑。
template std::vector<VECTOR3_T<ad_scalar_t>> getContactPositions(const PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,
                                                                 const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
// 函数说明：获取接触、positions，连接当前模块的数据流和控制逻辑。
template std::vector<VECTOR3_T<scalar_t>> getContactPositions(const PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                              const MpcRobotModelBase<scalar_t>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> computeFramePositions(const VECTOR_T<SCALAR_T>& q,
                                                       PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                       std::vector<std::string> frameNames) {  // 变量说明：frameNames 表示坐标系/帧、names。
  updateFramePlacements<SCALAR_T>(q, pinocchioInterface);
  return getFramePositions<SCALAR_T>(pinocchioInterface, frameNames);
}
// 函数说明：计算坐标系/帧、positions，连接当前模块的数据流和控制逻辑。
template std::vector<VECTOR3_T<ad_scalar_t>> computeFramePositions(const VECTOR_T<ad_scalar_t>& q,
                                                                   PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                                   std::vector<std::string> frameNames);  // 变量说明：frameNames 表示坐标系/帧、names。
// 函数说明：计算坐标系/帧、positions，连接当前模块的数据流和控制逻辑。
template std::vector<VECTOR3_T<scalar_t>> computeFramePositions(const VECTOR_T<scalar_t>& q,
                                                                PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                                std::vector<std::string> frameNames);  // 变量说明：frameNames 表示坐标系/帧、names。

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> getFramePositions(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                   std::vector<std::string> frameNames) {  // 变量说明：frameNames 表示坐标系/帧、names。
  std::vector<VECTOR3_T<SCALAR_T>> positions;  // 变量说明：positions 表示多个接触点或 frame 的世界系位置。
  positions.reserve(frameNames.size());
  const auto& data = pinocchioInterface.getData();  // 变量说明：data 表示数据。
  for (size_t i = 0; i < frameNames.size(); i++) {
    const pinocchio::FrameIndex frameIndex = pinocchioInterface.getModel().getFrameId(frameNames[i]);  // 变量说明：frameIndex 表示坐标系/帧、索引。
    const VECTOR3_T<SCALAR_T>& position = data.oMf[frameIndex].translation();  // 变量说明：position 表示位置。
    positions.emplace_back(position);
  }
  return positions;
}
// 函数说明：获取坐标系/帧、positions，连接当前模块的数据流和控制逻辑。
template std::vector<VECTOR3_T<ad_scalar_t>> getFramePositions(const PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,
                                                               std::vector<std::string> frameNames);  // 变量说明：frameNames 表示坐标系/帧、names。
// 函数说明：获取坐标系/帧、positions，连接当前模块的数据流和控制逻辑。
template std::vector<VECTOR3_T<scalar_t>> getFramePositions(const PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                            std::vector<std::string> frameNames);  // 变量说明：frameNames 表示坐标系/帧、names。

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t computeGroundHeightEstimate(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                     const vector_t& q,  // 变量说明：q 表示广义坐标/关节位置。
                                     size_t measuredMode) {  // 变量说明：measuredMode 表示measured、接触模式。
  updateFramePlacements<scalar_t>(q, pinocchioInterface);
  return getGroundHeightEstimate(pinocchioInterface, mpcRobotModel, measuredMode);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t getGroundHeightEstimate(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                 size_t measuredMode) {  // 变量说明：measuredMode 表示measured、接触模式。
  contact_flag_t measuredContactFlags = modeNumber2StanceLeg(measuredMode);  // 变量说明：measuredContactFlags 表示measured、接触、flags。

  std::vector<vector3_t> contactPositions = getContactPositions<scalar_t>(pinocchioInterface, mpcRobotModel);  // 变量说明：contactPositions 表示接触、positions。

  static scalar_t terrainHeight = 0.0;  // 变量说明：terrainHeight 表示terrain、height。

  // Use right foot if in contact
  if (measuredContactFlags[0] && measuredContactFlags[1]) {
    vector3_t footPosition1 = contactPositions[0];  // 变量说明：footPosition1 表示足端、position1。
    vector3_t footPosition2 = contactPositions[1];  // 变量说明：footPosition2 表示足端、position2。
    terrainHeight = 0.5 * (footPosition1[2] + footPosition2[2]);
  } else if (measuredContactFlags[0]) {
    vector3_t footPosition = contactPositions[0];  // 变量说明：footPosition 表示足端、位置。
    terrainHeight = footPosition[2];
  } else if (measuredContactFlags[1]) {
    vector3_t footPosition = contactPositions[1];  // 变量说明：footPosition 表示足端、位置。
    terrainHeight = footPosition[2];
  }
  return terrainHeight;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
// 函数说明：计算浮动基/机身、加速度，连接当前模块的数据流和控制逻辑。
VECTOR6_T<SCALAR_T> computeBaseAcceleration(const MATRIX_T<SCALAR_T>& M,
                                            const VECTOR_T<SCALAR_T>& nle,  // 变量说明：nle 表示 Pinocchio 非线性项，包括重力、科氏和离心项。
                                            const VECTOR_T<SCALAR_T>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                            const VECTOR_T<SCALAR_T>& externalForcesInJointSpace) {  // 变量说明：externalForcesInJointSpace 表示external、forces、in、关节、space。
  // Due to the block diagonal structure of the generalized mass matrix corresponding to the base the base mass matrix can be split into a
  // linear and angular part. Which are both inverted separately. This does not only exploit part of the sparsity but also prevents a CppAD
  // branching error when multiplying a 6x6 matrix with a6 dim. vector.

  Eigen::Matrix<SCALAR_T, 3, 3> M_bb_lin = M.topLeftCorner(3, 3);
  Eigen::Matrix<SCALAR_T, 3, 3> M_bb_ang = M.block(3, 3, 3, 3);
  auto M_bj = M.block(0, 6, 6, qdd_joints.size());  // 变量说明：M_bj 表示m、bj。
  Eigen::Matrix<SCALAR_T, 3, 3> M_bb_lin_inv = M_bb_lin.inverse();
  Eigen::Matrix<SCALAR_T, 3, 3> M_bb_ang_inv = M_bb_ang.inverse();

  VECTOR6_T<SCALAR_T> intermediate = -nle.head(6) - M_bj * qdd_joints + externalForcesInJointSpace.head(6);  // 变量说明：intermediate 表示求解浮动基加速度时的中间右端项。

  VECTOR6_T<SCALAR_T> baseAccelerations;  // 变量说明：baseAccelerations 表示浮动基/机身、accelerations。
  baseAccelerations.head(3) = M_bb_lin_inv * intermediate.head(3);
  baseAccelerations.tail(3) = M_bb_ang_inv * intermediate.tail(3);

  return baseAccelerations;
}
// 函数说明：计算浮动基/机身、加速度，连接当前模块的数据流和控制逻辑。
template VECTOR6_T<scalar_t> computeBaseAcceleration(const MATRIX_T<scalar_t>& M,
                                                     const VECTOR_T<scalar_t>& nle,  // 变量说明：nle 表示 Pinocchio 非线性项，包括重力、科氏和离心项。
                                                     const VECTOR_T<scalar_t>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                                     const VECTOR_T<scalar_t>& externalForcesInJointSpace);  // 变量说明：externalForcesInJointSpace 表示external、forces、in、关节、space。
// 函数说明：计算浮动基/机身、加速度，连接当前模块的数据流和控制逻辑。
template VECTOR6_T<ad_scalar_t> computeBaseAcceleration(const MATRIX_T<ad_scalar_t>& M,
                                                        const VECTOR_T<ad_scalar_t>& nle,  // 变量说明：nle 表示 Pinocchio 非线性项，包括重力、科氏和离心项。
                                                        const VECTOR_T<ad_scalar_t>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                                        const VECTOR_T<ad_scalar_t>& externalForcesInJointSpace);  // 变量说明：externalForcesInJointSpace 表示external、forces、in、关节、space。

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
// 函数说明：计算关节、torques，连接当前模块的数据流和控制逻辑。
VECTOR_T<SCALAR_T> computeJointTorques(const VECTOR_T<SCALAR_T>& q,
                                       const VECTOR_T<SCALAR_T>& qd,  // 变量说明：qd 表示广义速度向量。
                                       const VECTOR_T<SCALAR_T>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                       const std::array<VECTOR6_T<SCALAR_T>, 2>& footWrenches,  // 变量说明：该对象使用标准库容器/数组保存相关数据。
                                       PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface) {  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
  const auto& model = pinocchioInterface.getModel();  // 变量说明：model 表示模型。
  pinocchio::DataTpl<SCALAR_T>& data = pinocchioInterface.getData();  // 变量说明：data 表示数据。
  pinocchio::crba(model, data, q);
  pinocchio::nonLinearEffects(model, data, q, qd);

  // Compute Jacobians for the foot frames
  MATRIX_T<SCALAR_T> J_foot_l = MATRIX_T<SCALAR_T>::Zero(6, qd.size());  // 变量说明：J_foot_l 表示j、足端、l。
  MATRIX_T<SCALAR_T> J_foot_r = MATRIX_T<SCALAR_T>::Zero(6, qd.size());  // 变量说明：J_foot_r 表示j、足端、r。

  ////////////////////////////////////////////////////////////////////////////

  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_l_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  J_foot_l);
  // 函数说明：计算坐标系/帧、雅可比，连接当前模块的数据流和控制逻辑。
  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_r_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  J_foot_r);

  // Project contact wrenches into the joint space

  VECTOR_T<SCALAR_T> externalForcesInJointSpace = J_foot_l.transpose() * footWrenches[0] + J_foot_r.transpose() * footWrenches[1];  // 变量说明：externalForcesInJointSpace 表示external、forces、in、关节、space。

  VECTOR6_T<SCALAR_T> baseAccelerations = computeBaseAcceleration(data.M, data.nle, qdd_joints, externalForcesInJointSpace);  // 变量说明：baseAccelerations 表示浮动基/机身、accelerations。

  VECTOR_T<SCALAR_T> q_dd(qd.size());
  q_dd << baseAccelerations, qdd_joints;
  size_t n_joints = qdd_joints.size();  // 变量说明：n_joints 表示n、joints。

  VECTOR_T<SCALAR_T> jointTorques =  // 变量说明：jointTorques 表示关节、torques。
      data.M.bottomRows(n_joints) * q_dd + data.nle.tail(n_joints) - externalForcesInJointSpace.tail(n_joints);

  // return jointTorques;
  return jointTorques;
}
// 函数说明：计算关节、torques，连接当前模块的数据流和控制逻辑。
template VECTOR_T<scalar_t> computeJointTorques(const VECTOR_T<scalar_t>& q,
                                                const VECTOR_T<scalar_t>& qd,  // 变量说明：qd 表示广义速度向量。
                                                const VECTOR_T<scalar_t>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                                const std::array<VECTOR6_T<scalar_t>, 2>& footWrenches,  // 变量说明：该对象使用标准库容器/数组保存相关数据。
                                                PinocchioInterfaceTpl<scalar_t>& pinocchioInterface);  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
// 函数说明：计算关节、torques，连接当前模块的数据流和控制逻辑。
template VECTOR_T<ad_scalar_t> computeJointTorques(const VECTOR_T<ad_scalar_t>& q,
                                                   const VECTOR_T<ad_scalar_t>& qd,  // 变量说明：qd 表示广义速度向量。
                                                   const VECTOR_T<ad_scalar_t>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                                   const std::array<VECTOR6_T<ad_scalar_t>, 2>& footWrenches,  // 变量说明：该对象使用标准库容器/数组保存相关数据。
                                                   PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface);  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
// 函数说明：计算关节、torques、rnea，连接当前模块的数据流和控制逻辑。
VECTOR_T<SCALAR_T> computeJointTorquesRNEA(const VECTOR_T<SCALAR_T>& q,
                                           const VECTOR_T<SCALAR_T>& qd,  // 变量说明：qd 表示广义速度向量。
                                           const VECTOR_T<SCALAR_T>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                           const std::array<VECTOR6_T<SCALAR_T>, 2>& footWrenches,  // 变量说明：该对象使用标准库容器/数组保存相关数据。
                                           PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface) {  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
  const auto& model = pinocchioInterface.getModel();  // 变量说明：model 表示模型。
  auto& data = pinocchioInterface.getData();  // 变量说明：data 表示数据。

  pinocchio::container::aligned_vector<pinocchio::Force> fextDesired(model.njoints, pinocchio::Force::Zero());
  pinocchio::forwardKinematics(model, data, q, qd);
  // 函数说明：更新坐标系/帧、placements，连接当前模块的数据流和控制逻辑。
  pinocchio::updateFramePlacements(model, data);

  auto setExternalForce = [&](const std::string& frameName, size_t i) {  // 变量说明：setExternalForce 表示set、external、力。
    const auto frameIndex = model.getFrameId(frameName);  // 变量说明：frameIndex 表示坐标系/帧、索引。
    const auto jointIndex = model.frames[frameIndex].parentJoint;  // 变量说明：jointIndex 表示关节、索引。
    const VECTOR3_T<SCALAR_T> translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();  // 变量说明：translationJointFrameToContactFrame 表示translation、关节、坐标系/帧、to、接触、坐标系/帧。
    const MATRIX3_T<SCALAR_T> rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();  // 变量说明：rotationWorldFrameToJointFrame 表示旋转、world、坐标系/帧、to、关节、坐标系/帧。
    const VECTOR3_T<SCALAR_T> contactForce = rotationWorldFrameToJointFrame * footWrenches[i].head(3);  // 变量说明：contactForce 表示接触、力。
    const VECTOR3_T<SCALAR_T> contactTorque = rotationWorldFrameToJointFrame * footWrenches[i].tail(3);  // 变量说明：contactTorque 表示接触、力矩。
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce) + contactTorque;
  };

  setExternalForce("foot_l_contact", 0);
  setExternalForce("foot_r_contact", 1);
  pinocchio::crba(model, data, q);
  pinocchio::nonLinearEffects(model, data, q, qd);

  // Compute Jacobians for the foot frames
  MATRIX_T<SCALAR_T> J_foot_l = MATRIX_T<SCALAR_T>::Zero(6, qd.size());  // 变量说明：J_foot_l 表示j、足端、l。
  MATRIX_T<SCALAR_T> J_foot_r = MATRIX_T<SCALAR_T>::Zero(6, qd.size());  // 变量说明：J_foot_r 表示j、足端、r。

  ////////////////////////////////////////////////////////////////////////////

  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_l_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  J_foot_l);
  // 函数说明：计算坐标系/帧、雅可比，连接当前模块的数据流和控制逻辑。
  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_r_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  J_foot_r);

  // Project contact wrenches into the joint space

  VECTOR_T<SCALAR_T> externalForcesInJointSpace = J_foot_l.transpose() * footWrenches[0] + J_foot_r.transpose() * footWrenches[1];  // 变量说明：externalForcesInJointSpace 表示external、forces、in、关节、space。

  // Repalce q with external forces in joint space.

  VECTOR6_T<SCALAR_T> baseAccelerations = computeBaseAcceleration(data.M, data.nle, qdd_joints, externalForcesInJointSpace);  // 变量说明：baseAccelerations 表示浮动基/机身、accelerations。

  VECTOR_T<SCALAR_T> q_dd(qd.size());
  q_dd << baseAccelerations, qdd_joints;

  vector_t torques = pinocchio::rnea(model, data, q, qd, q_dd, fextDesired);  // 变量说明：torques 表示逆动力学计算出的广义力矩。

  return torques.tail(qdd_joints.size());
}
// 函数说明：计算关节、torques、rnea，连接当前模块的数据流和控制逻辑。
template VECTOR_T<scalar_t> computeJointTorquesRNEA(const VECTOR_T<scalar_t>& q,
                                                    const VECTOR_T<scalar_t>& qd,  // 变量说明：qd 表示广义速度向量。
                                                    const VECTOR_T<scalar_t>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                                    const std::array<VECTOR6_T<scalar_t>, 2>& footWrenches,  // 变量说明：该对象使用标准库容器/数组保存相关数据。
                                                    PinocchioInterfaceTpl<scalar_t>& pinocchioInterface);  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
// template VECTOR_T<ad_scalar_t> computeJointTorquesRNEA(const VECTOR_T<ad_scalar_t>& q,
//                                                        const VECTOR_T<ad_scalar_t>& qd,
//                                                        const VECTOR_T<ad_scalar_t>& qdd_joints,
//                                                        const std::array<VECTOR6_T<ad_scalar_t>, 2>& footWrenches,
//                                                        PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface);

}  // namespace ocs2::humanoid

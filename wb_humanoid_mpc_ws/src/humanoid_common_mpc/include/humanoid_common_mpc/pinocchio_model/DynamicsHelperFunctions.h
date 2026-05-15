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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <pinocchio/fwd.hpp>

#include <array>
#include <cppad/cg.hpp>
#include <iostream>
#include <memory>

#include <pinocchio/algorithm/center-of-mass.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/pinocchio_model/PinocchioFrameConversions.h"

namespace ocs2::humanoid {

///
/// @brief Updates all frame placement in the PinocchioInterface.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param q Current generalized coordinates.
/// @param pinocchioInterface Pinocchio interface.
///

template <typename SCALAR_T>
void updateFramePlacements(const VECTOR_T<SCALAR_T>& q, PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface);

///
/// @brief Computes and returns all the contact positions in the inertial frame.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param q Current generalized coordinates.
/// @param pinocchioInterface Pinocchio interface.
///
/// @return Vector of contact positions in the inertial frame.

template <typename SCALAR_T>
// 函数说明：更新坐标系/帧、placements，连接当前模块的数据流和控制逻辑。
void updateFramePlacements(const VECTOR_T<SCALAR_T>& q, const pinocchio::ModelTpl<SCALAR_T>& model, pinocchio::DataTpl<SCALAR_T>& data);

///
/// @brief Computes and returns all the contact positions in the inertial frame.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param q Current generalized coordinates.
/// @param pinocchioInterface Pinocchio interface.
///
/// @return Vector of contact positions in the inertial frame.

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> computeContactPositions(const VECTOR_T<SCALAR_T>& q,
                                                         PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                         const MpcRobotModelBase<SCALAR_T>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。

///
/// @brief Returns all the contact positions in the inertial frame.
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param pinocchioInterface Pinocchio interface.
///
/// @return Vector of contact positions in the inertial frame.

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> getContactPositions(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                     const MpcRobotModelBase<SCALAR_T>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。

///
/// @brief Computes and returns all the frame positions in the inertial frame.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param q Current generalized coordinates.
/// @param pinocchioInterface Pinocchio interface.
/// @param frameNames Names of the frames for which the positions should be computed.
///
/// @return Vector of contact positions in the inertial frame.

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> computeFramePositions(const VECTOR_T<SCALAR_T>& q,
                                                       PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                       std::vector<std::string> frameNames);  // 变量说明：frameNames 表示坐标系/帧、names。

///
/// @brief Returns all the frame positions in the inertial frame.
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param pinocchioInterface Pinocchio interface.
/// @param frameNames Names of the frames for which the positions should be returned.
///
/// @return Vector of contact positions in the inertial frame.

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> getFramePositions(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                   std::vector<std::string> frameNames);  // 变量说明：frameNames 表示坐标系/帧、names。

///
/// @brief Gets the estimated ground height using the feet in contact for a pinocchio model with updated frame placements.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param pinocchioInterface Pinocchio interface.
/// @param measuredMode mode of which feet are in contact.
///
/// @return the estimated ground height.

scalar_t getGroundHeightEstimate(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                 size_t measuredMode);  // 变量说明：measuredMode 表示measured、接触模式。

///
/// @brief Computes the estimated ground height using the feet in contact.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param pinocchioInterface Pinocchio interface.
/// @param q Current generalized coordinates.
/// @param measuredMode mode of which feet are in contact.
///
/// @return the estimated ground height.

scalar_t computeGroundHeightEstimate(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                     const vector_t& q,  // 变量说明：q 表示广义坐标/关节位置。
                                     size_t measuredMode);  // 变量说明：measuredMode 表示measured、接触模式。

///
/// @brief Return amount of legs in contact
///
/// @return number of clodes leg contacts as unsigned integer

inline size_t numberOfLegsInContacts(const contact_flag_t& contactFlags) {
  size_t numStanceLegs = 0;  // 变量说明：numStanceLegs 表示num、支撑脚/站立、legs。
  for (auto legInContact : contactFlags) {
    if (legInContact) {
      ++numStanceLegs;
    }
  }
  return numStanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

inline vector_t weightCompensatingInput(const PinocchioInterface& pinocchioInterface,
                                        const contact_flag_t& contactFlags,  // 变量说明：contactFlags 表示接触、flags。
                                        const MpcRobotModelBase<scalar_t>& mpcRobotModel) {  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
  const static scalar_t totalGravitationalForce = computeTotalMass(pinocchioInterface.getModel()) * 9.81;  // 变量说明：totalGravitationalForce 表示total、gravitational、力。
  const auto numStanceLegs = numberOfLegsInContacts(contactFlags);  // 变量说明：numStanceLegs 表示num、支撑脚/站立、legs。
  vector_t input = vector_t::Zero(mpcRobotModel.getInputDim());  // 变量说明：input 表示输入。
  if (numStanceLegs > 0) {
    const vector3_t forceInInertialFrame(0.0, 0.0, totalGravitationalForce / numStanceLegs);
    for (size_t i = 0; i < contactFlags.size(); i++) {
      if (contactFlags[i]) {
        mpcRobotModel.setContactForce(input, forceInInertialFrame, i);
      }
    }
  }
  return input;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
// 函数说明：获取接触、坐标系/帧、索引，连接当前模块的数据流和控制逻辑。
inline pinocchio::FrameIndex getContactFrameIndex(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                  const MpcRobotModelBase<SCALAR_T>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                                  size_t contactIndex) {  // 变量说明：contactIndex 表示左右脚接触点编号。
  return pinocchioInterface.getModel().getFrameId(mpcRobotModel.modelSettings.contactNames[contactIndex]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
// 函数说明：获取接触、坐标系/帧、indices，连接当前模块的数据流和控制逻辑。
inline std::vector<pinocchio::FrameIndex> getContactFrameIndices(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                                 const MpcRobotModelBase<SCALAR_T>& mpcRobotModel) {  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
  std::vector<pinocchio::FrameIndex> contactFrameIndices;  // 变量说明：contactFrameIndices 表示接触、坐标系/帧、indices。
  contactFrameIndices.reserve(N_CONTACTS);
  for (size_t i = 0; i < N_CONTACTS; i++) {
    contactFrameIndices.emplace_back(getContactFrameIndex<SCALAR_T>(pinocchioInterface, mpcRobotModel, i));
  }
  return contactFrameIndices;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

///
/// @brief Computes the center of pressure (CoP) in the inertial frame.
///
/// @warning Assumes that the frame placements are up to date. Does not work when frame is not in contact -> f_z = 0 results in NaN.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param input Current input.
/// @param pinocchioInterface Pinocchio interface.
///
/// @return Location of center of pressure in the inertial frame.

template <typename SCALAR_T>
// 函数说明：计算接触、co、p，连接当前模块的数据流和控制逻辑。
inline VECTOR3_T<SCALAR_T> computeContactCoP(const VECTOR_T<SCALAR_T> input,
                                             const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                             size_t contactIndex,  // 变量说明：contactIndex 表示左右脚接触点编号。
                                             const MpcRobotModelBase<scalar_t>& mpcRobotModel) {  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
  const auto localContactWrench =  // 变量说明：localContactWrench 表示local、接触、力和力矩。
      rotateVectorWorldToLocal<SCALAR_T>(mpcRobotModel.getContactWrench(input, contactIndex), pinocchioInterface.getData(),
                                         getContactFrameIndex(pinocchioInterface, mpcRobotModel, contactIndex));
  SCALAR_T copX = -localContactWrench[4] / localContactWrench[2];  // 变量说明：copX 表示cop、x。
  SCALAR_T copY = localContactWrench[3] / localContactWrench[2];  // 变量说明：copY 表示cop、y。
  VECTOR3_T<SCALAR_T> copInLocalFrame(copX, copY, 0.0);
  return transformPointLocalToWorld(copInLocalFrame, pinocchioInterface.getData(),
                                    getContactFrameIndex(pinocchioInterface, mpcRobotModel, contactIndex));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

///
/// @brief Computes the center of pressure (CoP) for all contacts in the inertial frame.
///
/// @warning Assumes that the frame placements are up to date.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param input Current input.
/// @param pinocchioInterface Pinocchio interface.
/// @param contactFlags Flags indicating which contacts are in contact. Returns 0 vector if not in contact.
///
/// @return Locations of center of pressure in the inertial frame.

inline std::vector<vector3_t> computeContactsCoP(const vector_t input,
                                                 const PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                 const contact_flag_t& contactFlags,  // 变量说明：contactFlags 表示接触、flags。
                                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel) {  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
  std::vector<vector3_t> contactCoPs;  // 变量说明：contactCoPs 表示接触、co、ps。
  contactCoPs.reserve(N_CONTACTS);
  for (size_t contactIndex = 0; contactIndex < N_CONTACTS; contactIndex++) {
    if (contactFlags[contactIndex]) {
      contactCoPs.emplace_back(computeContactCoP<scalar_t>(input, pinocchioInterface, contactIndex, mpcRobotModel));
    } else {
      contactCoPs.emplace_back(vector3_t::Zero());
    }
  }
  return contactCoPs;
}

///
/// @brief Computes euler zyx angles from an eigen quaternion
///
/// @param quat Quaternion
///
/// @return vector3_t (euler_z, euler_y,euler_x)

static inline vector3_t quaternionToEulerZYX(const quaternion_t& quat) {
  scalar_t w = quat.w();  // 变量说明：w 表示w。
  scalar_t x = quat.x();  // 变量说明：x 表示 x 坐标或状态向量，依当前上下文决定。
  scalar_t y = quat.y();  // 变量说明：y 表示y。
  scalar_t z = quat.z();  // 变量说明：z 表示z。

  // Yaw (Z axis rotation)
  scalar_t yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));  // 变量说明：yaw 表示偏航。
  // Pitch (Y axis rotation)
  scalar_t pitch = std::asin(2.0 * (w * y - z * x));  // 变量说明：pitch 表示俯仰。
  // Roll (X axis rotation)
  scalar_t roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));  // 变量说明：roll 表示横滚。

  return vector3_t(yaw, pitch, roll);
}

///
/// @brief Computes the base acceleration
///
/// @param M Given mass matrix
/// @param nle nonlinear effects in joint space inlcuding gravity comp, centrifugal, corriolis
/// @param qdd_joints Generalized accelerations
/// @param externalForcesInJointSpace Sum of J^T F_ext
///
/// @return linear and angular base acceleration.

template <typename SCALAR_T>
// 函数说明：计算浮动基/机身、加速度，连接当前模块的数据流和控制逻辑。
VECTOR6_T<SCALAR_T> computeBaseAcceleration(const MATRIX_T<SCALAR_T>& M,
                                            const VECTOR_T<SCALAR_T>& nle,  // 变量说明：nle 表示 Pinocchio 非线性项，包括重力、科氏和离心项。
                                            const VECTOR_T<SCALAR_T>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                            const VECTOR_T<SCALAR_T>& externalForcesInJointSpace);  // 变量说明：externalForcesInJointSpace 表示external、forces、in、关节、space。

///
///
/// @param q Generalized coordinates
/// @param v Generalized velocities
/// @param a Generalized accelerations
/// @param footWrenches [W_left, W_right]
/// @param pinocchioInterface
///
/// @return joint torques of same dimension as qdd_joints

template <typename SCALAR_T>
// 函数说明：计算关节、torques，连接当前模块的数据流和控制逻辑。
VECTOR_T<SCALAR_T> computeJointTorques(const VECTOR_T<SCALAR_T>& q,
                                       const VECTOR_T<SCALAR_T>& qd,  // 变量说明：qd 表示广义速度向量。
                                       const VECTOR_T<SCALAR_T>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                       const std::array<VECTOR6_T<SCALAR_T>, 2>& footWrenches,  // 变量说明：该对象使用标准库容器/数组保存相关数据。
                                       PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface);  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。

///
/// @brief WARNING!!!!!! This formualtion currently does not work! Since pinocchio is not aware of the custom 6 dof base joint the results
/// are wrong. Computes the joint torques via custom inverse dynamics
///
/// @param q Generalized coordinates
/// @param v Generalized velocities
/// @param a Generalized accelerations
/// @param footWrenches [W_left, W_right]
/// @param pinocchioInterface
///
/// @return joint torques of same dimension as qdd_joints

template <typename SCALAR_T>
// 函数说明：计算关节、torques、rnea，连接当前模块的数据流和控制逻辑。
VECTOR_T<SCALAR_T> computeJointTorquesRNEA(const VECTOR_T<SCALAR_T>& q,
                                           const VECTOR_T<SCALAR_T>& qd,  // 变量说明：qd 表示广义速度向量。
                                           const VECTOR_T<SCALAR_T>& qdd_joints,  // 变量说明：qdd_joints 表示qdd、joints。
                                           const std::array<VECTOR6_T<SCALAR_T>, 2>& footWrenches,  // 变量说明：该对象使用标准库容器/数组保存相关数据。
                                           PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface);  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。

}  // namespace ocs2::humanoid

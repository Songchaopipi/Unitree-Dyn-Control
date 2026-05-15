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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/pinocchio_model/PinocchioFrameConversions.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <pinocchio/fwd.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

///
/// @brief Returns rotation matrix from local to world frame
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param Pinocchio interface data member.
/// @param Local frame index of the local frame.
///
/// @return 3x3 rotation matrix corresponding to the passive rotation from local frame to world frame.

template <typename SCALAR_T>
// 函数说明：获取旋转、矩阵、local、to、world，连接当前模块的数据流和控制逻辑。
inline const MATRIX3_T<SCALAR_T>& getRotationMatrixLocalToWorld(const pinocchio::DataTpl<SCALAR_T>& data,
                                                                const pinocchio::FrameIndex localFrameIndex) {  // 变量说明：localFrameIndex 表示local、坐标系/帧、索引。
  return data.oMf[localFrameIndex].rotation();
}

///
/// @brief Transforms a 3D vector from world to local frame
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param 3D vector expressed in world frame.
/// @param Pinocchio interface.
/// @param Local frame index of the local frame.
///
/// @return vector converted into world frame .

template <typename SCALAR_T>
// 函数说明：处理rotate、向量、world、to、local，连接当前模块的数据流和控制逻辑。
inline VECTOR3_T<SCALAR_T> rotateVectorWorldToLocal(const VECTOR3_T<SCALAR_T>& vectorInWorldFrame,
                                                    const pinocchio::DataTpl<SCALAR_T>& data,  // 变量说明：data 表示数据。
                                                    const pinocchio::FrameIndex& localFrameIndex) {  // 变量说明：localFrameIndex 表示local、坐标系/帧、索引。
  const MATRIX3_T<SCALAR_T>& R_WorldToLocal = getRotationMatrixLocalToWorld(data, localFrameIndex).transpose();  // 变量说明：R_WorldToLocal 表示r、world、to、local。
  return (R_WorldToLocal * vectorInWorldFrame);
}

///
/// @brief Transforms a 3D vector from local to world frame
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param 3D vector expressed in world frame.
/// @param Pinocchio interface.
/// @param Local frame index of the local frame.
///
/// @return vector converted into local frame .

template <typename SCALAR_T>
// 函数说明：处理rotate、向量、local、to、world，连接当前模块的数据流和控制逻辑。
inline VECTOR3_T<SCALAR_T> rotateVectorLocalToWorld(const VECTOR3_T<SCALAR_T>& vectorInLocalFrame,
                                                    const pinocchio::DataTpl<SCALAR_T>& data,  // 变量说明：data 表示数据。
                                                    const pinocchio::FrameIndex& localFrameIndex) {  // 变量说明：localFrameIndex 表示local、坐标系/帧、索引。
  const MATRIX3_T<SCALAR_T>& R_WorldToLocal = getRotationMatrixLocalToWorld(data, localFrameIndex);  // 变量说明：R_WorldToLocal 表示r、world、to、local。
  return (R_WorldToLocal * vectorInLocalFrame);
}

///
/// @brief Transforms a 6D vector from world to local frame
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param 6D vector expressed in world frame.
/// @param Pinocchio interface.
/// @param Local frame index of the local frame.
///
/// @return vector converted into world frame .

template <typename SCALAR_T>
// 函数说明：处理rotate、向量、world、to、local，连接当前模块的数据流和控制逻辑。
inline VECTOR6_T<SCALAR_T> rotateVectorWorldToLocal(const VECTOR6_T<SCALAR_T>& vectorInWorldFrame,
                                                    const pinocchio::DataTpl<SCALAR_T>& data,  // 变量说明：data 表示数据。
                                                    const pinocchio::FrameIndex& localFrameIndex) {  // 变量说明：localFrameIndex 表示local、坐标系/帧、索引。
  VECTOR6_T<SCALAR_T> vectorInLocalFrame(6);
  vectorInLocalFrame.head(3) = rotateVectorWorldToLocal(VECTOR3_T<SCALAR_T>(vectorInWorldFrame.head(3)), data, localFrameIndex),
  vectorInLocalFrame.tail(3) = rotateVectorWorldToLocal(VECTOR3_T<SCALAR_T>(vectorInWorldFrame.tail(3)), data, localFrameIndex);
  return vectorInLocalFrame;
}

///
/// @brief Transforms a 6D vector from local to world frame
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param 6D vector expressed in world frame.
/// @param Pinocchio interface.
/// @param Local frame index of the local frame.
///
/// @return vector converted into local frame .

template <typename SCALAR_T>
// 函数说明：处理rotate、向量、local、to、world，连接当前模块的数据流和控制逻辑。
inline VECTOR6_T<SCALAR_T> rotateVectorLocalToWorld(const VECTOR6_T<SCALAR_T>& vectorInLocalFrame,
                                                    const pinocchio::DataTpl<SCALAR_T>& data,  // 变量说明：data 表示数据。
                                                    const pinocchio::FrameIndex& localFrameIndex) {  // 变量说明：localFrameIndex 表示local、坐标系/帧、索引。
  VECTOR6_T<SCALAR_T> vectorInWorldFrame(6);
  vectorInWorldFrame.head(3) = rotateVectorLocalToWorld(VECTOR3_T<SCALAR_T>(vectorInLocalFrame.head(3)), data, localFrameIndex),
  vectorInWorldFrame.tail(3) = rotateVectorLocalToWorld(VECTOR3_T<SCALAR_T>(vectorInLocalFrame.tail(3)), data, localFrameIndex);
  return vectorInWorldFrame;
}

///
/// @brief Returns transformation matrix from local to world frame
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param Pinocchio interface.
/// @param Local frame index of the local frame.
///
/// @return 4x4 matrix corresponding to the transformation from local to world frame.

template <typename SCALAR_T>
// 函数说明：获取transformation、矩阵、local、to、world，连接当前模块的数据流和控制逻辑。
inline MATRIX4_T<SCALAR_T> getTransformationMatrixLocalToWorld(const pinocchio::DataTpl<SCALAR_T>& data,
                                                               const pinocchio::FrameIndex localFrameIndex) {  // 变量说明：localFrameIndex 表示local、坐标系/帧、索引。
  return (data.oMf[localFrameIndex].toHomogeneousMatrix_impl());
}

///
/// @brief Returns transformation matrix from local to world frame
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param Pinocchio interface.
/// @param Local frame index of the local frame.
///
/// @return 4x4 matrix corresponding to the transformation from world to local frame.

template <typename SCALAR_T>
// 函数说明：获取transformation、矩阵、world、to、local，连接当前模块的数据流和控制逻辑。
inline MATRIX4_T<SCALAR_T> getTransformationMatrixWorldToLocal(const pinocchio::DataTpl<SCALAR_T>& data,
                                                               const pinocchio::FrameIndex localFrameIndex) {  // 变量说明：localFrameIndex 表示local、坐标系/帧、索引。
  MATRIX4_T<SCALAR_T> transformWorldToLocal = MATRIX4_T<SCALAR_T>::Identity();  // 变量说明：transformWorldToLocal 表示transform、world、to、local。
  transformWorldToLocal.block(0, 0, 3, 3) = data.oMf[localFrameIndex].rotation().transpose();
  transformWorldToLocal.block(0, 3, 3, 1) = -data.oMf[localFrameIndex].rotation().transpose() * data.oMf[localFrameIndex].translation();
  return transformWorldToLocal;
}

///
/// @brief Transforms a 3D vector from local to world frame
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param 3D vector expressed in world frame.
/// @param Pinocchio interface.
/// @param Local frame index of the local frame.
///
/// @return vector converted into local frame .

template <typename SCALAR_T>
// 函数说明：处理transform、point、local、to、world，连接当前模块的数据流和控制逻辑。
inline VECTOR3_T<SCALAR_T> transformPointLocalToWorld(const VECTOR3_T<SCALAR_T>& pointInLocalFrame,
                                                      const pinocchio::DataTpl<SCALAR_T>& data,  // 变量说明：data 表示数据。
                                                      const pinocchio::FrameIndex& localFrameIndex) {  // 变量说明：localFrameIndex 表示local、坐标系/帧、索引。
  VECTOR4_T<SCALAR_T> homogeniousPoint;  // 变量说明：homogeniousPoint 表示homogenious、point。
  homogeniousPoint << pointInLocalFrame, 1.0;
  return (getTransformationMatrixLocalToWorld(data, localFrameIndex) * homogeniousPoint).head(3);
}

///
/// @brief Transforms a 3D vector from world to local frame
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param 3D vector expressed in world frame.
/// @param Pinocchio interface.
/// @param Local frame index of the local frame.
///
/// @return vector converted into world frame .

template <typename SCALAR_T>
// 函数说明：处理transform、point、world、to、local，连接当前模块的数据流和控制逻辑。
inline VECTOR3_T<SCALAR_T> transformPointWorldToLocal(const VECTOR3_T<SCALAR_T>& pointInWorldFrame,
                                                      const pinocchio::DataTpl<SCALAR_T>& data,  // 变量说明：data 表示数据。
                                                      const pinocchio::FrameIndex& localFrameIndex) {  // 变量说明：localFrameIndex 表示local、坐标系/帧、索引。
  VECTOR4_T<SCALAR_T> homogeniousPoint;  // 变量说明：homogeniousPoint 表示homogenious、point。
  homogeniousPoint << pointInWorldFrame, 1.0;
  return (getTransformationMatrixWorldToLocal(data, localFrameIndex) * homogeniousPoint).head(3);
}

}  // namespace ocs2::humanoid

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

// 自动中文注释：src/humanoid_common_mpc/src/contact/ContactPolygon.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/contact/ContactPolygon.h"

#include <boost/property_tree/ptree.hpp>

namespace ocs2::humanoid {

// 函数说明：处理接触、polygon，连接当前模块的数据流和控制逻辑。
ContactPolygon::ContactPolygon(const std::vector<vector3_t>& polygonPoints,
                               const ContactCenterPoint& contactCenterPoint,  // 变量说明：contactCenterPoint 表示接触、center、point。
                               const scalar_t& scaleFactor)
    // 函数说明：处理polygon、points，连接当前模块的数据流和控制逻辑。
    : polygonPoints_(polygonPoints), polygonLimits_(0, 0, 0, 0), contactCenterPoint_(contactCenterPoint) {
  polygonPointFrameNames_.reserve(polygonPoints_.size());

  for (int i = 0; i < polygonPoints_.size(); i++) {
    polygonPointFrameNames_.emplace_back((contactCenterPoint.frameName + "_p_" + std::to_string(i)));
    if (scaleFactor != 1.0) {
      polygonPoints_[i] = polygonPoints_[i] * scaleFactor;
      // update x limits
      if (polygonPoints_[i][0] < polygonLimits_.x_min)
        polygonLimits_.x_min = polygonPoints_[i][0];
      else if (polygonPoints_[i][0] > polygonLimits_.x_max)
        polygonLimits_.x_max = polygonPoints_[i][0];
      // update y limits
      if (polygonPoints_[i][1] < polygonLimits_.y_min)
        polygonLimits_.y_min = polygonPoints_[i][1];
      else if (polygonPoints_[i][1] > polygonLimits_.y_max)
        polygonLimits_.y_max = polygonPoints_[i][1];
    }
  }
}

// 函数说明：处理接触、polygon，连接当前模块的数据流和控制逻辑。
ContactPolygon::ContactPolygon(const std::vector<vector3_t>& polygonPoints,
                               const PolygonBounds& polygonBounds,  // 变量说明：polygonBounds 表示polygon、bounds。
                               const ContactCenterPoint& contactCenterPoint)
    // 函数说明：处理polygon、points，连接当前模块的数据流和控制逻辑。
    : polygonPoints_(polygonPoints), polygonLimits_(polygonBounds), contactCenterPoint_(contactCenterPoint) {
  polygonPointFrameNames_.reserve(polygonPoints_.size());

  for (int i = 0; i < polygonPoints_.size(); i++) {
    polygonPointFrameNames_.emplace_back((contactCenterPoint.frameName + "_p_" + std::to_string(i)));
  }
}

// 函数说明：获取接触、point、translation、cross、product、矩阵，连接当前模块的数据流和控制逻辑。
matrix3_t ContactPolygon::getContactPointTranslationCrossProductMatrix(int index) const {  // 变量说明：ContactPolygon 表示接触、polygon。
  vector3_t current_point = polygonPoints_[index];  // 变量说明：current_point 表示当前、point。
  matrix3_t crossprodMat = matrix3_t::Zero();  // 变量说明：crossprodMat 表示crossprod、mat。
  crossprodMat(0, 2) = current_point[1];
  crossprodMat(1, 2) = -current_point[0];
  crossprodMat(2, 0) = -current_point[1];
  crossprodMat(2, 1) = current_point[0];
  return crossprodMat;
}

}  // namespace ocs2::humanoid

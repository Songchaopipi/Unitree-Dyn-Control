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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/contact/ContactPolygon.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include "humanoid_common_mpc/contact/ContactCenterPoint.h"

namespace ocs2::humanoid {

/// \brief the maximum extension of the polygon with respect to the specified frame

struct PolygonBounds {
  PolygonBounds(
      const scalar_t& x_min, const scalar_t& x_max, const scalar_t& y_min, const scalar_t& y_max, const scalar_t& scaleFactor = 1.0)  // 变量说明：x_min 表示x、min。
      : x_min(x_min * scaleFactor), x_max(x_max * scaleFactor), y_min(y_min * scaleFactor), y_max(y_max * scaleFactor){};

  scalar_t x_min;  // 变量说明：x_min 表示x、min。
  scalar_t x_max;  // 变量说明：x_max 表示x、max。
  scalar_t y_min;  // 变量说明：y_min 表示y、min。
  scalar_t y_max;  // 变量说明：y_max 表示y、max。
};

///
/// \brief A planar contact polygon defined by the convex hull spanned up by a set of corner points expressed in the
/// frame name specified
///
/// \param[in] polygonPoints A vector of 3D points on the xy plane (z = 0)
/// \param[in] frameName name of pinocchio frame the polygon is specified in
/// \param[in] scaleFactor A factor to shrink or extent the polygon
///

class ContactPolygon {
 public:
  ContactPolygon(const std::vector<vector3_t>& polygonPoints,
                 const ContactCenterPoint& contactCenterPoint,  // 变量说明：contactCenterPoint 表示接触、center、point。
                 const scalar_t& scaleFactor = 1.0);  // 变量说明：scaleFactor 表示scale、factor。

  // 函数说明：获取number、of、接触、points，连接当前模块的数据流和控制逻辑。
  size_t getNumberOfContactPoints() const { return polygonPoints_.size(); };
  // 函数说明：获取接触、point、translation，连接当前模块的数据流和控制逻辑。
  vector3_t getContactPointTranslation(int index) const { return vector3_t(polygonPoints_[index][0], polygonPoints_[index][1], 0.0); };
  // 函数说明：获取接触、point、translation、cross、product、矩阵，连接当前模块的数据流和控制逻辑。
  matrix3_t getContactPointTranslationCrossProductMatrix(int index) const;
  // 函数说明：获取parent、关节、名称，连接当前模块的数据流和控制逻辑。
  const std::string& getParentJointName() const { return contactCenterPoint_.parentJointName; };
  // 函数说明：获取polygon、point、坐标系/帧、名称，连接当前模块的数据流和控制逻辑。
  const std::string& getPolygonPointFrameName(int i) const { return polygonPointFrameNames_[i]; };
  // 函数说明：获取接触、center、point，连接当前模块的数据流和控制逻辑。
  const ContactCenterPoint& getContactCenterPoint() const { return contactCenterPoint_; };
  // 函数说明：获取bounds，连接当前模块的数据流和控制逻辑。
  const PolygonBounds& getBounds() const { return polygonLimits_; };

 protected:
  // constructor can be used by child class. Child class has to ensure that the polygonLimits are correct
  ContactPolygon(const std::vector<vector3_t>& polygonPoints,
                 const PolygonBounds& polygonBounds,  // 变量说明：polygonBounds 表示polygon、bounds。
                 const ContactCenterPoint& contactCenterPoint);  // 变量说明：contactCenterPoint 表示接触、center、point。

 private:
  // contact dimensions relative to frame attached at contact surface
  PolygonBounds polygonLimits_;  // 变量说明：polygonLimits_ 表示polygon、limits。
  std::vector<vector3_t> polygonPoints_;  // 变量说明：polygonPoints_ 表示polygon、points。
  std::vector<std::string> polygonPointFrameNames_;  // 变量说明：polygonPointFrameNames_ 表示polygon、point、坐标系/帧、names。
  const ContactCenterPoint contactCenterPoint_;  // 变量说明：contactCenterPoint_ 表示接触、center、point。
};  // namespace ContactPolygon

}  // namespace ocs2::humanoid

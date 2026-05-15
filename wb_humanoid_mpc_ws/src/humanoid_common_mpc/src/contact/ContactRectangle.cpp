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

// 自动中文注释：src/humanoid_common_mpc/src/contact/ContactRectangle.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/contact/ContactRectangle.h"

#include "humanoid_common_mpc/common/ModelSettings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2::humanoid {

// 函数说明：处理接触、rectangle，连接当前模块的数据流和控制逻辑。
ContactRectangle::ContactRectangle(const PolygonBounds& polygonBounds,
                                   const ContactCenterPoint& contactCenterPoint,  // 变量说明：contactCenterPoint 表示接触、center、point。
                                   const scalar_t& scaleFactor)
    // 函数说明：处理接触、polygon，连接当前模块的数据流和控制逻辑。
    : ContactPolygon({vector3_t(polygonBounds.x_min * scaleFactor, polygonBounds.y_min * scaleFactor, 0.0),
                      vector3_t(polygonBounds.x_max * scaleFactor, polygonBounds.y_min * scaleFactor, 0.0),
                      vector3_t(polygonBounds.x_max * scaleFactor, polygonBounds.y_max * scaleFactor, 0.0),
                      vector3_t(polygonBounds.x_min * scaleFactor, polygonBounds.y_max * scaleFactor, 0.0)},
                     PolygonBounds(polygonBounds.x_min, polygonBounds.x_max, polygonBounds.y_min, polygonBounds.y_max, scaleFactor),
                     contactCenterPoint) {}

std::vector<vector3_t> ContactRectangle::pointsFromBounds(const PolygonBounds& polygonBounds, const scalar_t& scaleFactor) {  // 变量说明：ContactRectangle 表示接触、rectangle。
  std::vector<vector3_t> polygonPoints = {vector3_t(polygonBounds.x_min * scaleFactor, polygonBounds.y_min * scaleFactor, 0.0),  // 变量说明：polygonPoints 表示polygon、points。
                                          vector3_t(polygonBounds.x_max * scaleFactor, polygonBounds.y_min * scaleFactor, 0.0),
                                          vector3_t(polygonBounds.x_max * scaleFactor, polygonBounds.y_max * scaleFactor, 0.0),
                                          vector3_t(polygonBounds.x_min * scaleFactor, polygonBounds.y_max * scaleFactor, 0.0)};
  return polygonPoints;
}

// 函数说明：加载接触、rectangle，连接当前模块的数据流和控制逻辑。
ContactRectangle ContactRectangle::loadContactRectangle(const std::string& taskFile,  // 变量说明：ContactRectangle 表示接触、rectangle。
                                                        const ModelSettings& modelSettings,  // 变量说明：modelSettings 表示机器人关节和接触配置。
                                                        int contactIndex,  // 变量说明：contactIndex 表示左右脚接触点编号。
                                                        bool verbose) {  // 变量说明：verbose 表示是否打印配置和初始化信息。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "contacts.";  // 变量说明：prefix 表示读取 task.info 时使用的字段前缀。

  scalar_t x_min = 0;  // 变量说明：x_min 表示x、min。
  scalar_t x_max = 0;  // 变量说明：x_max 表示x、max。
  scalar_t y_min = 0;  // 变量说明：y_min 表示y、min。
  scalar_t y_max = 0;  // 变量说明：y_max 表示y、max。
  scalar_t scaleFactor = 1.0;  // 变量说明：scaleFactor 表示scale、factor。
  if (verbose) {
    std::cerr << "\n #### Contact Rectangle Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, x_min, prefix + "contact_rectangle.x_min", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, x_max, prefix + "contact_rectangle.x_max", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, y_min, prefix + "contact_rectangle.y_min", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, y_max, prefix + "contact_rectangle.y_max", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, scaleFactor, prefix + "contact_rectangle.scale_factor", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  ContactCenterPoint ccp(ContactCenterPoint::loadContactCenterPoint(taskFile, modelSettings, contactIndex, verbose));
  return ContactRectangle(PolygonBounds(x_min, x_max, y_min, y_max), ccp, scaleFactor);
}

}  // namespace ocs2::humanoid

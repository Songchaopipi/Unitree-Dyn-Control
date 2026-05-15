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

// 自动中文注释：src/humanoid_common_mpc/src/contact/ContactCenterPoint.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/contact/ContactCenterPoint.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include "humanoid_common_mpc/common/ModelSettings.h"

namespace ocs2::humanoid {

// 函数说明：加载接触、center、point，连接当前模块的数据流和控制逻辑。
ContactCenterPoint ContactCenterPoint::loadContactCenterPoint(const std::string& taskFile,  // 变量说明：ContactCenterPoint 表示接触、center、point。
                                                              const ModelSettings& modelSettings,  // 变量说明：modelSettings 表示机器人关节和接触配置。
                                                              int contactIndex,  // 变量说明：contactIndex 表示左右脚接触点编号。
                                                              bool verbose) {  // 变量说明：verbose 表示是否打印配置和初始化信息。
  assert(contactIndex < N_CONTACTS && "Contact index is out of bound!");
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "contacts.";  // 变量说明：prefix 表示读取 task.info 时使用的字段前缀。

  scalar_t x, y, z;  // 变量说明：x 表示 x 坐标或状态向量，依当前上下文决定。
  if (verbose) {
    std::cerr << "\n #### Contact Center Point Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, x, prefix + "contact_frame_translation.x", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, y, prefix + "contact_frame_translation.y", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, z, prefix + "contact_frame_translation.z", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  vector3_t translationFromParent;  // 变量说明：translationFromParent 表示translation、from、parent。
  translationFromParent << x, y, z;
  std::string frameName = modelSettings.contactNames[contactIndex];  // 变量说明：frameName 表示坐标系/帧、名称。
  std::string parentJointName = modelSettings.contactParentJointNames[contactIndex];  // 变量说明：parentJointName 表示parent、关节、名称。
  return ContactCenterPoint(frameName, parentJointName, translationFromParent);
}

}  // namespace ocs2::humanoid

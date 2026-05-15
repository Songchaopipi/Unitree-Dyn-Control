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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/contact/ContactCenterPoint.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <string>
#include <vector>

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

struct ContactCenterPoint {
  const std::string frameName;  // 变量说明：frameName 表示坐标系/帧、名称。
  const std::string parentJointName;  // 变量说明：parentJointName 表示parent、关节、名称。
  const vector3_t translationFromParent;  // 变量说明：translationFromParent 表示translation、from、parent。

  ContactCenterPoint(const std::string& frameName, const std::string& parentJointName, const vector3_t& translationFromParent)
      : frameName(frameName), parentJointName(parentJointName), translationFromParent(translationFromParent){};

  // 函数说明：加载接触、center、point，连接当前模块的数据流和控制逻辑。
  static ContactCenterPoint loadContactCenterPoint(const std::string& taskFile,
                                                   const ModelSettings& modelSettings,  // 变量说明：modelSettings 表示机器人关节和接触配置。
                                                   int contactIndex,  // 变量说明：contactIndex 表示左右脚接触点编号。
                                                   bool verbose = false);  // 变量说明：verbose 表示是否打印配置和初始化信息。
};

}  // namespace ocs2::humanoid

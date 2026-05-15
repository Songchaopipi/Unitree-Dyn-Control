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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/pinocchio_model/createPinocchioModel.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/contact/ContactPolygon.h"
#include "humanoid_common_mpc/contact/ContactRectangle.h"

namespace ocs2::humanoid {

///
/// \brief Creates a standard pinocchio model from the urdf
///
/// \param[in] urdfFilePath: The absolute path to the URDF file for the robot.
///

PinocchioInterface createDefaultPinocchioInterface(const std::string& urdfFilePath);

///
/// \brief Creates a custom pinocchio model from the urdf by setting all the joints not contained in mpcModelJointNames to
/// FIXED and adds a frame for each corner point of the contact poygons specified in the task file
///
/// \param[in] taskFilePath: The absolute path to the task file used to specify the contact configuration to add a
/// frame for each corner of the contact polygon. \param[in] urdfFilePath: The absolute path to the URDF file for the
/// robot. \param[in] mpcModelJointNames: A list of joint names that are actuated. All other joints are set to FIXED.
///

PinocchioInterface createCustomPinocchioInterface(const std::string& taskFilePath,
                                                  const std::string& urdfFilePath,  // 变量说明：urdfFilePath 表示urdf、file、path。
                                                  const ModelSettings& modelSettings,  // 变量说明：modelSettings 表示机器人关节和接触配置。
                                                  bool scaleTotalMass = false,  // 变量说明：scaleTotalMass 表示scale、total、mass。
                                                  scalar_t totalMass = 1.0,  // 变量说明：totalMass 表示total、mass。
                                                  bool verbose = false);  // 变量说明：verbose 表示是否打印配置和初始化信息。

}  // namespace ocs2::humanoid

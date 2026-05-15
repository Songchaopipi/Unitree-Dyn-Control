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
// 自动中文注释：src/humanoid_centroidal_mpc/include/humanoid_centroidal_mpc/dynamics/DynamicsHelperFunctions.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <pinocchio/fwd.hpp>

#include <array>
#include <cppad/cg.hpp>
#include <iostream>
#include <memory>

#include <pinocchio/algorithm/center-of-mass.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/** Computes an input with zero joint velocity and forces which equally distribute the robot weight between contact
 * feet. */

inline vector_t weightCompensatingInput(const CentroidalModelInfoTpl<scalar_t>& info,
                                        const contact_flag_t& contactFlags,  // 变量说明：contactFlags 表示接触、flags。
                                        const MpcRobotModelBase<scalar_t>& mpcRobotModel) {  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
  // Robot mass stays constant
  const static scalar_t totalGravitationalForce = info.robotMass * 9.81;  // 变量说明：totalGravitationalForce 表示total、gravitational、力。
  const auto numStanceLegs = numberOfLegsInContacts(contactFlags);  // 变量说明：numStanceLegs 表示num、支撑脚/站立、legs。
  vector_t input = vector_t::Zero(mpcRobotModel.getInputDim());  // 变量说明：input 表示输入。
  if (numStanceLegs > 0) {
    const vector3_t forceInInertialFrame(0.0, 0.0, totalGravitationalForce / numStanceLegs);
    for (size_t i = 0; i < contactFlags.size(); i++) {
      if (contactFlags[i]) {
        mpcRobotModel.setContactForce(input, forceInInertialFrame, i);
      }
    }  // end of i loop
  }
  return input;
}

}  // namespace ocs2::humanoid

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

// 自动中文注释：src/humanoid_common_mpc/src/pinocchio_model/pinocchioUtils.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include <humanoid_common_mpc/pinocchio_model/pinocchioUtils.h>

#include <fstream>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/multibody/model.hpp>
#include "pinocchio/parsers/urdf.hpp"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void checkPinocchioJointNaming(const PinocchioInterface& pinocchioInterface, const ModelSettings& modelSettings, bool verbose) {
  const pinocchio::Model& model = pinocchioInterface.getModel();  // 变量说明：model 表示模型。
  for (size_t i = 0; i < modelSettings.mpcModelJointNames.size(); i++) {
    if (verbose) {
      std::cout << "URDF Joint Name " << i << ": " << model.names[i + 2] << std::endl;
      std::cout << "Model Settings Joint Name " << i << ": " << modelSettings.mpcModelJointNames[i] << std::endl;
    }
    // Offset of 2 required to skip universe and root joint
    assert(modelSettings.mpcModelJointNames[i] == model.names[i + 2] && "Joint name of PinocchioModel and Model Settings do not match!");
  }
  if (verbose) {
    std::cout << "Joint naming check of pinocchio model passed. " << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::pair<vector_t, vector_t> readPinocchioJointLimits(const PinocchioInterface& pinocchioInterface,
                                                       const ModelSettings& modelSettings,  // 变量说明：modelSettings 表示机器人关节和接触配置。
                                                       bool verbose) {  // 变量说明：verbose 表示是否打印配置和初始化信息。
  // check that Pinocchio Model joint naming and order is identical to model setting.
  checkPinocchioJointNaming(pinocchioInterface, modelSettings);
  const pinocchio::Model& model = pinocchioInterface.getModel();  // 变量说明：model 表示模型。
  // Take the tail to avoid limits of universe and root joints
  vector_t upper_limits = model.upperPositionLimit.tail(modelSettings.mpcModelJointNames.size());  // 变量说明：upper_limits 表示upper、limits。
  vector_t lower_limits = model.lowerPositionLimit.tail(modelSettings.mpcModelJointNames.size());  // 变量说明：lower_limits 表示lower、limits。
  if (verbose) {
    std::cout << "Joint Name , min, max" << std::endl;
    for (size_t i = 0; i < modelSettings.mpcModelJointNames.size(); i++) {
      std::cout << modelSettings.mpcModelJointNames[i] << ": " << lower_limits[i] << ", " << upper_limits[i] << std::endl;
    }
  }
  return {lower_limits, upper_limits};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void scalePinocchioModelInertia(pinocchio::ModelTpl<scalar_t>& model, scalar_t targetRobotMass, bool verbose) {
  scalar_t robotMass = pinocchio::computeTotalMass(model);  // 变量说明：robotMass 表示机器人、mass。
  scalar_t inertiaScaleFactor = targetRobotMass / robotMass;  // 变量说明：inertiaScaleFactor 表示inertia、scale、factor。
  if (verbose) {
    std::cout << "Current robot mass: " << robotMass << std::endl;
    std::cout << "Target robot mass: " << targetRobotMass << std::endl;
    std::cout << "Adapting robot mass by a factor of " << inertiaScaleFactor << "." << std::endl;
  }
  for (size_t i = 0; i < model.inertias.size(); i++) {
    const auto inertia = model.inertias[i];  // 变量说明：inertia 表示inertia。
    auto scaledMass = inertia.mass() * inertiaScaleFactor;  // 变量说明：scaledMass 表示scaled、mass。
    matrix3_t inertiaMatrix = inertia.inertia().matrix();  // 变量说明：inertiaMatrix 表示inertia、矩阵。
    inertiaMatrix = inertiaMatrix * inertiaScaleFactor;
    auto scaledInertia = pinocchio::Symmetric3(inertiaMatrix);  // 变量说明：scaledInertia 表示scaled、inertia。
    model.inertias[i] = pinocchio::ModelTpl<scalar_t>::Inertia(scaledMass, inertia.lever(), scaledInertia);
  }
  if (verbose) {
    std::cout << "Robot mass scaled to " << pinocchio::computeTotalMass(model) << "." << std::endl;
  }
}

}  // namespace ocs2::humanoid

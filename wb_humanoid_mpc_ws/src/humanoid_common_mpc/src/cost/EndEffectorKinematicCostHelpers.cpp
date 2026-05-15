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

// 自动中文注释：src/humanoid_common_mpc/src/cost/EndEffectorKinematicCostHelpers.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/cost/EndEffectorKinematicCostHelpers.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2::humanoid {

// 函数说明：转换向量，连接当前模块的数据流和控制逻辑。
vector12_t EndEffectorKinematicsWeights::toVector() {  // 变量说明：EndEffectorKinematicsWeights 表示end、effector、kinematics、weights。
  vector12_t weightVector;  // 变量说明：weightVector 表示权重、向量。
  weightVector << contactPositionErrorWeight, contactOrientationErrorWeight, contactLinearVelocityErrorWeight,
      contactAngularVelocityErrorWeight;
  return weightVector;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorKinematicsWeights EndEffectorKinematicsWeights::getWeights(const std::string& taskFile, const std::string prefix, bool verbose) {  // 变量说明：EndEffectorKinematicsWeights 表示end、effector、kinematics、weights。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile, pt);

  // Load all weights
  scalar_t pos_x = 0;  // 变量说明：pos_x 表示位置、x。
  scalar_t pos_y = 0;  // 变量说明：pos_y 表示位置、y。
  scalar_t pos_z = 0;  // 变量说明：pos_z 表示位置、z。
  scalar_t orientation_x = 0;  // 变量说明：orientation_x 表示姿态、x。
  scalar_t orientation_y = 0;  // 变量说明：orientation_y 表示姿态、y。
  scalar_t orientation_z = 0;  // 变量说明：orientation_z 表示姿态、z。
  scalar_t lin_velocity_x = 0;  // 变量说明：lin_velocity_x 表示lin、速度、x。
  scalar_t lin_velocity_y = 0;  // 变量说明：lin_velocity_y 表示lin、速度、y。
  scalar_t lin_velocity_z = 0;  // 变量说明：lin_velocity_z 表示lin、速度、z。
  scalar_t ang_velocity_x = 0;  // 变量说明：ang_velocity_x 表示ang、速度、x。
  scalar_t ang_velocity_y = 0;  // 变量说明：ang_velocity_y 表示ang、速度、y。
  scalar_t ang_velocity_z = 0;  // 变量说明：ang_velocity_z 表示ang、速度、z。
  if (verbose) {
    std::cerr << "\n #### End Effector Kinematics Quadratic Cost Weights: ";
    std::cerr << "Loading weigths from: " << prefix;
    std::cerr << "\n #### =============================================================================\n";
  }
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, pos_x, prefix + "pos_x", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, pos_y, prefix + "pos_y", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, pos_z, prefix + "pos_z", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, orientation_x, prefix + "orientation_x", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, orientation_y, prefix + "orientation_y", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, orientation_z, prefix + "orientation_z", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, lin_velocity_x, prefix + "lin_velocity_x", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, lin_velocity_y, prefix + "lin_velocity_y", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, lin_velocity_z, prefix + "lin_velocity_z", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, ang_velocity_x, prefix + "ang_velocity_x", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, ang_velocity_y, prefix + "ang_velocity_y", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, ang_velocity_z, prefix + "ang_velocity_z", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  EndEffectorKinematicsWeights weights;  // 变量说明：weights 表示代价/任务权重。

  weights.contactPositionErrorWeight = vector3_t(pos_x, pos_y, pos_z);
  weights.contactOrientationErrorWeight = vector3_t(orientation_x, orientation_y, orientation_z);
  weights.contactLinearVelocityErrorWeight = vector3_t(lin_velocity_x, lin_velocity_y, lin_velocity_z);
  weights.contactAngularVelocityErrorWeight = vector3_t(ang_velocity_x, ang_velocity_y, ang_velocity_z);

  return weights;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<std::string> EndEffectorKinematicsWeights::getDescriptions() {  // 变量说明：EndEffectorKinematicsWeights 表示end、effector、kinematics、weights。
  return {"pos_x",          "pos_y",          "pos_z",          "orientation_x",  "orientation_y",  "orientation_z",
          "lin_velocity_x", "lin_velocity_y", "lin_velocity_z", "ang_velocity_x", "ang_velocity_y", "ang_velocity_z"};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
// 函数说明：计算task、space、errors，连接当前模块的数据流和控制逻辑。
VECTOR12_T<SCALAR_T> computeTaskSpaceErrors(const EndEffectorKinematicsCostElement<SCALAR_T>& current,
                                            const EndEffectorKinematicsCostElement<SCALAR_T>& reference) {  // 变量说明：reference 表示参考。
  const VECTOR3_T<SCALAR_T> orientationError = quaternionDistance<SCALAR_T>(current.getOrientation(), reference.getOrientation());  // 变量说明：orientationError 表示姿态、误差。

  VECTOR12_T<SCALAR_T> errors;  // 变量说明：errors 表示当前任务 residual/误差向量。
  errors << (current.getPosition() - reference.getPosition()), orientationError,
      (current.getLinearVelocity() - reference.getLinearVelocity()), (current.getAngularVelocity() - reference.getAngularVelocity());
  return errors;
}
// 函数说明：计算task、space、errors，连接当前模块的数据流和控制逻辑。
template VECTOR12_T<scalar_t> computeTaskSpaceErrors(const EndEffectorKinematicsCostElement<scalar_t>& current,
                                                     const EndEffectorKinematicsCostElement<scalar_t>& reference);  // 变量说明：reference 表示参考。
// 函数说明：计算task、space、errors，连接当前模块的数据流和控制逻辑。
template VECTOR12_T<ad_scalar_t> computeTaskSpaceErrors(const EndEffectorKinematicsCostElement<ad_scalar_t>& current,
                                                        const EndEffectorKinematicsCostElement<ad_scalar_t>& reference);  // 变量说明：reference 表示参考。

}  // namespace ocs2::humanoid

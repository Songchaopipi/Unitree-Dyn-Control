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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/common/ModelSettings.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <array>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

class ModelSettings {
 public:
  // 足端约束的反馈增益。MPC 的约束通常写成“速度/加速度误差 = 增益 * 位置/姿态误差”，
  // 这些参数决定接触脚贴地、足底姿态保持和切换相位时的约束强度。
  struct FootConstraintConfig {
    scalar_t positionErrorGain_z{1.0};  // 变量说明：positionErrorGain_z 表示位置、误差、增益、z。
    scalar_t orientationErrorGain{1.0};  // 变量说明：orientationErrorGain 表示姿态、误差、增益。
    scalar_t linearVelocityErrorGain_z{1.0};  // 变量说明：linearVelocityErrorGain_z 表示linear、速度、误差、增益、z。
    scalar_t linearVelocityErrorGain_xy{1.0};  // 变量说明：linearVelocityErrorGain_xy 表示linear、速度、误差、增益、xy。
    scalar_t angularVelocityErrorGain{1.0};  // 变量说明：angularVelocityErrorGain 表示angular、速度、误差、增益。
    scalar_t linearAccelerationErrorGain_z{1.0};  // 变量说明：linearAccelerationErrorGain_z 表示linear、加速度、误差、增益、z。
    scalar_t linearAccelerationErrorGain_xy{1.0};  // 变量说明：linearAccelerationErrorGain_xy 表示linear、加速度、误差、增益、xy。
    scalar_t angularAccelerationErrorGain{1.0};  // 变量说明：angularAccelerationErrorGain 表示angular、加速度、误差、增益。
  };

  ModelSettings(const std::string& configFile, const std::string& urdfFile, const std::string& mpcName, bool verbose = "false");

  ModelSettings() = delete;

  ModelSettings(const ModelSettings&) = delete;

 public:
  // 机器人和自动微分代码生成设置：robotName 用于拼接 CppAD 代码目录，
  // verbose/recompile 控制是否打印和重新生成动力学、代价、约束的动态库。
  std::string robotName;  // 变量说明：robotName 表示机器人、名称。

  bool verboseCppAd = true;  // 变量说明：verboseCppAd 表示verbose、cpp、自动微分。
  bool recompileLibrariesCppAd = true;  // 变量说明：recompileLibrariesCppAd 表示recompile、libraries、cpp、自动微分。
  std::string modelFolderCppAd = "build/cppad_autocode_gen";  // 变量说明：modelFolderCppAd 表示模型、folder、cpp、自动微分。

  // 相位切换附近额外插入的站立时间，用于避免刚触地/离地时约束突变太硬。
  scalar_t phaseTransitionStanceTime;  // 变量说明：phaseTransitionStanceTime 表示步态相位、transition、支撑脚/站立、时间。
  bool armSwingReferenceActive = true;  // 变量说明：armSwingReferenceActive 表示是否启用手臂摆动参考。

  // fullJointNames 是 URDF/MuJoCo 的完整关节顺序；fixedJointNames 会从 MPC 决策变量中剔除。
  // 本工程里腕部关节通常固定，MPC 只优化腿、腰和主要手臂关节。
  std::vector<std::string> fullJointNames;  // 变量说明：fullJointNames 表示完整模型、关节、names。
  std::vector<std::string> fixedJointNames;  // 变量说明：fixedJointNames 表示固定关节、关节、names。

  // 接触点配置。6DoF 接触包含力和力矩，3DoF 接触只包含力；
  // contactParentJointNames 用来给 Pinocchio 添加足底接触 frame。
  std::vector<std::string> contactNames6DoF;  // 变量说明：contactNames6DoF 表示接触、names6、do、f。
  std::vector<std::string> contactNames3DoF{};  // 变量说明：contactNames3DoF 表示接触、names3、do、f。
  std::vector<std::string> contactParentJointNames;  // 变量说明：contactParentJointNames 表示接触、parent、关节、names。

  // MPC 内部关节顺序和完整模型关节顺序之间的映射。所有 policy 的 q_j/qd_j/tau_j 都按 mpcModelJointNames 排列。
  std::vector<std::string> mpcModelJointNames;      // Active joints (all joints except the fixed ones)
  std::vector<size_t> mpcModelToFullJointsIndices;  // an Array of indices mapping the active joints to the full joints
  std::unordered_map<std::string, size_t> jointIndexMap;
  std::vector<std::string> contactNames;  // containing all 3Dof and 6Dof contacts

  // mpc_joint_dim 是优化模型关节数，full_joint_dim 是完整机器人关节数。
  size_t mpc_joint_dim;  // 变量说明：mpc_joint_dim 表示MPC、关节、维度。
  size_t full_joint_dim;  // 变量说明：full_joint_dim 表示完整模型、关节、维度。

  // 上肢 mimic 约束使用的关节名和索引：让肩/肘在简化模型里保持成组运动。
  std::string j_l_shoulder_y_name;  // 变量说明：j_l_shoulder_y_name 表示j、l、shoulder、y、名称。
  std::string j_r_shoulder_y_name;  // 变量说明：j_r_shoulder_y_name 表示j、r、shoulder、y、名称。
  std::string j_l_elbow_y_name;  // 变量说明：j_l_elbow_y_name 表示j、l、elbow、y、名称。
  std::string j_r_elbow_y_name;  // 变量说明：j_r_elbow_y_name 表示j、r、elbow、y、名称。

  size_t j_l_shoulder_y_index;  // 变量说明：j_l_shoulder_y_index 表示j、l、shoulder、y、索引。
  size_t j_r_shoulder_y_index;  // 变量说明：j_r_shoulder_y_index 表示j、r、shoulder、y、索引。
  size_t j_l_elbow_y_index;  // 变量说明：j_l_elbow_y_index 表示j、l、elbow、y、索引。
  size_t j_r_elbow_y_index;  // 变量说明：j_r_elbow_y_index 表示j、r、elbow、y、索引。

  FootConstraintConfig footConstraintConfig;  // 变量说明：footConstraintConfig 表示足端、约束、config。
};

}  // namespace ocs2::humanoid

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

// 自动中文注释：src/humanoid_common_mpc/src/common/ModelSettings.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/common/ModelSettings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <stdexcept>
#include <unordered_set>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/pinocchio_model/createPinocchioModel.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/// Helper functions contained in a local anonymous namespace
/******************************************************************************************************/
namespace {

/**
 * @brief Creates a joint Index map from a list of joint names.
 */

static std::unordered_map<std::string, size_t> createJointIndexMap(const std::vector<std::string>& jointNames, size_t offset = 0) {  // 变量说明：该对象使用标准库容器/数组保存相关数据。
  std::unordered_map<std::string, size_t> jointIndexMap;
  for (size_t i = 0; i < jointNames.size(); ++i) {
    jointIndexMap[jointNames[i]] = i + offset;
  }
  return jointIndexMap;
}

// 函数说明：初始化关节、names，连接当前模块的数据流和控制逻辑。
static std::vector<std::string> initializeJointNames(const std::vector<std::string>& fullJointNames,
                                                     const std::vector<std::string>& fixedJointNames,  // 变量说明：fixedJointNames 表示固定关节、关节、names。
                                                     bool verbose) {  // 变量说明：verbose 表示是否打印配置和初始化信息。
  const std::unordered_set<std::string> fixedJointNameSet(fixedJointNames.begin(), fixedJointNames.end());
  const std::unordered_set<std::string> fullJointNameSet(fullJointNames.begin(), fullJointNames.end());
  for (const auto& fixedJoint : fixedJointNameSet) {
    if (fullJointNameSet.find(fixedJoint) == fullJointNameSet.end()) {
      throw std::invalid_argument("Configured fixed joint is missing from URDF: " + fixedJoint);
    }
  }

  if (verbose) std::cout << "Initialize the following active MPC joints: " << std::endl;
  const size_t n_joints = fullJointNames.size() - fixedJointNameSet.size();  // 变量说明：n_joints 表示n、joints。
  std::vector<std::string> mpcModelJointNames;  // 变量说明：mpcModelJointNames 表示MPC、模型、关节、names。
  if (n_joints > 0) {
    mpcModelJointNames.reserve(n_joints);
  } else {
    throw std::invalid_argument("Number of joints must be greater than zero");
  }
  for (const auto& joint : fullJointNames) {
    if (fixedJointNameSet.find(joint) == fixedJointNameSet.end()) {
      // If the joint is not found in fixedJointNames, add it to mpcModelJointNames
      if (verbose) std::cout << joint << std::endl;
      mpcModelJointNames.emplace_back(joint);
    }
  }
  if (verbose) std::cout << "Num active joints: " << mpcModelJointNames.size() << std::endl;
  return mpcModelJointNames;
}

std::vector<size_t> initializeMpcToFullJointIndices(const std::vector<std::string>& fullJointNames,
                                                    const std::vector<std::string>& mpcModelJointNames) {  // 变量说明：mpcModelJointNames 表示MPC、模型、关节、names。
  std::unordered_map<std::string, size_t> fullJointIndexMap = createJointIndexMap(fullJointNames);
  std::vector<size_t> mpcModelJointIndices;  // 变量说明：mpcModelJointIndices 表示MPC、模型、关节、indices。
  mpcModelJointIndices.reserve(mpcModelJointNames.size());
  for (const auto& jointName : mpcModelJointNames) {
    const auto it = fullJointIndexMap.find(jointName);
    if (it == fullJointIndexMap.end()) {
      throw std::invalid_argument("MPC joint is missing from full joint list: " + jointName);
    }
    mpcModelJointIndices.emplace_back(it->second);
  }
  return mpcModelJointIndices;
}

std::vector<std::string> concatenateStringVectors(const std::vector<std::string>& a, const std::vector<std::string>& b) {
  std::vector<std::string> temp_vec(a);
  temp_vec.insert(temp_vec.begin(), b.begin(), b.end());
  return temp_vec;
}

}  // namespace

// 函数说明：处理模型、配置，连接当前模块的数据流和控制逻辑。
ModelSettings::ModelSettings(const std::string& configFile, const std::string& urdfFile, const std::string& mpcName, bool verbose) {
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(configFile, pt);

  std::string prefix{"model_settings."};  // 变量说明：prefix 表示读取 task.info 时使用的字段前缀。

  if (verbose) {
    std::cerr << "\n #### Robot Model Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->robotName, prefix + "robotName", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->verboseCppAd, prefix + "verboseCppAd", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->recompileLibrariesCppAd, prefix + "recompileLibrariesCppAd", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->phaseTransitionStanceTime, prefix + "phaseTransitionStanceTime", verbose);
  loadData::loadPtreeValue(pt, this->armSwingReferenceActive, prefix + "armSwingReferenceActive", verbose);

  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->j_l_shoulder_y_name, prefix + "armJointNames.left_shoulder_y", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->j_r_shoulder_y_name, prefix + "armJointNames.right_shoulder_y", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->j_l_elbow_y_name, prefix + "armJointNames.left_elbow_y", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->j_r_elbow_y_name, prefix + "armJointNames.right_elbow_y", verbose);
  modelFolderCppAd = "cppad_code_gen/cppad_" + mpcName + robotName;

  // 函数说明：加载std、向量，连接当前模块的数据流和控制逻辑。
  loadData::loadStdVector(configFile, prefix + "fixedJointNames", fixedJointNames, verbose);
  // 函数说明：加载std、向量，连接当前模块的数据流和控制逻辑。
  loadData::loadStdVector(configFile, prefix + "contactNames6DoF", contactNames6DoF, verbose);
  // 函数说明：加载std、向量，连接当前模块的数据流和控制逻辑。
  loadData::loadStdVector(configFile, prefix + "contactParentJointNames", contactParentJointNames, verbose);

  if (verbose) {
    std::cout << "Initializing MPC by fixing joints: " << std::endl;
    for (std::string fixedJoint : fixedJointNames) std::cout << fixedJoint << std::endl;
  }

  // Get full joint order from a full pinocchio interface, this removes any joints marked as fix in the urdf.
  PinocchioInterface fullPinocchioInterface = createDefaultPinocchioInterface(urdfFile);  // 变量说明：fullPinocchioInterface 表示完整模型、Pinocchio、interface。
  const pinocchio::Model& model = fullPinocchioInterface.getModel();  // 变量说明：model 表示模型。
  if (verbose) std::cout << "Full URDF joints: " << std::endl;
  fullJointNames.reserve(model.njoints - 2);  // Substract universe and root joint
  for (pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id) {
    if (verbose) std::cout << model.names[joint_id] << std::endl;
    fullJointNames.emplace_back(model.names[joint_id]);
  }

  this->mpcModelJointNames = initializeJointNames(this->fullJointNames, this->fixedJointNames, verbose);
  this->mpcModelToFullJointsIndices = initializeMpcToFullJointIndices(this->fullJointNames, this->mpcModelJointNames);
  this->jointIndexMap = createJointIndexMap(this->mpcModelJointNames);
  this->contactNames = concatenateStringVectors(this->contactNames3DoF, this->contactNames6DoF);

  this->mpc_joint_dim = this->mpcModelJointNames.size();
  this->full_joint_dim = this->fullJointNames.size();

  j_l_shoulder_y_index = this->jointIndexMap.at(j_l_shoulder_y_name);
  j_r_shoulder_y_index = this->jointIndexMap.at(j_r_shoulder_y_name);
  j_l_elbow_y_index = this->jointIndexMap.at(j_l_elbow_y_name);
  j_r_elbow_y_index = this->jointIndexMap.at(j_r_elbow_y_name);

  const std::string footConstraintPrefix = prefix + "foot_constraint.";  // 变量说明：footConstraintPrefix 表示足端、约束、prefix。

  if (verbose) {
    std::cerr << "\n #### Robot Model Foot Constraint Config:";
    std::cerr << "\n #### =============================================================================\n";
  }

  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->footConstraintConfig.positionErrorGain_z, footConstraintPrefix + "positionErrorGain_z", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->footConstraintConfig.orientationErrorGain, footConstraintPrefix + "orientationErrorGain", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearVelocityErrorGain_z, footConstraintPrefix + "linearVelocityErrorGain_z",
                           verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearVelocityErrorGain_xy, footConstraintPrefix + "linearVelocityErrorGain_xy",
                           verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->footConstraintConfig.angularVelocityErrorGain, footConstraintPrefix + "angularVelocityErrorGain",
                           verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearAccelerationErrorGain_z,
                           footConstraintPrefix + "linearAccelerationErrorGain_z", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->footConstraintConfig.linearAccelerationErrorGain_xy,
                           footConstraintPrefix + "linearAccelerationErrorGain_xy", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, this->footConstraintConfig.angularAccelerationErrorGain,
                           footConstraintPrefix + "angularAccelerationErrorGain", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }
}

}  // namespace ocs2::humanoid

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

// 自动中文注释：src/humanoid_common_mpc/src/constraint/FootCollisionConstraint.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include <pinocchio/fwd.hpp>

#include "humanoid_common_mpc/constraint/FootCollisionConstraint.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

FootCollisionConstraint::FootCollisionConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                 const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                 const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                                 const Config& config,  // 变量说明：config 表示从配置文件读取出的参数集合。
                                                 std::string costName,  // 变量说明：costName 表示自动微分代价库名称。
                                                 const ModelSettings& modelSettings)
    // 函数说明：处理状态、约束、cpp、自动微分，连接当前模块的数据流和控制逻辑。
    : StateConstraintCppAd(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelPtr_(&mpcRobotModel),
      cfg_(std::move(config)) {
  initialize(mpcRobotModelPtr_->getStateDim(), 2, costName, modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd,
             modelSettings.verboseCppAd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

FootCollisionConstraint::FootCollisionConstraint(const FootCollisionConstraint& other)
    // 函数说明：处理状态、约束、cpp、自动微分，连接当前模块的数据流和控制逻辑。
    : StateConstraintCppAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelPtr_(other.mpcRobotModelPtr_),
      cfg_(other.cfg_),
      numConstraints_(other.numConstraints_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FootCollisionConstraint::isActive(scalar_t time) const {  // 变量说明：FootCollisionConstraint 表示足端、collision、约束。
  if (!isActive_) return false;

  // Inactivate the constraint if both feet are in contact. Prevents it from fighting against the stance foot constraints.
  auto contactFlags = referenceManagerPtr_->getContactFlags(time);  // 变量说明：contactFlags 表示接触、flags。
  return !(contactFlags[0] && contactFlags[1]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t FootCollisionConstraint::constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {  // 变量说明：FootCollisionConstraint 表示足端、collision、约束。
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;  // 变量说明：rf 表示足端速度/Jacobian 使用的 LOCAL_WORLD_ALIGNED 参考系。

  const auto& model = pinocchioInterfaceCppAd_.getModel();  // 变量说明：model 表示模型。
  auto data = pinocchioInterfaceCppAd_.getData();  // 变量说明：data 表示数据。

  const ad_vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(state);  // 变量说明：q 表示广义坐标/关节位置。
  pinocchio::forwardKinematics(model, data, q);

  // Ankle collision points
  ad_vector3_t pos_ankle_l = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftAnkleFrame)).translation();  // 变量说明：pos_ankle_l 表示位置、ankle、l。
  ad_vector3_t pos_ankle_r = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightAnkleFrame)).translation();  // 变量说明：pos_ankle_r 表示位置、ankle、r。

  // Foot collision points
  ad_vector3_t pos_f_l = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftFootCenterFrame)).translation();  // 变量说明：pos_f_l 表示位置、f、l。
  ad_vector3_t pos_f_r = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightFootCenterFrame)).translation();  // 变量说明：pos_f_r 表示位置、f、r。
  ad_vector3_t pos_f_l_p1 = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftFootFrame1)).translation();  // 变量说明：pos_f_l_p1 表示位置、f、l、p1。
  ad_vector3_t pos_f_r_p1 = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightFootFrame1)).translation();  // 变量说明：pos_f_r_p1 表示位置、f、r、p1。
  ad_vector3_t pos_f_l_p2 = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftFootFrame2)).translation();  // 变量说明：pos_f_l_p2 表示位置、f、l、p2。
  ad_vector3_t pos_f_r_p2 = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightFootFrame2)).translation();  // 变量说明：pos_f_r_p2 表示位置、f、r、p2。

  // Knee collision points
  ad_vector3_t pos_k_l = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftKneeFrame)).translation();  // 变量说明：pos_k_l 表示位置、k、l。
  ad_vector3_t pos_k_r = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightKneeFrame)).translation();  // 变量说明：pos_k_r 表示位置、k、r。

  // Calcualte the square of the min distance (2*radius)^2
  // parameters[0] is the collision sphere radius
  ad_scalar_t minDistFoot = 2.0 * parameters[0];  // 变量说明：minDistFoot 表示min、dist、足端。
  ad_scalar_t minDistKnee = 2.0 * parameters[1];  // 变量说明：minDistKnee 表示min、dist、knee。

  // 函数说明：处理约束、values，连接当前模块的数据流和控制逻辑。
  ad_vector_t constraintValues(numConstraints_);
  constraintValues[0] = ((pos_f_l_p1 - pos_f_r_p1).norm() - minDistFoot);
  constraintValues[1] = ((pos_f_l_p1 - pos_f_r_p2).norm() - minDistFoot);
  constraintValues[2] = ((pos_f_l_p2 - pos_f_r_p1).norm() - minDistFoot);
  constraintValues[3] = ((pos_f_l_p2 - pos_f_r_p2).norm() - minDistFoot);

  constraintValues[4] = ((pos_f_l - pos_f_r_p1).norm() - minDistFoot);
  constraintValues[5] = ((pos_f_l - pos_f_r_p2).norm() - minDistFoot);
  constraintValues[6] = ((pos_f_r - pos_f_l_p1).norm() - minDistFoot);
  constraintValues[7] = ((pos_f_r - pos_f_l_p2).norm() - minDistFoot);
  constraintValues[8] = ((pos_f_l - pos_f_r).norm() - minDistFoot);

  constraintValues[9] = ((pos_k_l - pos_k_r).norm() - minDistKnee);

  constraintValues[10] = ((pos_f_l - pos_ankle_r).norm() - minDistFoot);
  constraintValues[11] = ((pos_f_l_p1 - pos_ankle_r).norm() - minDistFoot);
  constraintValues[12] = ((pos_f_l_p2 - pos_ankle_r).norm() - minDistFoot);
  constraintValues[13] = ((pos_f_r - pos_ankle_l).norm() - minDistFoot);
  constraintValues[14] = ((pos_f_r_p1 - pos_ankle_l).norm() - minDistFoot);
  constraintValues[15] = ((pos_f_r_p2 - pos_ankle_l).norm() - minDistFoot);

  return constraintValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

FootCollisionConstraint::Config FootCollisionConstraint::loadFootCollisionConstraintConfig(const std::string taskFile, bool verbose) {  // 变量说明：FootCollisionConstraint 表示足端、collision、约束。
  boost::property_tree::ptree pt;  // 变量说明：pt 表示 Boost property_tree 配置树。
  // 函数说明：读取模型信息，连接当前模块的数据流和控制逻辑。
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "collision_constraint.";  // 变量说明：prefix 表示读取 task.info 时使用的字段前缀。

  Config collisionConfig;  // 变量说明：collisionConfig 表示collision、config。

  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, collisionConfig.leftAnkleFrame, prefix + "foot.leftAnkleFrame", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, collisionConfig.rightAnkleFrame, prefix + "foot.rightAnkleFrame", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, collisionConfig.footCollisionSphereRadius, prefix + "foot.footCollisionSphereRadius", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, collisionConfig.leftKneeFrame, prefix + "knee.leftKneeFrame", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, collisionConfig.rightKneeFrame, prefix + "knee.rightKneeFrame", verbose);
  // 函数说明：加载ptree、数值，连接当前模块的数据流和控制逻辑。
  loadData::loadPtreeValue(pt, collisionConfig.kneeCollisionSphereRadius, prefix + "knee.kneeCollisionSphereRadius", verbose);

  return collisionConfig;
}

}  // namespace ocs2::humanoid

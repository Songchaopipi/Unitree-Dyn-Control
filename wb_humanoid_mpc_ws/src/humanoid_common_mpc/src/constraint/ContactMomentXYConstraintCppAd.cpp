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

// 自动中文注释：src/humanoid_common_mpc/src/constraint/ContactMomentXYConstraintCppAd.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include <pinocchio/fwd.hpp>

#include "humanoid_common_mpc/constraint/ContactMomentXYConstraintCppAd.h"

#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ContactMomentXYConstraintCppAd::ContactMomentXYConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                               const ContactRectangle& contactRectangle,  // 变量说明：contactRectangle 表示接触、rectangle。
                                                               size_t contactPointIndex,  // 变量说明：contactPointIndex 表示接触、point、索引。
                                                               const PinocchioInterface& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                                                               const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。
                                                               std::string costName,  // 变量说明：costName 表示自动微分代价库名称。
                                                               const ModelSettings& modelSettings)
    // 函数说明：处理状态、输入、约束、cpp、自动微分，连接当前模块的数据流和控制逻辑。
    : StateInputConstraintCppAd(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      mpcRobotModelPtr_(&mpcRobotModel),
      contactRectangle_(contactRectangle),
      contactPointIndex_(contactPointIndex),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()) {
  initialize(mpcRobotModelPtr_->getStateDim(), mpcRobotModelPtr_->getInputDim(), 0, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ContactMomentXYConstraintCppAd::ContactMomentXYConstraintCppAd(const ContactMomentXYConstraintCppAd& other)
    // 函数说明：处理状态、输入、约束、cpp、自动微分，连接当前模块的数据流和控制逻辑。
    : StateInputConstraintCppAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      mpcRobotModelPtr_(other.mpcRobotModelPtr_),
      contactRectangle_(other.contactRectangle_),
      contactPointIndex_(other.contactPointIndex_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ContactMomentXYConstraintCppAd::isActive(scalar_t time) const {  // 变量说明：ContactMomentXYConstraintCppAd 表示接触、moment、xyconstraint、cpp、自动微分。
  if (!isActive_) return false;
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t ContactMomentXYConstraintCppAd::constraintFunction(ad_scalar_t time,  // 变量说明：ContactMomentXYConstraintCppAd 表示接触、moment、xyconstraint、cpp、自动微分。
                                                               const ad_vector_t& state,  // 变量说明：state 表示状态。
                                                               const ad_vector_t& input,  // 变量说明：input 表示输入。
                                                               const ad_vector_t& parameters) const {  // 变量说明：parameters 表示传给自动微分代价的参数向量。
  const auto& model = pinocchioInterfaceCppAd_.getModel();  // 变量说明：model 表示模型。
  auto data = pinocchioInterfaceCppAd_.getData();  // make copy of model since method is const
  updateFramePlacements(mpcRobotModelPtr_->getGeneralizedCoordinates(state), model, data);
  pinocchio::FrameIndex frameID = getContactFrameIndex(pinocchioInterfaceCppAd_, *mpcRobotModelPtr_, contactPointIndex_);  // 变量说明：frameID 表示坐标系/帧、编号。

  const ad_vector3_t localForce = rotateVectorWorldToLocal(mpcRobotModelPtr_->getContactForce(input, contactPointIndex_), data, frameID);  // 变量说明：localForce 表示local、力。
  const ad_vector3_t localMoments = rotateVectorWorldToLocal(mpcRobotModelPtr_->getContactMoment(input, contactPointIndex_), data, frameID);  // 变量说明：localMoments 表示local、moments。

  // 函数说明：处理约束、数值，连接当前模块的数据流和控制逻辑。
  ad_vector_t constraintValue(4);
  constraintValue << localMoments.x() - contactRectangle_.getBounds().y_min * localForce.z(),
      -localMoments.x() + contactRectangle_.getBounds().y_max * localForce.z(),
      -localMoments.y() - contactRectangle_.getBounds().x_min * localForce.z(),
      localMoments.y() + contactRectangle_.getBounds().x_max * localForce.z();
  return constraintValue;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

}  // namespace ocs2::humanoid

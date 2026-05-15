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
// 自动中文注释：src/humanoid_centroidal_mpc/include/humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/* State Vector definition

  Define the state vector x = [h, q_b, q_j]^T
  中文说明：这是本工程最核心的 MPC 状态排布。
  - h 是单位质量归一化后的质心动量，前 3 维近似质心线速度，后 3 维是角动量 / mass。
  - q_b 是浮动基座位姿，采用 [x, y, z, yaw, pitch, roll]，不是四元数。
  - q_j 是 active joints，顺序来自 ModelSettings::mpcModelJointNames。

    Centroidal momentum h = [vcom_x, vcom_y, vcom_z, L_x / mass, L_y / mass, L_z / mass]^T
    Base pose q_b = [p_base_x, p_base_y, p_base_z, theta_base_z, theta_base_y, theta_base_x]
    Joint angles q_j

*/
/******************************************************************************************************/

/******************************************************************************************************/
/* Input Vector definition

  Define the input vector u = [W_l, W_r, q_dot_j]^T
  中文说明：输入前 12 维是左右脚接触 wrench，后面是 active joint velocity。
  这里的 W_l/W_r 都在世界系表达，力和力矩顺序为 [fx, fy, fz, mx, my, mz]。

    Left contact Wrench W_l in inertial frame
    Right contact Wrench W_r in inertial frame
    Joint velocities q_dot_j

*/
/******************************************************************************************************/

template <typename SCALAR_T>
class CentroidalMpcRobotModel : public MpcRobotModelBase<SCALAR_T> {
 public:
  CentroidalMpcRobotModel(const ModelSettings& modelSettings,
                          const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,  // 变量说明：pinocchioInterface 表示 Pinocchio 模型接口。
                          const CentroidalModelInfoTpl<SCALAR_T>& centroidalModelInfo)
      : MpcRobotModelBase<SCALAR_T>(modelSettings, 12 + modelSettings.mpc_joint_dim, 6 * N_CONTACTS + modelSettings.mpc_joint_dim),
        pinocchioInterface_(pinocchioInterface),
        centroidalModelInfo_(centroidalModelInfo),
        pinocchioMappingPtr_(new CentroidalModelPinocchioMappingTpl<SCALAR_T>(centroidalModelInfo)) {
    pinocchioMappingPtr_->setPinocchioInterface(pinocchioInterface_);
  };

  ~CentroidalMpcRobotModel() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  CentroidalMpcRobotModel* clone() const override { return new CentroidalMpcRobotModel(*this); }

  /******************************************************************************************************/
  /*                                          Start indices                                             */
  /******************************************************************************************************/

  size_t getBaseStartindex() const override { return 6; };
  // 函数说明：获取关节、startindex，连接当前模块的数据流和控制逻辑。
  size_t getJointStartindex() const override { return 12; };
  // 函数说明：获取关节、velocities、startindex，连接当前模块的数据流和控制逻辑。
  size_t getJointVelocitiesStartindex() const override { return 6 * N_CONTACTS; };

  // Assumes contact wrench [f_x, f_y, f_z, M_x, M_y, M_z]^T
  size_t getContactWrenchStartIndices(size_t contactIndex) const override { return 6 * contactIndex; };
  // 函数说明：获取接触、力、start、indices，连接当前模块的数据流和控制逻辑。
  size_t getContactForceStartIndices(size_t contactIndex) const override { return getContactWrenchStartIndices(contactIndex); };
  // 函数说明：获取接触、moment、start、indices，连接当前模块的数据流和控制逻辑。
  size_t getContactMomentStartIndices(size_t contactIndex) const override { return getContactWrenchStartIndices(contactIndex) + 3; };

  /******************************************************************************************************/
  /*                                     Generalized coordinates                                        */
  /******************************************************************************************************/

  VECTOR_T<SCALAR_T> getGeneralizedCoordinates(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    // state 尾部正好是 [base pose, q_j]，可直接作为 Pinocchio 的广义坐标。
    return state.tail(6 + this->modelSettings.mpc_joint_dim);
  };

  // 函数说明：获取浮动基/机身、pose，连接当前模块的数据流和控制逻辑。
  VECTOR6_T<SCALAR_T> getBasePose(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(6, 6);
  };

  // 函数说明：获取浮动基/机身、位置，连接当前模块的数据流和控制逻辑。
  VECTOR3_T<SCALAR_T> getBasePosition(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(6, 3);
  }

  // 函数说明：获取浮动基/机身、姿态、欧拉角、zyx，连接当前模块的数据流和控制逻辑。
  VECTOR3_T<SCALAR_T> getBaseOrientationEulerZYX(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(6 + 3, 3);
  }

  // 返回归一化动量前 3 维。对 centroidal 模型来说它就是质心线速度表达。
  VECTOR3_T<SCALAR_T> getBaseComLinearVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.head(3);
  };

  // 函数说明：获取浮动基/机身、质心、速度，连接当前模块的数据流和控制逻辑。
  VECTOR6_T<SCALAR_T> getBaseComVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.head(6);
  };

  // 函数说明：获取关节、angles，连接当前模块的数据流和控制逻辑。
  VECTOR_T<SCALAR_T> getJointAngles(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.tail(this->modelSettings.mpc_joint_dim);
  };

  // 函数说明：获取关节、velocities，连接当前模块的数据流和控制逻辑。
  VECTOR_T<SCALAR_T> getJointVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) const override {
    assert(input.size() == this->input_dim);
    return input.tail(this->modelSettings.mpc_joint_dim);
  };

  // 函数说明：获取generalized、velocities，连接当前模块的数据流和控制逻辑。
  VECTOR_T<SCALAR_T> getGeneralizedVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) override {
    assert(state.size() == this->state_dim);
    assert(input.size() == this->input_dim);
    // PinocchioMapping 会用当前构型下的 centroidal momentum matrix，把 h 和 qdot_j 还原成 base velocity。
    updateCentroidalDynamics<SCALAR_T>(pinocchioInterface_, centroidalModelInfo_, pinocchioMappingPtr_->getPinocchioJointPosition(state));
    return pinocchioMappingPtr_->getPinocchioJointVelocity(state, input);
  };

  // 函数说明：设置generalized、coordinates，连接当前模块的数据流和控制逻辑。
  void setGeneralizedCoordinates(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& generalizedCorrdinates) const override {
    assert(state.size() == this->state_dim);
    assert(generalizedCorrdinates.size() == 6 + this->modelSettings.mpc_joint_dim);
    state.tail(6 + this->modelSettings.mpc_joint_dim) = generalizedCorrdinates;
  }

  // 函数说明：设置浮动基/机身、pose，连接当前模块的数据流和控制逻辑。
  void setBasePose(VECTOR_T<SCALAR_T>& state, const VECTOR6_T<SCALAR_T>& basePose) const override {
    assert(state.size() == this->state_dim);
    state.segment(6, 6) = basePose;
  }

  // 函数说明：设置浮动基/机身、位置，连接当前模块的数据流和控制逻辑。
  void setBasePosition(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& position) const override {
    assert(state.size() == this->state_dim);
    state.segment(6, 3) = position;
  }

  // 函数说明：设置浮动基/机身、姿态、欧拉角、zyx，连接当前模块的数据流和控制逻辑。
  void setBaseOrientationEulerZYX(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& eulerAnglesZYX) const override {
    assert(state.size() == this->state_dim);
    state.segment(6 + 3, 3) = eulerAnglesZYX;
  }

  // 函数说明：设置关节、angles，连接当前模块的数据流和控制逻辑。
  void setJointAngles(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& jointAngles) const override {
    assert(state.size() == this->state_dim);
    state.tail(this->modelSettings.mpc_joint_dim) = jointAngles;
  }

  // 函数说明：设置关节、velocities，连接当前模块的数据流和控制逻辑。
  void setJointVelocities(VECTOR_T<SCALAR_T>& state, VECTOR_T<SCALAR_T>& input, const VECTOR_T<SCALAR_T>& jointVelocities) const override {
    assert(state.size() == this->state_dim);
    assert(input.size() == this->input_dim);
    input.tail(this->modelSettings.mpc_joint_dim) = jointVelocities;
  }

  // 函数说明：处理adapt、浮动基/机身、pose、height，连接当前模块的数据流和控制逻辑。
  void adaptBasePoseHeight(VECTOR_T<SCALAR_T>& state, scalar_t heightChange) const {
    assert(state.size() == this->state_dim);
    state[6 + 2] += heightChange;
  }

  /******************************************************************************************************/
  /*                                          Contacts                                                  */
  /******************************************************************************************************/

  VECTOR6_T<SCALAR_T> getContactWrench(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    // contactIndex=0/1 分别对应 config 中 contactNames6DoF 的左右脚顺序。
    return input.segment(getContactWrenchStartIndices(contactIndex), CONTACT_WRENCH_DIM);
  };

  // 函数说明：获取接触、力，连接当前模块的数据流和控制逻辑。
  VECTOR3_T<SCALAR_T> getContactForce(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment(getContactWrenchStartIndices(contactIndex), 3);
  };

  // 函数说明：获取接触、moment，连接当前模块的数据流和控制逻辑。
  VECTOR3_T<SCALAR_T> getContactMoment(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment((getContactWrenchStartIndices(contactIndex) + 3), 3);
  };

  // 函数说明：设置接触、力和力矩，连接当前模块的数据流和控制逻辑。
  void setContactWrench(VECTOR_T<SCALAR_T>& input, const VECTOR6_T<SCALAR_T>& wrench, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactWrenchStartIndices(contactIndex), 6) = wrench;
  };

  // 函数说明：设置接触、力，连接当前模块的数据流和控制逻辑。
  void setContactForce(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& force, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactWrenchStartIndices(contactIndex), 3) = force;
  };

  // 函数说明：设置接触、moment，连接当前模块的数据流和控制逻辑。
  void setContactMoment(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& moment, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactWrenchStartIndices(contactIndex) + 3, 3) = moment;
  };

  /******************************************************************************************************/
  /*                                    Custom Centroidal Methods                                       */
  /******************************************************************************************************/

  VECTOR_T<SCALAR_T> getCentroidalMomentum(const VECTOR_T<SCALAR_T>& state) const {
    assert(state.size() == this->state_dim);
    // 注意这里返回的是 normalized momentum，不是未除以质量的物理动量。
    return state.head(6);
  };

  // 函数说明：获取质心动力学、模型、模型信息，连接当前模块的数据流和控制逻辑。
  const CentroidalModelInfoTpl<SCALAR_T>& getCentroidalModelInfo() const { return centroidalModelInfo_; }

 private:
  CentroidalMpcRobotModel(const CentroidalMpcRobotModel& rhs)
      : MpcRobotModelBase<SCALAR_T>(rhs),
        pinocchioInterface_(rhs.pinocchioInterface_),
        centroidalModelInfo_(rhs.centroidalModelInfo_),
        pinocchioMappingPtr_(rhs.pinocchioMappingPtr_->clone()) {
    pinocchioMappingPtr_->setPinocchioInterface(pinocchioInterface_);
  };

  // PinocchioInterface 保存模型和运行时 data；Mapping 负责 OCS2 centroidal state/input 与 Pinocchio q/v 的互转。
  PinocchioInterfaceTpl<SCALAR_T> pinocchioInterface_;  // 变量说明：pinocchioInterface_ 表示Pinocchio、interface。
  const CentroidalModelInfoTpl<SCALAR_T> centroidalModelInfo_;  // 变量说明：centroidalModelInfo_ 表示质心动力学、模型、模型信息。
  const std::unique_ptr<CentroidalModelPinocchioMappingTpl<SCALAR_T>> pinocchioMappingPtr_;  // 变量说明：pinocchioMappingPtr_ 表示Pinocchio、mapping、指针。
};

}  // namespace ocs2::humanoid

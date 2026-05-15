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
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/constraint/FrictionForceConeConstraint.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <ocs2_core/constraint/StateInputConstraint.h>
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"

#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

/**
 * Implements the constraint h(t,x,u) >= 0
 *
 * frictionCoefficient * (Fz + gripperForce) - sqrt(Fx * Fx + Fy * Fy + regularization) >= 0
 *
 * The gripper force shifts the origin of the friction cone down in z-direction by the amount of gripping force available. This makes it
 * possible to produce tangential forces without applying a regular normal force on that foot, or to "pull" on the foot with magnitude up to
 * the gripping force.
 *
 * The regularization prevents the constraint gradient / hessian to go to infinity when Fx = Fz = 0. It also creates a parabolic safety
 * margin to the friction cone. For example: when Fx = Fy = 0 the constraint zero-crossing will be at Fz = 1/frictionCoefficient *
 * sqrt(regularization) instead of Fz = 0
 *
 */
class FrictionForceConeConstraint final : public StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * frictionCoefficient: The coefficient of friction.
   * regularization: A positive number to regulize the friction constraint. refer to the FrictionForceConeConstraint documentation.
   * gripperForce: Gripper force in normal direction.
   * hessianDiagonalShift: The Hessian shift to assure a strictly-convex quadratic constraint approximation.
   */
  struct Config {
    // 函数说明：处理config，连接当前模块的数据流和控制逻辑。
    explicit Config(scalar_t frictionCoefficientParam = 0.7,
                    scalar_t regularizationParam = 25.0,  // 变量说明：regularizationParam 表示regularization、param。
                    scalar_t gripperForceParam = 0.0,  // 变量说明：gripperForceParam 表示gripper、力、param。
                    scalar_t hessianDiagonalShiftParam = 1e-6)  // 变量说明：hessianDiagonalShiftParam 表示hessian、diagonal、shift、param。
        // 函数说明：处理摩擦、coefficient，连接当前模块的数据流和控制逻辑。
        : frictionCoefficient(frictionCoefficientParam),
          regularization(regularizationParam),
          gripperForce(gripperForceParam),
          hessianDiagonalShift(hessianDiagonalShiftParam) {
      assert(frictionCoefficient > 0.0);
      assert(regularization > 0.0);
      assert(hessianDiagonalShift >= 0.0);
    }

    scalar_t frictionCoefficient;  // 变量说明：frictionCoefficient 表示摩擦、coefficient。
    scalar_t regularization;  // 变量说明：regularization 表示约束 Hessian/导数中的正则化系数。
    scalar_t gripperForce;  // 变量说明：gripperForce 表示gripper、力。
    scalar_t hessianDiagonalShift;  // 变量说明：hessianDiagonalShift 表示hessian、diagonal、shift。
  };

  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager.
   * @param [in] config : Friction model settings.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   * @param [in] info : The centroidal model information.
   */
  FrictionForceConeConstraint(const SwitchedModelReferenceManager& referenceManager,
                              Config config,  // 变量说明：config 表示从配置文件读取出的参数集合。
                              size_t contactPointIndex,  // 变量说明：contactPointIndex 表示接触、point、索引。
                              const MpcRobotModelBase<scalar_t>& mpcRobotModel);  // 变量说明：mpcRobotModel 表示MPC、机器人、模型。

  ~FrictionForceConeConstraint() override = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  FrictionForceConeConstraint* clone() const override { return new FrictionForceConeConstraint(*this); }

  // 函数说明：判断主动关节，连接当前模块的数据流和控制逻辑。
  bool isActive(scalar_t time) const override;
  // 函数说明：设置主动关节，连接当前模块的数据流和控制逻辑。
  void setActive(bool active) override { isActive_ = active; }
  // 函数说明：获取主动关节，连接当前模块的数据流和控制逻辑。
  bool getActive() const override { return isActive_; }
  // 函数说明：获取num、constraints，连接当前模块的数据流和控制逻辑。
  size_t getNumConstraints(scalar_t time) const override { return 1; };
  // 函数说明：获取数值，连接当前模块的数据流和控制逻辑。
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  // 函数说明：获取linear、approximation，连接当前模块的数据流和控制逻辑。
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time,
                                                           const vector_t& state,  // 变量说明：state 表示状态。
                                                           const vector_t& input,  // 变量说明：input 表示输入。
                                                           const PreComputation& preComp) const override;  // 变量说明：preComp 表示pre、comp。
  // 函数说明：获取quadratic、approximation，连接当前模块的数据流和控制逻辑。
  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time,
                                                                 const vector_t& state,  // 变量说明：state 表示状态。
                                                                 const vector_t& input,  // 变量说明：input 表示输入。
                                                                 const PreComputation& preComp) const override;  // 变量说明：preComp 表示pre、comp。

  /** Sets the estimated terrain normal expressed in the world frame. */
  void setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld);

 private:
  struct LocalForceDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    matrix3_t dF_du;  // derivative local force w.r.t. forces in world frame
  };

  struct ConeLocalDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t dCone_dF;    // derivative w.r.t local force
    matrix3_t d2Cone_dF2;  // second derivative w.r.t local force
  };

  struct ConeDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t dCone_du;  // 变量说明：dCone_du 表示d、cone、du。
    matrix3_t d2Cone_du2;  // 变量说明：d2Cone_du2 表示d2、cone、du2。
  };

  FrictionForceConeConstraint(const FrictionForceConeConstraint& other);
  // 函数说明：处理cone、约束，连接当前模块的数据流和控制逻辑。
  vector_t coneConstraint(const vector3_t& localForces) const;
  // 函数说明：计算local、力、derivatives，连接当前模块的数据流和控制逻辑。
  LocalForceDerivatives computeLocalForceDerivatives(const vector3_t& forcesInBodyFrame) const;
  // 函数说明：计算cone、local、derivatives，连接当前模块的数据流和控制逻辑。
  ConeLocalDerivatives computeConeLocalDerivatives(const vector3_t& localForces) const;
  // 函数说明：计算cone、约束、derivatives，连接当前模块的数据流和控制逻辑。
  ConeDerivatives computeConeConstraintDerivatives(const ConeLocalDerivatives& coneLocalDerivatives,
                                                   const LocalForceDerivatives& localForceDerivatives) const;  // 变量说明：localForceDerivatives 表示local、力、derivatives。

  // 函数说明：处理摩擦、cone、输入、derivative，连接当前模块的数据流和控制逻辑。
  matrix_t frictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
  // 函数说明：处理摩擦、cone、second、derivative、输入，连接当前模块的数据流和控制逻辑。
  matrix_t frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
  // 函数说明：处理摩擦、cone、second、derivative、状态，连接当前模块的数据流和控制逻辑。
  matrix_t frictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives) const;

  const SwitchedModelReferenceManager* referenceManagerPtr_;  // 变量说明：referenceManagerPtr_ 表示切换模型参考管理器指针。
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。

  const Config config_;  // 变量说明：config_ 表示当前对象持有的约束/代价配置。
  const size_t contactPointIndex_;  // 变量说明：contactPointIndex_ 表示接触、point、索引。

  // rotation world to terrain
  matrix3_t t_R_w = matrix3_t::Identity();  // 变量说明：t_R_w 表示t、r、w。

  bool isActive_ = true;  // 变量说明：isActive_ 表示该足端代价是否启用。
};

}  // namespace ocs2::humanoid

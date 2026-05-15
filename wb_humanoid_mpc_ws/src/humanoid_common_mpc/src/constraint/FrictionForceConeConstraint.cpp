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

// 自动中文注释：src/humanoid_common_mpc/src/constraint/FrictionForceConeConstraint.cpp 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。
#include "humanoid_common_mpc/constraint/FrictionForceConeConstraint.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionForceConeConstraint::FrictionForceConeConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                         Config config,  // 变量说明：config 表示从配置文件读取出的参数集合。
                                                         size_t contactPointIndex,  // 变量说明：contactPointIndex 表示接触、point、索引。
                                                         const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    // 函数说明：处理状态、输入、约束，连接当前模块的数据流和控制逻辑。
    : StateInputConstraint(ConstraintOrder::Quadratic),
      referenceManagerPtr_(&referenceManager),
      config_(std::move(config)),
      mpcRobotModelPtr_(&mpcRobotModel),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

FrictionForceConeConstraint::FrictionForceConeConstraint(const FrictionForceConeConstraint& rhs)
    // 函数说明：处理状态、输入、约束，连接当前模块的数据流和控制逻辑。
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      config_(rhs.config_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_),
      contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrictionForceConeConstraint::setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld) {  // 变量说明：FrictionForceConeConstraint 表示摩擦、力、cone、约束。
  t_R_w.setIdentity();
  throw std::runtime_error("[FrictionForceConeConstraint] setSurfaceNormalInWorld() is not implemented!");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FrictionForceConeConstraint::isActive(scalar_t time) const {  // 变量说明：FrictionForceConeConstraint 表示摩擦、力、cone、约束。
  if (!isActive_) return false;
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FrictionForceConeConstraint::getValue(scalar_t time,  // 变量说明：FrictionForceConeConstraint 表示摩擦、力、cone、约束。
                                               const vector_t& state,  // 变量说明：state 表示状态。
                                               const vector_t& input,  // 变量说明：input 表示输入。
                                               const PreComputation& preComp) const {  // 变量说明：preComp 表示pre、comp。
  const auto forcesInWorldFrame = mpcRobotModelPtr_->getContactForce(input, contactPointIndex_);  // 变量说明：forcesInWorldFrame 表示forces、in、world、坐标系/帧。
  const vector3_t localForce = t_R_w * forcesInWorldFrame;  // 变量说明：localForce 表示local、力。
  return coneConstraint(localForce);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation FrictionForceConeConstraint::getLinearApproximation(scalar_t time,  // 变量说明：FrictionForceConeConstraint 表示摩擦、力、cone、约束。
                                                                                      const vector_t& state,  // 变量说明：state 表示状态。
                                                                                      const vector_t& input,  // 变量说明：input 表示输入。
                                                                                      const PreComputation& preComp) const {  // 变量说明：preComp 表示pre、comp。
  const vector3_t forcesInWorldFrame = mpcRobotModelPtr_->getContactForce(input, contactPointIndex_);  // 变量说明：forcesInWorldFrame 表示forces、in、world、坐标系/帧。
  const vector3_t localForce = t_R_w * forcesInWorldFrame;  // 变量说明：localForce 表示local、力。

  const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame);  // 变量说明：localForceDerivatives 表示local、力、derivatives。
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);  // 变量说明：coneLocalDerivatives 表示cone、local、derivatives。
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);  // 变量说明：coneDerivatives 表示cone、derivatives。

  VectorFunctionLinearApproximation linearApproximation;  // 变量说明：linearApproximation 表示linear、approximation。
  linearApproximation.f = coneConstraint(localForce);
  linearApproximation.dfdx = matrix_t::Zero(1, state.size());
  linearApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation FrictionForceConeConstraint::getQuadraticApproximation(scalar_t time,  // 变量说明：FrictionForceConeConstraint 表示摩擦、力、cone、约束。
                                                                                            const vector_t& state,  // 变量说明：state 表示状态。
                                                                                            const vector_t& input,  // 变量说明：input 表示输入。
                                                                                            const PreComputation& preComp) const {  // 变量说明：preComp 表示pre、comp。
  const vector3_t forcesInWorldFrame = mpcRobotModelPtr_->getContactForce(input, contactPointIndex_);  // 变量说明：forcesInWorldFrame 表示forces、in、world、坐标系/帧。
  const vector3_t localForce = t_R_w * forcesInWorldFrame;  // 变量说明：localForce 表示local、力。

  const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame);  // 变量说明：localForceDerivatives 表示local、力、derivatives。
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);  // 变量说明：coneLocalDerivatives 表示cone、local、derivatives。
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);  // 变量说明：coneDerivatives 表示cone、derivatives。

  VectorFunctionQuadraticApproximation quadraticApproximation;  // 变量说明：quadraticApproximation 表示quadratic、approximation。
  quadraticApproximation.f = coneConstraint(localForce);
  quadraticApproximation.dfdx = matrix_t::Zero(1, state.size());
  quadraticApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
  quadraticApproximation.dfdxx.emplace_back(frictionConeSecondDerivativeState(state.size(), coneDerivatives));
  quadraticApproximation.dfduu.emplace_back(frictionConeSecondDerivativeInput(input.size(), coneDerivatives));
  quadraticApproximation.dfdux.emplace_back(matrix_t::Zero(input.size(), state.size()));
  return quadraticApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionForceConeConstraint::LocalForceDerivatives FrictionForceConeConstraint::computeLocalForceDerivatives(
    const vector3_t& forcesInWorldFrame) const {  // 变量说明：forcesInWorldFrame 表示forces、in、world、坐标系/帧。
  LocalForceDerivatives localForceDerivatives{};  // 变量说明：localForceDerivatives 表示local、力、derivatives。
  localForceDerivatives.dF_du = t_R_w;
  return localForceDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionForceConeConstraint::ConeLocalDerivatives FrictionForceConeConstraint::computeConeLocalDerivatives(
    const vector3_t& localForces) const {  // 变量说明：localForces 表示local、forces。
  const auto F_x_square = localForces.x() * localForces.x();  // 变量说明：F_x_square 表示f、x、square。
  const auto F_y_square = localForces.y() * localForces.y();  // 变量说明：F_y_square 表示f、y、square。
  const auto F_tangent_square = F_x_square + F_y_square + config_.regularization;  // 变量说明：F_tangent_square 表示f、tangent、square。
  const auto F_tangent_norm = sqrt(F_tangent_square);  // 变量说明：F_tangent_norm 表示f、tangent、norm。
  const auto F_tangent_square_pow32 = F_tangent_norm * F_tangent_square;  // = F_tangent_square ^ (3/2)

  ConeLocalDerivatives coneDerivatives{};  // 变量说明：coneDerivatives 表示cone、derivatives。
  coneDerivatives.dCone_dF(0) = -localForces.x() / F_tangent_norm;
  coneDerivatives.dCone_dF(1) = -localForces.y() / F_tangent_norm;
  coneDerivatives.dCone_dF(2) = config_.frictionCoefficient;

  coneDerivatives.d2Cone_dF2(0, 0) = -(F_y_square + config_.regularization) / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(0, 1) = localForces.x() * localForces.y() / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(0, 2) = 0.0;
  coneDerivatives.d2Cone_dF2(1, 0) = coneDerivatives.d2Cone_dF2(0, 1);
  coneDerivatives.d2Cone_dF2(1, 1) = -(F_x_square + config_.regularization) / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(1, 2) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 0) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 1) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 2) = 0.0;

  return coneDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FrictionForceConeConstraint::coneConstraint(const vector3_t& localForces) const {  // 变量说明：FrictionForceConeConstraint 表示摩擦、力、cone、约束。
  const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + config_.regularization;  // 变量说明：F_tangent_square 表示f、tangent、square。
  const auto F_tangent_norm = sqrt(F_tangent_square);  // 变量说明：F_tangent_norm 表示f、tangent、norm。
  const scalar_t coneConstraint = config_.frictionCoefficient * (localForces.z() + config_.gripperForce) - F_tangent_norm;  // 变量说明：coneConstraint 表示cone、约束。
  return (vector_t(1) << coneConstraint).finished();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionForceConeConstraint::ConeDerivatives FrictionForceConeConstraint::computeConeConstraintDerivatives(
    const ConeLocalDerivatives& coneLocalDerivatives, const LocalForceDerivatives& localForceDerivatives) const {  // 变量说明：coneLocalDerivatives 表示cone、local、derivatives。
  ConeDerivatives coneDerivatives;  // 变量说明：coneDerivatives 表示cone、derivatives。
  // First order derivatives
  coneDerivatives.dCone_du.noalias() = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_du;

  // Second order derivatives
  coneDerivatives.d2Cone_du2.noalias() =
      localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;

  return coneDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionForceConeConstraint::frictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const {  // 变量说明：FrictionForceConeConstraint 表示摩擦、力、cone、约束。
  matrix_t dhdu = matrix_t::Zero(1, inputDim);  // 变量说明：dhdu 表示dhdu。
  dhdu.block<1, 3>(0, mpcRobotModelPtr_->getContactForceStartIndices(contactPointIndex_)) = coneDerivatives.dCone_du;
  return dhdu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionForceConeConstraint::frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const {  // 变量说明：FrictionForceConeConstraint 表示摩擦、力、cone、约束。
  matrix_t ddhdudu = matrix_t::Zero(inputDim, inputDim);  // 变量说明：ddhdudu 表示ddhdudu。
  ddhdudu.block<3, 3>(mpcRobotModelPtr_->getContactForceStartIndices(contactPointIndex_),
                      mpcRobotModelPtr_->getContactForceStartIndices(contactPointIndex_)) = coneDerivatives.d2Cone_du2;
  ddhdudu.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdudu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionForceConeConstraint::frictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives) const {  // 变量说明：FrictionForceConeConstraint 表示摩擦、力、cone、约束。
  matrix_t ddhdxdx = matrix_t::Zero(stateDim, stateDim);  // 变量说明：ddhdxdx 表示ddhdxdx。
  ddhdxdx.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdxdx;
}

}  // namespace ocs2::humanoid

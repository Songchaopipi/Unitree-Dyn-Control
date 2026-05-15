/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

namespace ocs2 {
namespace {

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseCentroidalMomentumMatrixInverseLocal(
    const Eigen::Matrix<SCALAR_T, 6, 6>& Ab) {
  const SCALAR_T mass = Ab(0, 0);
  Eigen::Matrix<SCALAR_T, 3, 3> Ab_22_inv = Ab.template block<3, 3>(3, 3).inverse();
  Eigen::Matrix<SCALAR_T, 6, 6> Ab_inv = Eigen::Matrix<SCALAR_T, 6, 6>::Zero();
  Ab_inv << 1.0 / mass * Eigen::Matrix<SCALAR_T, 3, 3>::Identity(), -1.0 / mass * Ab.template block<3, 3>(0, 3) * Ab_22_inv,
      Eigen::Matrix<SCALAR_T, 3, 3>::Zero(), Ab_22_inv;
  return Ab_inv;
}

template <typename SCALAR_T>
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getMappingZyxGradientLocal(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  (void)x;

  Eigen::Matrix<SCALAR_T, 3, 3> dTdz, dTdy, dTdx;
  dTdz << SCALAR_T(0), -cos(z), -cos(y) * sin(z), SCALAR_T(0), -sin(z), cos(y) * cos(z), SCALAR_T(0), SCALAR_T(0),
      SCALAR_T(0);

  dTdy << SCALAR_T(0), SCALAR_T(0), -sin(y) * cos(z), SCALAR_T(0), SCALAR_T(0), -sin(y) * sin(z), SCALAR_T(0), SCALAR_T(0),
      -cos(y);

  dTdx.setZero();
  return {dTdz, dTdy, dTdx};
}

template <typename SCALAR_T>
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getRotationMatrixZyxGradientLocal(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T dc1 = -s1;
  const SCALAR_T dc2 = -s2;
  const SCALAR_T dc3 = -s3;
  const SCALAR_T ds1 = c1;
  const SCALAR_T ds2 = c2;
  const SCALAR_T ds3 = c3;

  Eigen::Matrix<SCALAR_T, 3, 3> dRdz, dRdy, dRdx;
  dRdz << dc1 * c2, dc1 * s2 * s3 - ds1 * c3, dc1 * s2 * c3 + ds1 * s3, ds1 * c2, ds1 * s2 * s3 + dc1 * c3,
      ds1 * s2 * c3 - dc1 * s3, SCALAR_T(0), SCALAR_T(0), SCALAR_T(0);

  dRdy << c1 * dc2, c1 * ds2 * s3, c1 * ds2 * c3, s1 * dc2, s1 * ds2 * s3, s1 * ds2 * c3, -ds2, dc2 * s3, dc2 * c3;

  dRdx << SCALAR_T(0), c1 * s2 * ds3 - s1 * dc3, c1 * s2 * dc3 + s1 * ds3, SCALAR_T(0), s1 * s2 * ds3 + c1 * dc3,
      s1 * s2 * dc3 - c1 * ds3, SCALAR_T(0), c2 * ds3, c2 * dc3;

  return {dRdz, dRdy, dRdx};
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 3> getCentroidalMomentumZyxGradientLocal(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                    const CentroidalModelInfoTpl<SCALAR_T>& info,
                                                                    const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                                                    const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v) {
  using matrix_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, Eigen::Dynamic>;
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3_t = Eigen::Matrix<SCALAR_T, 3, 3>;

  const auto& data = interface.getData();
  const auto m = info.robotMass;
  const vector3_t eulerAngles = q.template segment<3>(3);
  const vector3_t eulerAnglesDerivatives = v.template segment<3>(3);
  const auto T = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAngles);
  const auto R = getRotationMatrixFromZyxEulerAngles(eulerAngles);
  matrix3_t Ibaseframe, Iworldframe;
  vector3_t rbaseframe, rworldframe;

  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      Iworldframe = data.Ig.inertia().matrix();
      Ibaseframe.noalias() = R.transpose() * (Iworldframe * R);
      rworldframe = q.template head<3>() - data.com[0];
      rbaseframe.noalias() = R.transpose() * rworldframe;
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      Ibaseframe = info.centroidalInertiaNominal;
      Iworldframe.noalias() = R * (Ibaseframe * R.transpose());
      rbaseframe = info.comToBasePositionNominal;
      rworldframe.noalias() = R * rbaseframe;
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
    }
  }

  const auto S = skewSymmetricMatrix(rworldframe);
  const auto dT = getMappingZyxGradientLocal(eulerAngles);
  const auto dR = getRotationMatrixZyxGradientLocal(eulerAngles);

  std::array<matrix3_t, 3> dS;
  for (size_t i = 0; i < 3; i++) {
    const vector3_t dr = dR[i] * rbaseframe;
    dS[i] = skewSymmetricMatrix(dr);
  }

  matrix_t dhdq = matrix_t::Zero(6, 3);
  for (size_t i = 0; i < 3; i++) {
    const vector3_t T_eulerAnglesDev = T * eulerAnglesDerivatives;
    const vector3_t dT_eulerAnglesDev = dT[i] * eulerAnglesDerivatives;
    const matrix3_t dR_I_Rtrans = dR[i] * Ibaseframe * R.transpose();

    dhdq.template block<3, 1>(0, i).noalias() = m * (S * dT_eulerAnglesDev);
    dhdq.template block<3, 1>(0, i).noalias() += m * (dS[i] * T_eulerAnglesDev);

    dhdq.template block<3, 1>(3, i).noalias() = (dR_I_Rtrans + dR_I_Rtrans.transpose()) * T_eulerAnglesDev;
    dhdq.template block<3, 1>(3, i).noalias() += Iworldframe * dT_eulerAnglesDev;
  }

  if (info.centroidalModelType == CentroidalModelType::FullCentroidalDynamics) {
    const auto jointVelocities = v.tail(info.actuatedDofNum);
    const vector3_t comLinearVelocityInWorldFrame = (1.0 / m) * (data.Ag.topRightCorner(3, info.actuatedDofNum) * jointVelocities);
    const vector3_t comAngularVelocityInWorldFrame =
        Iworldframe.inverse() * (data.Ag.bottomRightCorner(3, info.actuatedDofNum) * jointVelocities);
    const vector3_t linearMomentumInBaseFrame = m * (R.transpose() * comLinearVelocityInWorldFrame);
    const vector3_t angularMomentumInBaseFrame = Ibaseframe * (R.transpose() * comAngularVelocityInWorldFrame);
    for (size_t i = 0; i < 3; i++) {
      dhdq.template block<3, 1>(0, i).noalias() += dR[i] * linearMomentumInBaseFrame;
      dhdq.template block<3, 1>(3, i).noalias() += dR[i] * angularMomentumInBaseFrame;
    }
  }

  return dhdq;
}

template <typename SCALAR_T>
void updateCentroidalDynamicsLocal(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                                   const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q) {
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3_t = Eigen::Matrix<SCALAR_T, 3, 3>;
  using matrix6_t = Eigen::Matrix<SCALAR_T, 6, 6>;

  const auto& model = interface.getModel();
  auto& data = interface.getData();

  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      pinocchio::computeCentroidalMap(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      const vector3_t eulerAnglesZyx = q.template segment<3>(3);
      const matrix3_t mappingZyx = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAnglesZyx);
      const matrix3_t rotationBaseToWorld = getRotationMatrixFromZyxEulerAngles(eulerAnglesZyx);
      const vector3_t comToBasePositionInWorld = rotationBaseToWorld * info.comToBasePositionNominal;
      const matrix3_t skewSymmetricMap = skewSymmetricMatrix(comToBasePositionInWorld);
      const matrix3_t mat1 = rotationBaseToWorld * info.centroidalInertiaNominal;
      const matrix3_t mat2 = rotationBaseToWorld.transpose() * mappingZyx;
      matrix6_t Ab = matrix6_t::Zero();
      Ab.template topLeftCorner<3, 3>().diagonal().array() = info.robotMass;
      Ab.template topRightCorner<3, 3>().noalias() = info.robotMass * skewSymmetricMap * mappingZyx;
      Ab.template bottomRightCorner<3, 3>().noalias() = mat1 * mat2;
      data.Ag = Eigen::Matrix<SCALAR_T, -1, -1>::Zero(6, info.generalizedCoordinatesNum);
      data.Ag.template leftCols<6>() = Ab;
      data.com[0] = q.template head<3>() - comToBasePositionInWorld;
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
    }
  }
}

template <typename SCALAR_T>
void updateCentroidalDynamicsDerivativesLocal(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                                              const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                              const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v) {
  using matrix6x_t = Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
  const auto& model = interface.getModel();
  auto& data = interface.getData();

  vector_t a = vector_t::Zero(info.generalizedCoordinatesNum);
  matrix6x_t dhdq(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotdq(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotdv(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotda(6, info.generalizedCoordinatesNum);

  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      pinocchio::computeCentroidalDynamicsDerivatives(model, data, q, v, a, dhdq, dhdotdq, dhdotdv, dhdotda);
      data.Ag = dhdotda;
      data.dFdq.setZero(6, info.generalizedCoordinatesNum);
      data.dFdq.template middleCols<3>(3) = getCentroidalMomentumZyxGradientLocal(interface, info, q, v);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      data.dFdq.setZero(6, info.generalizedCoordinatesNum);
      data.dFdq.template middleCols<3>(3) = getCentroidalMomentumZyxGradientLocal(interface, info, q, v);
      pinocchio::computeJointJacobians(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
    }
  }
}

template <typename SCALAR_T>
const Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>& getCentroidalMomentumMatrixLocal(const PinocchioInterfaceTpl<SCALAR_T>& interface) {
  return interface.getData().Ag;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> getPositionComToContactPointInWorldFrameLocal(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                            const CentroidalModelInfoTpl<SCALAR_T>& info,
                                                                            size_t contactIndex) {
  const auto& data = interface.getData();
  return (data.oMf[info.endEffectorFrameIndices[contactIndex]].translation() - data.com[0]);
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrameLocal(
    const PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info, size_t contactIndex) {
  const auto& model = interface.getModel();
  auto data = interface.getData();
  Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic> jacobianWorldToContactPointInWorldFrame;
  jacobianWorldToContactPointInWorldFrame.setZero(6, info.generalizedCoordinatesNum);
  pinocchio::getFrameJacobian(model, data, info.endEffectorFrameIndices[contactIndex], pinocchio::LOCAL_WORLD_ALIGNED,
                              jacobianWorldToContactPointInWorldFrame);
  Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> J_com = getCentroidalMomentumMatrixLocal(interface).template topRows<3>() / info.robotMass;
  return (jacobianWorldToContactPointInWorldFrame.template topRows<3>() - J_com);
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 1> getNormalizedCentroidalMomentumRateLocal(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                       const CentroidalModelInfoTpl<SCALAR_T>& info,
                                                                       const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input) {
  const Eigen::Matrix<SCALAR_T, 3, 1> gravityVector(SCALAR_T(0.0), SCALAR_T(0.0), SCALAR_T(-9.81));
  Eigen::Matrix<SCALAR_T, 6, 1> centroidalMomentumRate;
  centroidalMomentumRate << info.robotMass * gravityVector, Eigen::Matrix<SCALAR_T, 3, 1>::Zero();

  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto contactForceInWorldFrame = centroidal_model::getContactForces(input, i, info);
    const auto positionComToContactPointInWorldFrame = getPositionComToContactPointInWorldFrameLocal(interface, info, i);
    centroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
    centroidalMomentumRate.template tail<3>().noalias() += positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame);
  }

  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const auto contactForceInWorldFrame = centroidal_model::getContactForces(input, i, info);
    const auto contactTorqueInWorldFrame = centroidal_model::getContactTorques(input, i, info);
    const auto positionComToContactPointInWorldFrame = getPositionComToContactPointInWorldFrameLocal(interface, info, i);
    centroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
    centroidalMomentumRate.template tail<3>().noalias() +=
        positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame) + contactTorqueInWorldFrame;
  }

  centroidalMomentumRate /= info.robotMass;
  return centroidalMomentumRate;
}

}  // namespace

template <>
Eigen::Matrix<scalar_t, 6, 6> computeFloatingBaseCentroidalMomentumMatrixInverse<scalar_t>(const Eigen::Matrix<scalar_t, 6, 6>& Ab) {
  return computeFloatingBaseCentroidalMomentumMatrixInverseLocal(Ab);
}

template <>
Eigen::Matrix<ad_scalar_t, 6, 6> computeFloatingBaseCentroidalMomentumMatrixInverse<ad_scalar_t>(
    const Eigen::Matrix<ad_scalar_t, 6, 6>& Ab) {
  return computeFloatingBaseCentroidalMomentumMatrixInverseLocal(Ab);
}

template <>
std::array<Eigen::Matrix<scalar_t, 3, 3>, 3> getMappingZyxGradient<scalar_t>(const Eigen::Matrix<scalar_t, 3, 1>& eulerAngles) {
  return getMappingZyxGradientLocal(eulerAngles);
}

template <>
std::array<Eigen::Matrix<ad_scalar_t, 3, 3>, 3> getMappingZyxGradient<ad_scalar_t>(
    const Eigen::Matrix<ad_scalar_t, 3, 1>& eulerAngles) {
  return getMappingZyxGradientLocal(eulerAngles);
}

template <>
std::array<Eigen::Matrix<scalar_t, 3, 3>, 3> getRotationMatrixZyxGradient<scalar_t>(const Eigen::Matrix<scalar_t, 3, 1>& eulerAngles) {
  return getRotationMatrixZyxGradientLocal(eulerAngles);
}

template <>
std::array<Eigen::Matrix<ad_scalar_t, 3, 3>, 3> getRotationMatrixZyxGradient<ad_scalar_t>(
    const Eigen::Matrix<ad_scalar_t, 3, 1>& eulerAngles) {
  return getRotationMatrixZyxGradientLocal(eulerAngles);
}

template <>
void updateCentroidalDynamics<scalar_t>(PinocchioInterface& interface, const CentroidalModelInfo& info, const vector_t& q) {
  updateCentroidalDynamicsLocal(interface, info, q);
}

template <>
void updateCentroidalDynamics<ad_scalar_t>(PinocchioInterfaceCppAd& interface, const CentroidalModelInfoCppAd& info, const ad_vector_t& q) {
  updateCentroidalDynamicsLocal(interface, info, q);
}

template <>
void updateCentroidalDynamicsDerivatives<scalar_t>(PinocchioInterface& interface, const CentroidalModelInfo& info, const vector_t& q,
                                                   const vector_t& v) {
  updateCentroidalDynamicsDerivativesLocal(interface, info, q, v);
}

template <>
void updateCentroidalDynamicsDerivatives<ad_scalar_t>(PinocchioInterfaceCppAd& interface, const CentroidalModelInfoCppAd& info,
                                                      const ad_vector_t& q, const ad_vector_t& v) {
  updateCentroidalDynamicsDerivativesLocal(interface, info, q, v);
}

template <>
const Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>& getCentroidalMomentumMatrix<scalar_t>(const PinocchioInterface& interface) {
  return getCentroidalMomentumMatrixLocal(interface);
}

template <>
const Eigen::Matrix<ad_scalar_t, 6, Eigen::Dynamic>& getCentroidalMomentumMatrix<ad_scalar_t>(const PinocchioInterfaceCppAd& interface) {
  return getCentroidalMomentumMatrixLocal(interface);
}

template <>
Eigen::Matrix<scalar_t, 6, 3> getCentroidalMomentumZyxGradient<scalar_t>(const PinocchioInterface& interface,
                                                                         const CentroidalModelInfo& info, const vector_t& q,
                                                                         const vector_t& v) {
  return getCentroidalMomentumZyxGradientLocal(interface, info, q, v);
}

template <>
Eigen::Matrix<ad_scalar_t, 6, 3> getCentroidalMomentumZyxGradient<ad_scalar_t>(const PinocchioInterfaceCppAd& interface,
                                                                               const CentroidalModelInfoCppAd& info,
                                                                               const ad_vector_t& q, const ad_vector_t& v) {
  return getCentroidalMomentumZyxGradientLocal(interface, info, q, v);
}

template <>
Eigen::Matrix<scalar_t, 3, 1> getPositionComToContactPointInWorldFrame<scalar_t>(const PinocchioInterface& interface,
                                                                                 const CentroidalModelInfo& info,
                                                                                 size_t contactIndex) {
  return getPositionComToContactPointInWorldFrameLocal(interface, info, contactIndex);
}

template <>
Eigen::Matrix<ad_scalar_t, 3, 1> getPositionComToContactPointInWorldFrame<ad_scalar_t>(const PinocchioInterfaceCppAd& interface,
                                                                                       const CentroidalModelInfoCppAd& info,
                                                                                       size_t contactIndex) {
  return getPositionComToContactPointInWorldFrameLocal(interface, info, contactIndex);
}

template <>
Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame<scalar_t>(
    const PinocchioInterface& interface, const CentroidalModelInfo& info, size_t contactIndex) {
  return getTranslationalJacobianComToContactPointInWorldFrameLocal(interface, info, contactIndex);
}

template <>
Eigen::Matrix<ad_scalar_t, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame<ad_scalar_t>(
    const PinocchioInterfaceCppAd& interface, const CentroidalModelInfoCppAd& info, size_t contactIndex) {
  return getTranslationalJacobianComToContactPointInWorldFrameLocal(interface, info, contactIndex);
}

template <>
Eigen::Matrix<scalar_t, 6, 1> getNormalizedCentroidalMomentumRate<scalar_t>(const PinocchioInterface& interface,
                                                                            const CentroidalModelInfo& info, const vector_t& input) {
  return getNormalizedCentroidalMomentumRateLocal(interface, info, input);
}

template <>
Eigen::Matrix<ad_scalar_t, 6, 1> getNormalizedCentroidalMomentumRate<ad_scalar_t>(const PinocchioInterfaceCppAd& interface,
                                                                                  const CentroidalModelInfoCppAd& info,
                                                                                  const ad_vector_t& input) {
  return getNormalizedCentroidalMomentumRateLocal(interface, info, input);
}

}  // namespace ocs2

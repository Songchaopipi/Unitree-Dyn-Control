#include <pinocchio/fwd.hpp>

#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <iostream>
#include <memory>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ocs2 {

template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>::PinocchioInterfaceTpl(const Model& model, const std::shared_ptr<const ::urdf::ModelInterface> urdfModelPtr) {
  robotModelPtr_ = std::make_shared<const Model>(model);
  robotDataPtr_ = std::make_unique<Data>(*robotModelPtr_);
  if (urdfModelPtr) {
    urdfModelPtr_ = std::make_shared<const ::urdf::ModelInterface>(*urdfModelPtr);
  }
}

template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>::~PinocchioInterfaceTpl() = default;

template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>::PinocchioInterfaceTpl(const PinocchioInterfaceTpl<SCALAR>& rhs)
    : robotModelPtr_(rhs.robotModelPtr_), robotDataPtr_(std::make_unique<Data>(*rhs.robotDataPtr_)), urdfModelPtr_(rhs.urdfModelPtr_) {}

template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>::PinocchioInterfaceTpl(PinocchioInterfaceTpl<SCALAR>&& rhs)
    : robotModelPtr_(std::move(rhs.robotModelPtr_)),
      robotDataPtr_(std::move(rhs.robotDataPtr_)),
      urdfModelPtr_(std::move(rhs.urdfModelPtr_)) {}

template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>& PinocchioInterfaceTpl<SCALAR>::operator=(const PinocchioInterfaceTpl<SCALAR>& rhs) {
  if (this != &rhs) {
    robotModelPtr_ = rhs.robotModelPtr_;
    robotDataPtr_ = std::make_unique<Data>(*rhs.robotDataPtr_);
    urdfModelPtr_ = rhs.urdfModelPtr_;
  }
  return *this;
}

template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>& PinocchioInterfaceTpl<SCALAR>::operator=(PinocchioInterfaceTpl<SCALAR>&& rhs) {
  robotModelPtr_ = std::move(rhs.robotModelPtr_);
  robotDataPtr_ = std::move(rhs.robotDataPtr_);
  urdfModelPtr_ = std::move(rhs.urdfModelPtr_);
  return *this;
}

template <>
template <typename T, PinocchioInterfaceTpl<scalar_t>::EnableIfScalar_t<T>>
PinocchioInterfaceCppAd PinocchioInterfaceTpl<scalar_t>::toCppAd() const {
  return PinocchioInterfaceCppAd(getModel().template cast<ad_scalar_t>(), urdfModelPtr_);
}

std::ostream& operator<<(std::ostream& os, const PinocchioInterface& pinocchioInterface) {
  const auto& model = pinocchioInterface.getModel();
  os << "PinocchioInterface: nq=" << model.nq << ", nv=" << model.nv << ", njoints=" << model.njoints << ", nframes=" << model.nframes;
  return os;
}

template class PinocchioInterfaceTpl<scalar_t>;
template class PinocchioInterfaceTpl<ad_scalar_t>;
template PinocchioInterfaceCppAd PinocchioInterfaceTpl<scalar_t>::toCppAd<scalar_t, true>() const;

}  // namespace ocs2

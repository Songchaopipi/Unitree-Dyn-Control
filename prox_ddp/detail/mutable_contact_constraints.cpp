#include "mutable_contact_constraints.hpp"

#include <stdexcept>

namespace g1_kino_detail
{

MutableBoxBounds::MutableBoxBounds(int dim)
    : lower(Eigen::VectorXd::Zero(dim)),
      upper(Eigen::VectorXd::Zero(dim))
{
}

MutableBoxConstraint::MutableBoxConstraint(std::shared_ptr<MutableBoxBounds> bounds)
    : bounds_(std::move(bounds))
{
    if (!bounds_)
    {
        throw std::invalid_argument("MutableBoxConstraint requires bounds");
    }
}

void MutableBoxConstraint::projection(const ConstVectorRef &z, VectorRef zout) const
{
    zout = z.cwiseMin(bounds_->upper).cwiseMax(bounds_->lower);
}

void MutableBoxConstraint::normalConeProjection(const ConstVectorRef &z, VectorRef zout) const
{
    projection(z, zout);
    zout = z - zout;
}

void MutableBoxConstraint::computeActiveSet(const ConstVectorRef &z, Eigen::Ref<ActiveType> out) const
{
    out.array() =
        (z.array() > bounds_->upper.array()) || (z.array() < bounds_->lower.array());
}

MaskedWrenchConeResidual::MaskedWrenchConeResidual(
    int ndx,
    int nu,
    int foot,
    double mu,
    double footHalfLength,
    double footHalfWidth,
    std::shared_ptr<ContactActivation> activation)
    : Base(ndx, nu, 17),
      residual_(ndx, nu, foot, mu, footHalfLength, footHalfWidth),
      activation_(std::move(activation))
{
    if (!activation_)
    {
        throw std::invalid_argument("MaskedWrenchConeResidual requires activation");
    }
}

void MaskedWrenchConeResidual::evaluate(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const
{
    if (!activation_->active)
    {
        data.value_.setConstant(-1.0);
        return;
    }
    residual_.evaluate(x, u, data);
}

void MaskedWrenchConeResidual::computeJacobians(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const
{
    data.Jx_.setZero();
    data.Ju_.setZero();
    if (activation_->active)
    {
        residual_.computeJacobians(x, u, data);
    }
}

std::shared_ptr<MaskedWrenchConeResidual::Data> MaskedWrenchConeResidual::createData() const
{
    return residual_.createData();
}

MaskedFrameVelocityResidual::MaskedFrameVelocityResidual(
    int ndx,
    int nu,
    const pinocchio::Model &model,
    const pinocchio::Motion &velocity,
    pinocchio::FrameIndex frameId,
    pinocchio::ReferenceFrame type,
    std::shared_ptr<ContactActivation> activation)
    : Base(ndx, nu, 6),
      residual_(ndx, nu, model, velocity, frameId, type),
      activation_(std::move(activation))
{
    if (!activation_)
    {
        throw std::invalid_argument("MaskedFrameVelocityResidual requires activation");
    }
}

void MaskedFrameVelocityResidual::evaluate(const ConstVectorRef &x, const ConstVectorRef &, Data &data) const
{
    data.value_.setZero();
    if (activation_->active)
    {
        residual_.evaluate(x, data);
    }
}

void MaskedFrameVelocityResidual::computeJacobians(const ConstVectorRef &x, const ConstVectorRef &, Data &data) const
{
    data.Jx_.setZero();
    data.Ju_.setZero();
    if (activation_->active)
    {
        residual_.computeJacobians(x, data);
    }
}

std::shared_ptr<MaskedFrameVelocityResidual::Data> MaskedFrameVelocityResidual::createData() const
{
    return residual_.createData();
}

} // namespace g1_kino_detail

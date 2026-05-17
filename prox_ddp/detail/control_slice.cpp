#include "control_slice.hpp"

namespace g1_kino_detail
{

ControlSliceResidual::ControlSliceResidual(int ndx, int nu, int offset, int size)
    : Base(ndx, nu, size), offset_(offset), size_(size)
{
}

void ControlSliceResidual::evaluate(const ConstVectorRef &, const ConstVectorRef &u, Data &data) const
{
    data.value_ = u.segment(offset_, size_);
}

void ControlSliceResidual::computeJacobians(const ConstVectorRef &, const ConstVectorRef &, Data &data) const
{
    data.Jx_.setZero();
    data.Ju_.setZero();
    data.Ju_.block(0, offset_, size_, size_).setIdentity();
}

} // namespace g1_kino_detail

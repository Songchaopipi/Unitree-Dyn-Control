#include "semi_implicit_kinodynamics.hpp"

#include <stdexcept>

namespace g1_kino_detail
{

// 数据说明：离散动力学计算需要的连续动力学 data 和矩阵临时量。
struct DirectSemiImplicitKinodynamicsData final : ExplicitDynamicsData
{
    // 函数说明：根据模型维度分配连续 data、dx 和 Jacobian 临时缓冲。
    explicit DirectSemiImplicitKinodynamicsData(const DirectSemiImplicitKinodynamics &model)
        : ExplicitDynamicsData(model),
          continuousData(model.continuousDynamics().createData()),
          dx(model.ndx1()),
          JtmpXnext2(model.ndx2(), model.ndx1()),
          JtmpU(model.ndx2(), model.nu)
    {
        dx.setZero();
        JtmpXnext2.setZero();
        JtmpU.setZero();
    }

    std::shared_ptr<ContinuousDynamicsData> continuousData;
    Eigen::VectorXd dx;
    Eigen::MatrixXd JtmpXnext2;
    Eigen::MatrixXd JtmpU;
};

DirectSemiImplicitKinodynamics::DirectSemiImplicitKinodynamics(
    const Space &space,
    const pinocchio::Model &model,
    const Eigen::Vector3d &gravity,
    const std::vector<bool> &contactStates,
    const std::vector<pinocchio::FrameIndex> &contactIds,
    int forceSize,
    double dt)
    : Base(space, model.nv - 6 + static_cast<int>(contactStates.size()) * forceSize),
      ode_(space, model, gravity, contactStates, contactIds, forceSize),
      timestep_(dt)
{
    if (space.ndx() % 2 != 0)
    {
        throw std::invalid_argument("DirectSemiImplicitKinodynamics requires even tangent dimension");
    }
    if (!(timestep_ > 0.0))
    {
        throw std::invalid_argument("DirectSemiImplicitKinodynamics requires positive dt");
    }
}

void DirectSemiImplicitKinodynamics::computeForwardState(
    const ConstVectorRef &x,
    const ConstVectorRef &u,
    DirectSemiImplicitKinodynamicsData &data,
    ContinuousDynamicsData &continuousData) const
{
    ode_.forward(x, u, continuousData);

    const int ndx = ndx1();
    const int half = ndx / 2;
    data.dx.setZero();
    data.dx.bottomRows(half) = continuousData.xdot_.bottomRows(half) * timestep_;
    space_next().integrate(x, data.dx, data.xnext_);
    data.dx.topRows(half) = data.xnext_.bottomRows(half) * timestep_;
    space_next().integrate(x, data.dx, data.xnext_);
}

void DirectSemiImplicitKinodynamics::forward(
    const ConstVectorRef &x,
    const ConstVectorRef &u,
    Data &data) const
{
    auto &directData = static_cast<DirectSemiImplicitKinodynamicsData &>(data);
    computeForwardState(x, u, directData, *directData.continuousData);
}

void DirectSemiImplicitKinodynamics::dForward(
    const ConstVectorRef &x,
    const ConstVectorRef &u,
    Data &data) const
{
    auto &directData = static_cast<DirectSemiImplicitKinodynamicsData &>(data);
    ContinuousDynamicsData &continuousData = *directData.continuousData;
    ode_.dForward(x, u, continuousData);

    const int ndx = ndx1();
    const int half = ndx / 2;
    const auto &space = space_next();

    directData.Jx() = timestep_ * continuousData.Jx();
    directData.Ju() = timestep_ * continuousData.Ju();
    space.JintegrateTransport(x, directData.dx, directData.Jx(), 1);
    space.JintegrateTransport(x, directData.dx, directData.Ju(), 1);
    space.Jintegrate(x, directData.dx, directData.Jtmp_xnext, 0);
    directData.Jx() += directData.Jtmp_xnext;

    directData.JtmpXnext2.setZero();
    directData.JtmpU.setZero();
    directData.JtmpXnext2.topRows(half) = timestep_ * directData.Jx().bottomRows(half);
    directData.JtmpXnext2.bottomRows(half) = timestep_ * continuousData.Jx().bottomRows(half);
    directData.JtmpU.topRows(half) = timestep_ * directData.Ju().bottomRows(half);
    directData.JtmpU.bottomRows(half) = timestep_ * continuousData.Ju().bottomRows(half);

    space.JintegrateTransport(x, directData.dx, directData.JtmpXnext2, 1);
    space.JintegrateTransport(x, directData.dx, directData.JtmpU, 1);
    directData.JtmpXnext2 += directData.Jtmp_xnext;
    directData.Jx().topRows(half) = directData.JtmpXnext2.topRows(half);
    directData.Ju().topRows(half) = directData.JtmpU.topRows(half);
}

std::shared_ptr<ExplicitDynamicsData> DirectSemiImplicitKinodynamics::createData() const
{
    return std::make_shared<DirectSemiImplicitKinodynamicsData>(*this);
}

} // namespace g1_kino_detail

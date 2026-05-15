#include "g1_kinodynamics_nmpc.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sstream>

#include <aligator/core/cost-abstract.hpp>
#include <aligator/core/explicit-dynamics.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/centroidal/centroidal-wrench-cone.hpp>
#include <aligator/modelling/constraints/box-constraint.hpp>
#include <aligator/modelling/constraints/equality-constraint.hpp>
#include <aligator/modelling/constraints/negative-orthant.hpp>
#include <aligator/modelling/dynamics/kinodynamics-fwd.hpp>
#include <aligator/modelling/function-xpr-slice.hpp>
#include <aligator/modelling/multibody/centroidal-momentum.hpp>
#include <aligator/modelling/multibody/centroidal-momentum-derivative.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <aligator/modelling/multibody/frame-velocity.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>

namespace
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Space = aligator::MultibodyPhaseSpace<double>;
using StageModel = aligator::StageModelTpl<double>;
using TrajOptProblem = aligator::TrajOptProblemTpl<double>;
using SolverProxDDP = aligator::SolverProxDDPTpl<double>;
using KinodynamicsFwdDynamics = aligator::dynamics::KinodynamicsFwdDynamicsTpl<double>;
using ContinuousDynamicsData = aligator::dynamics::ContinuousDynamicsDataTpl<double>;
using CostAbstract = aligator::CostAbstractTpl<double>;
using CostDataAbstract = aligator::CostDataAbstractTpl<double>;
using ExplicitDynamicsModel = aligator::ExplicitDynamicsModelTpl<double>;
using ExplicitDynamicsData = aligator::ExplicitDynamicsDataTpl<double>;
using StageFunctionData = aligator::StageFunctionDataTpl<double>;
using CentroidalMomentumResidual = aligator::CentroidalMomentumResidualTpl<double>;
using CentroidalMomentumDerivativeResidual = aligator::CentroidalMomentumDerivativeResidualTpl<double>;
using FramePlacementResidual = aligator::FramePlacementResidualTpl<double>;

class ControlSliceResidual final : public aligator::StageFunctionTpl<double>
{
public:
    using Base = aligator::StageFunctionTpl<double>;
    using Data = aligator::StageFunctionDataTpl<double>;

    ControlSliceResidual(int ndx, int nu, int offset, int size)
        : Base(ndx, nu, size), offset_(offset), size_(size)
    {
    }

    void evaluate(const ConstVectorRef &, const ConstVectorRef &u, Data &data) const override
    {
        data.value_ = u.segment(offset_, size_);
    }

    void computeJacobians(const ConstVectorRef &, const ConstVectorRef &, Data &data) const override
    {
        data.Jx_.setZero();
        data.Ju_.setZero();
        data.Ju_.block(0, offset_, size_, size_).setIdentity();
    }

private:
    int offset_{0};
    int size_{0};
};

struct DirectSemiImplicitKinodynamicsData;

class DirectSemiImplicitKinodynamics final : public ExplicitDynamicsModel
{
public:
    using Base = ExplicitDynamicsModel;
    using Data = ExplicitDynamicsData;

    DirectSemiImplicitKinodynamics(const Space &space,
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

    void forward(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override;
    void dForward(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override;
    std::shared_ptr<Data> createData() const override;

    const KinodynamicsFwdDynamics &continuousDynamics() const { return ode_; }

private:
    void computeForwardState(const ConstVectorRef &x,
                             const ConstVectorRef &u,
                             DirectSemiImplicitKinodynamicsData &data,
                             ContinuousDynamicsData &continuousData) const;

    KinodynamicsFwdDynamics ode_;
    double timestep_{0.0};
};

struct DirectSemiImplicitKinodynamicsData final : ExplicitDynamicsData
{
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

struct DirectKinoCostReferences
{
    Eigen::VectorXd xRef;
    Eigen::VectorXd uRef;
    std::array<pinocchio::SE3, G1KinodynamicsNmpc::kNumFeet> footPoseReference{
        pinocchio::SE3::Identity(), pinocchio::SE3::Identity()};
    Eigen::MatrixXd stateWeight;
    Eigen::MatrixXd controlWeight;
    Eigen::MatrixXd terminalWeight;
    Eigen::Matrix<double, 6, 6> centroidalWeight = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> centroidalDerivativeWeight = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> footPoseWeight = Eigen::Matrix<double, 6, 6>::Zero();
};

void resetCostData(CostDataAbstract &data)
{
    data.value_ = 0.0;
    data.grad_.setZero();
    data.hess_.setZero();
}

void resetResidualData(StageFunctionData &data)
{
    data.value_.setZero();
    data.jac_buffer_.setZero();
    data.vhp_buffer_.setZero();
}

void addQuadraticResidual(CostDataAbstract &data,
                          const Eigen::Ref<const Eigen::VectorXd> &residual,
                          const Eigen::Ref<const Eigen::MatrixXd> &Jx,
                          const Eigen::Ref<const Eigen::MatrixXd> &Ju,
                          const Eigen::Ref<const Eigen::MatrixXd> &weight)
{
    const Eigen::VectorXd weightedResidual = weight * residual;
    data.value_ += 0.5 * residual.dot(weightedResidual);
    data.Lx_.noalias() += Jx.transpose() * weightedResidual;
    data.Lu_.noalias() += Ju.transpose() * weightedResidual;

    const Eigen::MatrixXd weightedJx = weight * Jx;
    const Eigen::MatrixXd weightedJu = weight * Ju;
    data.Lxx_.noalias() += Jx.transpose() * weightedJx;
    data.Lxu_.noalias() += Jx.transpose() * weightedJu;
    data.Lux_.noalias() += Ju.transpose() * weightedJx;
    data.Luu_.noalias() += Ju.transpose() * weightedJu;
}

void addStateResidual(CostDataAbstract &data,
                      const Eigen::Ref<const Eigen::VectorXd> &residual,
                      const Eigen::Ref<const Eigen::MatrixXd> &Jx,
                      const Eigen::Ref<const Eigen::MatrixXd> &weight)
{
    const Eigen::VectorXd weightedResidual = weight * residual;
    data.value_ += 0.5 * residual.dot(weightedResidual);
    data.Lx_.noalias() += Jx.transpose() * weightedResidual;
    const Eigen::MatrixXd weightedJx = weight * Jx;
    data.Lxx_.noalias() += Jx.transpose() * weightedJx;
}

void addControlResidual(CostDataAbstract &data,
                        const Eigen::Ref<const Eigen::VectorXd> &residual,
                        const Eigen::Ref<const Eigen::MatrixXd> &weight)
{
    const Eigen::VectorXd weightedResidual = weight * residual;
    data.value_ += 0.5 * residual.dot(weightedResidual);
    data.Lu_.noalias() += weightedResidual;
    data.Luu_.noalias() += weight;
}

struct DirectKinoRunningCostData final : CostDataAbstract
{
    DirectKinoRunningCostData(int ndx,
                              int nu,
                              const CentroidalMomentumResidual &centroidalResidual,
                              const CentroidalMomentumDerivativeResidual &centroidalDerivativeResidual,
                              const std::array<FramePlacementResidual, G1KinodynamicsNmpc::kNumFeet> &footPoseResiduals)
        : CostDataAbstract(ndx, nu),
          stateResidual(ndx),
          stateJacobian(ndx, ndx),
          controlResidual(nu),
          centroidalData(centroidalResidual.createData()),
          centroidalDerivativeData(centroidalDerivativeResidual.createData())
    {
        stateResidual.setZero();
        stateJacobian.setZero();
        controlResidual.setZero();
        for (int foot = 0; foot < G1KinodynamicsNmpc::kNumFeet; ++foot)
        {
            footPoseData[foot] = footPoseResiduals[foot].createData();
        }
    }

    Eigen::VectorXd stateResidual;
    Eigen::MatrixXd stateJacobian;
    Eigen::VectorXd controlResidual;
    std::shared_ptr<StageFunctionData> centroidalData;
    std::shared_ptr<StageFunctionData> centroidalDerivativeData;
    std::array<std::shared_ptr<StageFunctionData>, G1KinodynamicsNmpc::kNumFeet> footPoseData;
};

class DirectKinoRunningCost final : public CostAbstract
{
public:
    DirectKinoRunningCost(const Space &space,
                          int nu,
                          std::shared_ptr<DirectKinoCostReferences> references,
                          const pinocchio::Model &model,
                          const Eigen::Vector3d &gravity,
                          const std::vector<bool> &contactStates,
                          const std::vector<pinocchio::FrameIndex> &contactIds,
                          const std::array<pinocchio::FrameIndex, G1KinodynamicsNmpc::kNumFeet> &footFrameIds)
        : CostAbstract(space, nu),
          references_(std::move(references)),
          centroidalResidual_(space.ndx(), nu, model, Vector6d::Zero()),
          centroidalDerivativeResidual_(space.ndx(), model, gravity, contactStates, contactIds, G1KinodynamicsNmpc::kForceSize),
          footPoseResiduals_{{
              FramePlacementResidual(space.ndx(), nu, model, pinocchio::SE3::Identity(), footFrameIds[0]),
              FramePlacementResidual(space.ndx(), nu, model, pinocchio::SE3::Identity(), footFrameIds[1]),
          }}
    {
        if (!references_)
        {
            throw std::invalid_argument("DirectKinoRunningCost requires references");
        }
    }

    void evaluate(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const override
    {
        evaluatePack(x, u, data);
    }

    void computeGradients(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const override
    {
        evaluatePack(x, u, data);
    }

    void computeHessians(const ConstVectorRef &, const ConstVectorRef &, CostData &) const override {}

    std::shared_ptr<CostData> createData() const override
    {
        return std::make_shared<DirectKinoRunningCostData>(
            ndx(), nu, centroidalResidual_, centroidalDerivativeResidual_, footPoseResiduals_);
    }

private:
    void evaluatePack(const ConstVectorRef &x, const ConstVectorRef &u, CostData &rawData) const
    {
        auto &data = static_cast<DirectKinoRunningCostData &>(rawData);
        resetCostData(data);

        this->space->difference(references_->xRef, x, data.stateResidual);
        this->space->Jdifference(references_->xRef, x, data.stateJacobian, 1);
        addStateResidual(data, data.stateResidual, data.stateJacobian, references_->stateWeight);

        data.controlResidual = u - references_->uRef;
        addControlResidual(data, data.controlResidual, references_->controlWeight);

        resetResidualData(*data.centroidalData);
        centroidalResidual_.evaluate(x, *data.centroidalData);
        centroidalResidual_.computeJacobians(x, *data.centroidalData);
        addQuadraticResidual(data,
                             data.centroidalData->value_,
                             data.centroidalData->Jx_,
                             data.centroidalData->Ju_,
                             references_->centroidalWeight);

        resetResidualData(*data.centroidalDerivativeData);
        centroidalDerivativeResidual_.evaluate(x, u, *data.centroidalDerivativeData);
        centroidalDerivativeResidual_.computeJacobians(x, u, *data.centroidalDerivativeData);
        addQuadraticResidual(data,
                             data.centroidalDerivativeData->value_,
                             data.centroidalDerivativeData->Jx_,
                             data.centroidalDerivativeData->Ju_,
                             references_->centroidalDerivativeWeight);

        for (int foot = 0; foot < G1KinodynamicsNmpc::kNumFeet; ++foot)
        {
            footPoseResiduals_[foot].setReference(references_->footPoseReference[foot]);
            resetResidualData(*data.footPoseData[foot]);
            footPoseResiduals_[foot].evaluate(x, *data.footPoseData[foot]);
            footPoseResiduals_[foot].computeJacobians(x, *data.footPoseData[foot]);
            addQuadraticResidual(data,
                                 data.footPoseData[foot]->value_,
                                 data.footPoseData[foot]->Jx_,
                                 data.footPoseData[foot]->Ju_,
                                 references_->footPoseWeight);
        }
    }

    std::shared_ptr<DirectKinoCostReferences> references_;
    CentroidalMomentumResidual centroidalResidual_;
    CentroidalMomentumDerivativeResidual centroidalDerivativeResidual_;
    mutable std::array<FramePlacementResidual, G1KinodynamicsNmpc::kNumFeet> footPoseResiduals_;
};

struct DirectKinoTerminalCostData final : CostDataAbstract
{
    DirectKinoTerminalCostData(int ndx,
                               int nu,
                               const CentroidalMomentumResidual &centroidalResidual)
        : CostDataAbstract(ndx, nu),
          stateResidual(ndx),
          stateJacobian(ndx, ndx),
          centroidalData(centroidalResidual.createData())
    {
        stateResidual.setZero();
        stateJacobian.setZero();
    }

    Eigen::VectorXd stateResidual;
    Eigen::MatrixXd stateJacobian;
    std::shared_ptr<StageFunctionData> centroidalData;
};

class DirectKinoTerminalCost final : public CostAbstract
{
public:
    DirectKinoTerminalCost(const Space &space,
                           int nu,
                           std::shared_ptr<DirectKinoCostReferences> references,
                           const pinocchio::Model &model)
        : CostAbstract(space, nu),
          references_(std::move(references)),
          centroidalResidual_(space.ndx(), nu, model, Vector6d::Zero())
    {
        if (!references_)
        {
            throw std::invalid_argument("DirectKinoTerminalCost requires references");
        }
    }

    void evaluate(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const override
    {
        evaluatePack(x, u, data);
    }

    void computeGradients(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const override
    {
        evaluatePack(x, u, data);
    }

    void computeHessians(const ConstVectorRef &, const ConstVectorRef &, CostData &) const override {}

    std::shared_ptr<CostData> createData() const override
    {
        return std::make_shared<DirectKinoTerminalCostData>(ndx(), nu, centroidalResidual_);
    }

private:
    void evaluatePack(const ConstVectorRef &x, const ConstVectorRef &u, CostData &rawData) const
    {
        auto &data = static_cast<DirectKinoTerminalCostData &>(rawData);
        resetCostData(data);

        this->space->difference(references_->xRef, x, data.stateResidual);
        this->space->Jdifference(references_->xRef, x, data.stateJacobian, 1);
        addStateResidual(data, data.stateResidual, data.stateJacobian, references_->terminalWeight);

        resetResidualData(*data.centroidalData);
        centroidalResidual_.evaluate(x, *data.centroidalData);
        centroidalResidual_.computeJacobians(x, *data.centroidalData);
        addQuadraticResidual(data,
                             data.centroidalData->value_,
                             data.centroidalData->Jx_,
                             data.centroidalData->Ju_,
                             references_->centroidalWeight * 5.0);
    }

    std::shared_ptr<DirectKinoCostReferences> references_;
    CentroidalMomentumResidual centroidalResidual_;
};

Eigen::VectorXd safeVectorOrZero(const Eigen::VectorXd &value, int size)
{
    if (value.size() == size && value.allFinite())
    {
        return value;
    }
    return Eigen::VectorXd::Zero(size);
}

Eigen::VectorXd makeStateWeightDiagonal(int nv)
{
    Eigen::VectorXd w = Eigen::VectorXd::Zero(2 * nv);
    if (nv < 6)
    {
        return w;
    }

    w.segment<3>(0) << 50.0, 50.0, 150.0;
    w.segment<3>(3) << 250.0, 250.0, 120.0;
    const int actuated = nv - 6;
    for (int i = 0; i < actuated; ++i)
    {
        const int id = 6 + i;
        if (i < 12)
        {
            w(id) = 1.0;
        }
        else if (i < 15)
        {
            w(id) = 20.0;
        }
        else
        {
            w(id) = 2.0;
        }
    }

    w.segment<3>(nv) << 4.0, 4.0, 8.0;
    w.segment<3>(nv + 3) << 20.0, 20.0, 8.0;
    for (int i = 0; i < actuated; ++i)
    {
        const int id = nv + 6 + i;
        w(id) = (i < 12) ? 0.2 : 1.0;
    }
    return w;
}

Eigen::MatrixXd makeControlWeight(int nu, int nv)
{
    Eigen::MatrixXd w = Eigen::MatrixXd::Zero(nu, nu);
    for (int foot = 0; foot < G1KinodynamicsNmpc::kNumFeet; ++foot)
    {
        const int offset = foot * G1KinodynamicsNmpc::kForceSize;
        w.diagonal().segment<6>(offset) << 2e-4, 2e-4, 2e-5, 2e-2, 2e-2, 2e-2;
    }
    const int accOffset = G1KinodynamicsNmpc::kNumFeet * G1KinodynamicsNmpc::kForceSize;
    if (nu > accOffset)
    {
        w.diagonal().segment(accOffset, nu - accOffset).setConstant(5e-1);
        const int legCount = std::min(12, nv - 6);
        if (legCount > 0)
        {
            w.diagonal().segment(accOffset, legCount).setConstant(2e-1);
        }
    }
    return w;
}
} // namespace

struct G1KinodynamicsNmpc::SolverCache
{
    struct Signature
    {
        int horizon{0};
        double dt{0.0};
        int nq{0};
        int nv{0};
        int nu{0};
        std::array<bool, kNumFeet> contactActive{{false, false}};
        bool useForceCone{false};
        bool constrainStandingFeet{false};
        double mu{0.0};
        double footHalfLength{0.0};
        double footHalfWidth{0.0};
        double forceLimit{0.0};
        double momentLimit{0.0};
        double jointAccelerationLimit{0.0};
        double tolerance{0.0};
        double muInit{0.0};
        int maxIterations{0};
        int maxAlIterations{0};

        bool operator==(const Signature &rhs) const
        {
            return horizon == rhs.horizon &&
                   dt == rhs.dt &&
                   nq == rhs.nq &&
                   nv == rhs.nv &&
                   nu == rhs.nu &&
                   contactActive == rhs.contactActive &&
                   useForceCone == rhs.useForceCone &&
                   constrainStandingFeet == rhs.constrainStandingFeet &&
                   mu == rhs.mu &&
                   footHalfLength == rhs.footHalfLength &&
                   footHalfWidth == rhs.footHalfWidth &&
                   forceLimit == rhs.forceLimit &&
                   momentLimit == rhs.momentLimit &&
                   jointAccelerationLimit == rhs.jointAccelerationLimit &&
                   tolerance == rhs.tolerance &&
                   muInit == rhs.muInit &&
                   maxIterations == rhs.maxIterations &&
                   maxAlIterations == rhs.maxAlIterations;
        }

        bool operator!=(const Signature &rhs) const { return !(*this == rhs); }
    };

    Signature signature;
    std::unique_ptr<TrajOptProblem> problem;
    std::unique_ptr<SolverProxDDP> solver;
    std::shared_ptr<DirectKinoCostReferences> references;
};

G1KinodynamicsNmpc::G1KinodynamicsNmpc(
    const pinocchio::Model &model,
    const std::array<pinocchio::FrameIndex, kNumFeet> &footFrameIds,
    int horizon,
    double dt)
    : model_(model),
      footFrameIds_(footFrameIds),
      horizon_(std::max(1, horizon)),
      dt_(std::max(1e-4, dt)),
      nu_(model.nv - 6 + kNumFeet * kForceSize)
{
    firstJointAccelerations_ = Eigen::VectorXd::Zero(std::max(0, model_.nv - 6));
    firstState_ = Eigen::VectorXd::Zero(model_.nq + model_.nv);
    firstControl_ = Eigen::VectorXd::Zero(nu_);
}

G1KinodynamicsNmpc::~G1KinodynamicsNmpc() = default;

Eigen::VectorXd G1KinodynamicsNmpc::makeState(const Input &input) const
{
    Eigen::VectorXd x(model_.nq + model_.nv);
    x.head(model_.nq) = safeVectorOrZero(input.q, model_.nq);
    x.tail(model_.nv) = safeVectorOrZero(input.v, model_.nv);
    return x;
}

Eigen::VectorXd G1KinodynamicsNmpc::makeStateReference(const Input &input) const
{
    Eigen::VectorXd xref(model_.nq + model_.nv);
    xref.head(model_.nq) = safeVectorOrZero(input.qReference, model_.nq);
    xref.tail(model_.nv) = safeVectorOrZero(input.vReference, model_.nv);
    return xref;
}

Eigen::VectorXd G1KinodynamicsNmpc::nominalControl(const Input &input) const
{
    Eigen::VectorXd u = Eigen::VectorXd::Zero(nu_);
    for (int foot = 0; foot < kNumFeet; ++foot)
    {
        if (input.contactActive[foot])
        {
            u.segment<kForceSize>(foot * kForceSize) = input.wrenchReference[foot];
        }
    }
    return u;
}

void G1KinodynamicsNmpc::initializeWarmStart(const Input &input)
{
    const Eigen::VectorXd x0 = makeState(input);
    const Eigen::VectorXd u0 = nominalControl(input);
    xsWarm_.assign(static_cast<std::size_t>(horizon_ + 1), x0);
    usWarm_.assign(static_cast<std::size_t>(horizon_), u0);
}

void G1KinodynamicsNmpc::shiftWarmStart(const Input &input)
{
    if (static_cast<int>(xsWarm_.size()) != horizon_ + 1 ||
        static_cast<int>(usWarm_.size()) != horizon_)
    {
        initializeWarmStart(input);
        return;
    }

    for (int k = 0; k < horizon_; ++k)
    {
        xsWarm_[k] = xsWarm_[std::min(k + 1, horizon_)];
    }
    xsWarm_.back() = xsWarm_[horizon_ - 1];
    xsWarm_.front() = makeState(input);

    for (int k = 0; k + 1 < horizon_; ++k)
    {
        usWarm_[k] = usWarm_[k + 1];
    }
    usWarm_.back() = nominalControl(input);

    const Eigen::VectorXd uNom = nominalControl(input);
    for (int k = 0; k < horizon_; ++k)
    {
        for (int foot = 0; foot < kNumFeet; ++foot)
        {
            const int offset = foot * kForceSize;
            if (!input.contactActive[foot])
            {
                usWarm_[k].segment<kForceSize>(offset).setZero();
            }
            else if (usWarm_[k].segment<kForceSize>(offset).norm() < 1e-9)
            {
                usWarm_[k].segment<kForceSize>(offset) = uNom.segment<kForceSize>(offset);
            }
        }
    }
}

bool G1KinodynamicsNmpc::collectSolverCacheReferences()
{
    return solverCache_ && solverCache_->problem && solverCache_->references;
}

void G1KinodynamicsNmpc::updateCachedProblemReferences(const Input &input,
                                                       const Eigen::VectorXd &x0,
                                                       const Eigen::VectorXd &xRef,
                                                       const Eigen::VectorXd &uRef)
{
    if (!solverCache_ || !solverCache_->problem)
    {
        return;
    }

    solverCache_->problem->setInitState(x0);

    solverCache_->references->xRef = xRef;
    solverCache_->references->uRef = uRef;
    for (int foot = 0; foot < kNumFeet; ++foot)
    {
        solverCache_->references->footPoseReference[foot] = input.footPoseReference[foot];
    }
}

bool G1KinodynamicsNmpc::ensureSolverCache(const Input &input,
                                           const Eigen::VectorXd &x0,
                                           const Eigen::VectorXd &xRef,
                                           const Eigen::VectorXd &uRef,
                                           double &solverSetupTime,
                                           bool &rebuilt)
{
    solverSetupTime = 0.0;
    rebuilt = false;

    SolverCache::Signature signature;
    signature.horizon = horizon_;
    signature.dt = dt_;
    signature.nq = model_.nq;
    signature.nv = model_.nv;
    signature.nu = nu_;
    signature.contactActive = input.contactActive;
    signature.useForceCone = useForceCone;
    signature.constrainStandingFeet = constrainStandingFeet;
    signature.mu = mu;
    signature.footHalfLength = footHalfLength;
    signature.footHalfWidth = footHalfWidth;
    signature.forceLimit = forceLimit;
    signature.momentLimit = momentLimit;
    signature.jointAccelerationLimit = jointAccelerationLimit;
    signature.tolerance = tolerance;
    signature.muInit = muInit;
    signature.maxIterations = maxIterations;
    signature.maxAlIterations = maxAlIterations;

    if (solverCache_ && solverCache_->problem && solverCache_->solver &&
        solverCache_->signature == signature)
    {
        updateCachedProblemReferences(input, x0, xRef, uRef);
        return true;
    }

    auto cache = std::make_unique<SolverCache>();
    cache->signature = signature;

    const Space space(model_);
    Eigen::MatrixXd wState = Eigen::MatrixXd::Zero(space.ndx(), space.ndx());
    wState.diagonal() = makeStateWeightDiagonal(model_.nv);
    const Eigen::MatrixXd wControl = makeControlWeight(nu_, model_.nv);
    // Keep the same intent as g1_cd_nmpc_ik_walk: regulate linear and angular
    // centroidal momentum, instead of leaving the linear momentum channel free.
    Eigen::Matrix<double, 6, 6> wCent = Eigen::Matrix<double, 6, 6>::Zero();
    wCent.diagonal() << 0.5, 0.5, 1.0, 5.0, 5.0, 2.0;
    Eigen::Matrix<double, 6, 6> wCentDer = Eigen::Matrix<double, 6, 6>::Zero();
    wCentDer.diagonal() << 0.05, 0.05, 0.1, 0.5, 0.5, 0.2;
    Eigen::Matrix<double, 6, 6> wFrame = Eigen::Matrix<double, 6, 6>::Identity();
    wFrame.diagonal() << 4000.0, 4000.0, 7000.0, 800.0, 800.0, 400.0;
    Eigen::MatrixXd wTerminal = wState * 5.0;

    auto references = std::make_shared<DirectKinoCostReferences>();
    references->xRef = xRef;
    references->uRef = uRef;
    references->footPoseReference = input.footPoseReference;
    references->stateWeight = wState;
    references->controlWeight = wControl;
    references->terminalWeight = wTerminal;
    references->centroidalWeight = wCent;
    references->centroidalDerivativeWeight = wCentDer;
    references->footPoseWeight = wFrame;

    std::vector<xyz::polymorphic<StageModel>> stages;
    stages.reserve(static_cast<std::size_t>(horizon_));
    const std::vector<bool> contactStates(input.contactActive.begin(), input.contactActive.end());
    const std::vector<pinocchio::FrameIndex> contactIds(footFrameIds_.begin(), footFrameIds_.end());

    for (int k = 0; k < horizon_; ++k)
    {
        const DirectKinoRunningCost runningCost(
            space, nu_, references, model_, gravity_, contactStates, contactIds, footFrameIds_);
        const DirectSemiImplicitKinodynamics dynamics(
            space, model_, gravity_, contactStates, contactIds, kForceSize, dt_);
        StageModel stage(runningCost, dynamics);

        for (int foot = 0; foot < kNumFeet; ++foot)
        {
            const int offset = foot * kForceSize;
            Eigen::VectorXd lower(kForceSize);
            Eigen::VectorXd upper(kForceSize);
            if (input.contactActive[foot])
            {
                lower << -forceLimit, -forceLimit, 0.0, -momentLimit, -momentLimit, -momentLimit;
                upper << forceLimit, forceLimit, forceLimit, momentLimit, momentLimit, momentLimit;
                if (useForceCone)
                {
                    stage.addConstraint(
                        aligator::CentroidalWrenchConeResidualTpl<double>(
                            space.ndx(), nu_, foot, mu, footHalfLength, footHalfWidth),
                        aligator::NegativeOrthantTpl<double>());
                }
                if (constrainStandingFeet)
                {
                    const pinocchio::Motion zeroVelocity = pinocchio::Motion::Zero();
                    stage.addConstraint(
                        aligator::FrameVelocityResidualTpl<double>(
                            space.ndx(), nu_, model_, zeroVelocity, footFrameIds_[foot],
                            pinocchio::LOCAL),
                        aligator::EqualityConstraintTpl<double>());
                }
            }
            else
            {
                lower.setZero();
                upper.setZero();
            }
            stage.addConstraint(
                ControlSliceResidual(space.ndx(), nu_, offset, kForceSize),
                aligator::BoxConstraintTpl<double>(lower, upper));
        }

        Eigen::VectorXd accLower = Eigen::VectorXd::Constant(model_.nv - 6, -jointAccelerationLimit);
        Eigen::VectorXd accUpper = Eigen::VectorXd::Constant(model_.nv - 6, jointAccelerationLimit);
        stage.addConstraint(
            ControlSliceResidual(space.ndx(), nu_, kNumFeet * kForceSize, model_.nv - 6),
            aligator::BoxConstraintTpl<double>(accLower, accUpper));

        stages.emplace_back(stage);
    }

    const DirectKinoTerminalCost terminalCost(space, nu_, references, model_);

    cache->problem = std::make_unique<TrajOptProblem>(x0, stages, terminalCost);
    cache->problem->setInitState(x0);
    cache->references = references;
    cache->solver = std::make_unique<SolverProxDDP>(
        tolerance, muInit, static_cast<std::size_t>(std::max(1, maxIterations)),
        aligator::QUIET);
    cache->solver->rollout_type_ = aligator::RolloutType::NONLINEAR;
    cache->solver->linear_solver_choice = aligator::LQSolverChoice::SERIAL;
    cache->solver->force_initial_condition_ = true;
    cache->solver->max_al_iters = std::max(1, maxAlIterations);

    solverCache_ = std::move(cache);
    if (!collectSolverCacheReferences())
    {
        solverCache_.reset();
        return false;
    }
    updateCachedProblemReferences(input, x0, xRef, uRef);

    const auto solverSetupStart = std::chrono::steady_clock::now();
    solverCache_->solver->setup(*solverCache_->problem);
    solverSetupTime = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - solverSetupStart)
                          .count();
    rebuilt = true;
    return true;
}

bool G1KinodynamicsNmpc::solve(const Input &input)
{
    status_ = -1;
    iterations_ = 0;
    solveTime_ = 0.0;
    firstWrenches_.setZero();
    firstJointAccelerations_.setZero();
    firstControl_.setZero();
    firstState_ = makeState(input);

    if (model_.nq <= 0 || model_.nv <= 6 || nu_ <= kNumFeet * kForceSize)
    {
        return false;
    }
    if (input.q.size() != model_.nq || input.v.size() != model_.nv ||
        input.qReference.size() != model_.nq || input.vReference.size() != model_.nv ||
        !input.q.allFinite() || !input.v.allFinite() ||
        !input.qReference.allFinite() || !input.vReference.allFinite())
    {
        return false;
    }

    const auto tStart = std::chrono::steady_clock::now();
    const Eigen::VectorXd x0 = makeState(input);
    const Eigen::VectorXd xRef = makeStateReference(input);
    const Eigen::VectorXd uRef = nominalControl(input);

    if (xsWarm_.empty() || usWarm_.empty())
    {
        initializeWarmStart(input);
    }
    else
    {
        shiftWarmStart(input);
    }

    std::vector<Eigen::VectorXd> xsInit = xsWarm_;
    std::vector<Eigen::VectorXd> usInit = usWarm_;

    double solverSetupTime = 0.0;
    double solverRunTime = 0.0;
    bool rebuiltSolverCache = false;
    try
    {
        if (!ensureSolverCache(input, x0, xRef, uRef, solverSetupTime, rebuiltSolverCache))
        {
            status_ = -2;
            return false;
        }
        const auto solverRunStart = std::chrono::steady_clock::now();
        solverCache_->solver->run(*solverCache_->problem, xsInit, usInit);
        solverRunTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - solverRunStart).count();
    }
    catch (...)
    {
        status_ = -2;
        return false;
    }

    SolverProxDDP &solver = *solverCache_->solver;
    iterations_ = static_cast<int>(solver.results_.num_iters);
    const auto tEnd = std::chrono::steady_clock::now();
    solveTime_ = std::chrono::duration<double>(tEnd - tStart).count();
    const double derivativeTime = static_cast<double>(solver.derivatives_time_);
    const double ddpTime = static_cast<double>(solver.ddp_time_);
    const double solverTotalTime = solverSetupTime + solverRunTime;
    const double runOtherTime = std::max(0.0, solverRunTime - derivativeTime - ddpTime);
    const double wrapperOtherTime = std::max(0.0, solveTime_ - solverTotalTime);

    std::ostringstream timingOut;
    timingOut << "[Kino-NMPC] prox-ddp timing"
              << " iter=" << iterations_
              << " total_ms=" << std::fixed << std::setprecision(3) << solverTotalTime * 1e3
              << " setup_ms=" << solverSetupTime * 1e3
              << " run_ms=" << solverRunTime * 1e3
              << " derivatives_ms=" << derivativeTime * 1e3
              << " ddp_ms=" << ddpTime * 1e3
              << " run_other_ms=" << runOtherTime * 1e3
              << " wrapper_other_ms=" << wrapperOtherTime * 1e3
              << " cache=" << (rebuiltSolverCache ? "rebuild" : "reuse")
              << " conv=" << std::boolalpha << solver.results_.conv;
    std::cout << timingOut.str() << std::endl;

    if (solver.results_.us.empty() || solver.results_.xs.empty())
    {
        status_ = -3;
        return false;
    }

    status_ = solver.results_.conv ? 0 : 1;
    updateOutputs(solver.results_.xs, solver.results_.us);
    return firstControl_.allFinite() && firstState_.allFinite();
}

void G1KinodynamicsNmpc::updateOutputs(const std::vector<Eigen::VectorXd> &xs,
                                       const std::vector<Eigen::VectorXd> &us)
{
    if (!us.empty() && us.front().size() == nu_)
    {
        firstControl_ = us.front();
        firstWrenches_ = firstControl_.head<kNumFeet * kForceSize>();
        firstJointAccelerations_ = firstControl_.tail(model_.nv - 6);
    }
    if (static_cast<int>(xs.size()) > 1 && xs[1].size() == model_.nq + model_.nv)
    {
        firstState_ = xs[1];
    }

    xsWarm_.clear();
    usWarm_.clear();
    xsWarm_.reserve(xs.size());
    usWarm_.reserve(us.size());
    for (const Eigen::VectorXd &x : xs)
    {
        if (x.size() == model_.nq + model_.nv)
        {
            xsWarm_.push_back(x);
        }
    }
    for (const Eigen::VectorXd &u : us)
    {
        if (u.size() == nu_)
        {
            usWarm_.push_back(u);
        }
    }
}

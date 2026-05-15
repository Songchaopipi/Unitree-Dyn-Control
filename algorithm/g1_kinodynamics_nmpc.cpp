#include "g1_kinodynamics_nmpc.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/centroidal/centroidal-wrench-cone.hpp>
#include <aligator/modelling/constraints/box-constraint.hpp>
#include <aligator/modelling/constraints/equality-constraint.hpp>
#include <aligator/modelling/constraints/negative-orthant.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#include <aligator/modelling/dynamics/kinodynamics-fwd.hpp>
#include <aligator/modelling/function-xpr-slice.hpp>
#include <aligator/modelling/multibody/centroidal-momentum.hpp>
#include <aligator/modelling/multibody/centroidal-momentum-derivative.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <aligator/modelling/multibody/frame-velocity.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/modelling/state-error.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>

namespace
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Space = aligator::MultibodyPhaseSpace<double>;
using CostStack = aligator::CostStackTpl<double>;
using StageModel = aligator::StageModelTpl<double>;
using TrajOptProblem = aligator::TrajOptProblemTpl<double>;
using SolverProxDDP = aligator::SolverProxDDPTpl<double>;
using KinodynamicsFwdDynamics = aligator::dynamics::KinodynamicsFwdDynamicsTpl<double>;
using IntegratorSemiImplEuler = aligator::dynamics::IntegratorSemiImplEulerTpl<double>;
using QuadraticStateCost = aligator::QuadraticStateCostTpl<double>;
using QuadraticControlCost = aligator::QuadraticControlCostTpl<double>;
using QuadraticResidualCost = aligator::QuadraticResidualCostTpl<double>;
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
    std::vector<QuadraticStateCost *> runningStateCosts;
    std::vector<QuadraticControlCost *> runningControlCosts;
    std::array<std::vector<FramePlacementResidual *>, kNumFeet> footPoseResiduals;
    QuadraticStateCost *terminalStateCost{nullptr};
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
    if (!solverCache_ || !solverCache_->problem)
    {
        return false;
    }

    solverCache_->runningStateCosts.clear();
    solverCache_->runningControlCosts.clear();
    for (auto &footResiduals : solverCache_->footPoseResiduals)
    {
        footResiduals.clear();
    }
    solverCache_->terminalStateCost = nullptr;

    solverCache_->runningStateCosts.reserve(static_cast<std::size_t>(horizon_));
    solverCache_->runningControlCosts.reserve(static_cast<std::size_t>(horizon_));
    for (auto &footResiduals : solverCache_->footPoseResiduals)
    {
        footResiduals.reserve(static_cast<std::size_t>(horizon_));
    }

    for (int k = 0; k < horizon_; ++k)
    {
        if (k >= static_cast<int>(solverCache_->problem->stages_.size()))
        {
            return false;
        }
        auto *costStack = solverCache_->problem->stages_[static_cast<std::size_t>(k)]->getCost<CostStack>();
        if (costStack == nullptr)
        {
            return false;
        }

        auto *stateCost = costStack->getComponent<QuadraticStateCost>("state_cost");
        auto *controlCost = costStack->getComponent<QuadraticControlCost>("control_cost");
        if (stateCost == nullptr || controlCost == nullptr)
        {
            return false;
        }
        solverCache_->runningStateCosts.push_back(stateCost);
        solverCache_->runningControlCosts.push_back(controlCost);

        for (int foot = 0; foot < kNumFeet; ++foot)
        {
            const char *name = (foot == 0) ? "left_foot_pose_cost" : "right_foot_pose_cost";
            auto *poseCost = costStack->getComponent<QuadraticResidualCost>(name);
            if (poseCost == nullptr)
            {
                return false;
            }
            auto *poseResidual = poseCost->getResidual<FramePlacementResidual>();
            if (poseResidual == nullptr)
            {
                return false;
            }
            solverCache_->footPoseResiduals[foot].push_back(poseResidual);
        }
    }

    auto *terminalCostStack = dynamic_cast<CostStack *>(&*solverCache_->problem->term_cost_);
    if (terminalCostStack == nullptr)
    {
        return false;
    }
    solverCache_->terminalStateCost =
        terminalCostStack->getComponent<QuadraticStateCost>("state_cost");
    return solverCache_->terminalStateCost != nullptr;
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

    for (QuadraticStateCost *cost : solverCache_->runningStateCosts)
    {
        cost->setTarget(xRef);
    }
    for (QuadraticControlCost *cost : solverCache_->runningControlCosts)
    {
        cost->setTarget(uRef);
    }
    for (int foot = 0; foot < kNumFeet; ++foot)
    {
        for (FramePlacementResidual *residual : solverCache_->footPoseResiduals[foot])
        {
            residual->setReference(input.footPoseReference[foot]);
        }
    }
    if (solverCache_->terminalStateCost != nullptr)
    {
        solverCache_->terminalStateCost->setTarget(xRef);
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

    std::vector<xyz::polymorphic<StageModel>> stages;
    stages.reserve(static_cast<std::size_t>(horizon_));
    const std::vector<bool> contactStates(input.contactActive.begin(), input.contactActive.end());
    const std::vector<pinocchio::FrameIndex> contactIds(footFrameIds_.begin(), footFrameIds_.end());

    for (int k = 0; k < horizon_; ++k)
    {
        CostStack runningCost(space, nu_);
        runningCost.addCost("state_cost",
                            aligator::QuadraticStateCostTpl<double>(space, nu_, xRef, wState));
        runningCost.addCost("control_cost",
                            aligator::QuadraticControlCostTpl<double>(space, uRef, wControl));
        runningCost.addCost("centroidal_cost",
                            aligator::QuadraticResidualCostTpl<double>(
                                space,
                                aligator::CentroidalMomentumResidualTpl<double>(
                                    space.ndx(), nu_, model_, Vector6d::Zero()),
                                wCent));
        runningCost.addCost("centroidal_derivative_cost",
                            aligator::QuadraticResidualCostTpl<double>(
                                space,
                                aligator::CentroidalMomentumDerivativeResidualTpl<double>(
                                    space.ndx(), model_, gravity_, contactStates, contactIds, kForceSize),
                                wCentDer));
        for (int foot = 0; foot < kNumFeet; ++foot)
        {
            runningCost.addCost(foot == 0 ? "left_foot_pose_cost" : "right_foot_pose_cost",
                                aligator::QuadraticResidualCostTpl<double>(
                                    space,
                                    aligator::FramePlacementResidualTpl<double>(
                                        space.ndx(), nu_, model_, input.footPoseReference[foot], footFrameIds_[foot]),
                                    wFrame));
        }

        const KinodynamicsFwdDynamics ode(space, model_, gravity_, contactStates, contactIds, kForceSize);
        const IntegratorSemiImplEuler dynamics(ode, dt_);
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

    CostStack terminalCost(space, nu_);
    terminalCost.addCost("state_cost",
                         aligator::QuadraticStateCostTpl<double>(space, nu_, xRef, wTerminal));
    terminalCost.addCost("centroidal_cost",
                         aligator::QuadraticResidualCostTpl<double>(
                             space,
                             aligator::CentroidalMomentumResidualTpl<double>(
                                 space.ndx(), nu_, model_, Vector6d::Zero()),
                             wCent * 5.0));

    cache->problem = std::make_unique<TrajOptProblem>(x0, stages, terminalCost);
    cache->problem->setInitState(x0);
    cache->solver = std::make_unique<SolverProxDDP>(
        tolerance, muInit, static_cast<std::size_t>(std::max(1, maxIterations)),
        aligator::QUIET);
    cache->solver->rollout_type_ = aligator::RolloutType::LINEAR;
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

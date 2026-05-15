#include "g1_centroidal_nmpc.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/centroidal/angular-acceleration.hpp>
#include <aligator/modelling/centroidal/angular-momentum.hpp>
#include <aligator/modelling/centroidal/centroidal-acceleration.hpp>
#include <aligator/modelling/centroidal/centroidal-translation.hpp>
#include <aligator/modelling/centroidal/linear-momentum.hpp>
#include <aligator/modelling/constraints/box-constraint.hpp>
#include <aligator/modelling/constraints/negative-orthant.hpp>
#include <aligator/modelling/contact-map.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/dynamics/ode-abstract.hpp>
#include <aligator/modelling/dynamics/centroidal-fwd.hpp>
#include <aligator/modelling/dynamics/integrator-euler.hpp>
#include <aligator/modelling/function-xpr-slice.hpp>
#include <aligator/modelling/state-error.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>

#include "pino_kin_dyn.h"

namespace
{
constexpr double kGravity = 9.80665;
const Eigen::Vector3d kGravityVector(0.0, 0.0, -kGravity);

Eigen::Vector3d comVelocityWorld(const DataBus &robotState)
{
    if (robotState.Jcom_W.rows() == 3 && robotState.Jcom_W.cols() == robotState.dq.size())
    {
        const Eigen::Vector3d vCom = robotState.Jcom_W * robotState.dq;
        if (vCom.allFinite())
        {
            return vCom;
        }
    }
    if (robotState.dq.size() >= 3)
    {
        return robotState.dq.head<3>();
    }
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d angularMomentumWorld(const DataBus &robotState)
{
    if (robotState.dyn_Ag.rows() >= 6 && robotState.dyn_Ag.cols() == robotState.dq.size())
    {
        const Eigen::VectorXd h = robotState.dyn_Ag * robotState.dq;
        if (h.size() >= 6 && h.allFinite())
        {
            return h.tail<3>();
        }
    }
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d firstFiniteOrZero(const Eigen::Vector3d &value)
{
    return value.allFinite() ? value : Eigen::Vector3d::Zero();
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d out;
    out << 0.0, -v.z(), v.y(),
        v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return out;
}

std::string vector3String(const Eigen::Vector3d &v)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(4)
        << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
    return out.str();
}

template <std::size_t N>
std::string contactString(const std::array<bool, N> &active)
{
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < N; ++i)
    {
        out << (active[i] ? 1 : 0);
        if (i + 1 < N)
        {
            out << ", ";
        }
    }
    out << "]";
    return out.str();
}

class PlaneCentroidalDynamics final : public aligator::dynamics::ODEAbstractTpl<double>
{
public:
    using Base = aligator::dynamics::ODEAbstractTpl<double>;
    using Data = aligator::dynamics::ContinuousDynamicsDataTpl<double>;

    PlaneCentroidalDynamics(const aligator::VectorSpaceTpl<double> &space,
                            double mass,
                            const Eigen::Vector3d &gravity,
                            const aligator::ContactMapTpl<double> &contactMap)
        : Base(space, static_cast<int>(contactMap.size_) * 6),
          mass_(std::max(1.0, mass)),
          gravity_(gravity),
          contactMap_(contactMap),
          H_(g1_kin_dyn::planeForceToWrenchMap())
    {
    }

    void forward(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override
    {
        data.xdot_.setZero();
        data.xdot_.head<3>() = x.segment<3>(3) / mass_;
        data.xdot_.segment<3>(3) = mass_ * gravity_;

        for (std::size_t foot = 0; foot < contactMap_.size_; ++foot)
        {
            if (!contactMap_.contact_states_[foot])
            {
                continue;
            }
            const long offset = static_cast<long>(6 * foot);
            const Eigen::Matrix<double, 6, 1> wrench = H_ * u.segment<6>(offset);
            const Eigen::Vector3d force = wrench.head<3>();
            const Eigen::Vector3d moment = wrench.tail<3>();
            const Eigen::Vector3d r = contactMap_.contact_poses_[foot] - x.head<3>();
            data.xdot_.segment<3>(3) += force;
            data.xdot_.tail<3>() += r.cross(force) + moment;
        }
    }

    void dForward(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override
    {
        data.Jx_.setZero();
        data.Ju_.setZero();
        data.Jx_.block<3, 3>(0, 3).setIdentity();
        data.Jx_.block<3, 3>(0, 3) /= mass_;

        for (std::size_t foot = 0; foot < contactMap_.size_; ++foot)
        {
            if (!contactMap_.contact_states_[foot])
            {
                continue;
            }
            const long offset = static_cast<long>(6 * foot);
            const Eigen::Matrix<double, 6, 1> wrench = H_ * u.segment<6>(offset);
            const Eigen::Vector3d force = wrench.head<3>();
            data.Jx_.block<3, 3>(6, 0) += skew(force);
            data.Ju_.block<3, 6>(3, offset) = H_.topRows<3>();
            const Eigen::Vector3d r = contactMap_.contact_poses_[foot] - x.head<3>();
            data.Ju_.block<3, 6>(6, offset) = skew(r) * H_.topRows<3>() + H_.bottomRows<3>();
        }
    }

    std::shared_ptr<Data> createData() const override
    {
        return std::make_shared<Data>(9, static_cast<int>(contactMap_.size_) * 6);
    }

private:
    double mass_{1.0};
    Eigen::Vector3d gravity_{Eigen::Vector3d::Zero()};
    aligator::ContactMapTpl<double> contactMap_;
    Eigen::Matrix<double, 6, 6> H_{Eigen::Matrix<double, 6, 6>::Identity()};
};

class PlaneFootConeResidual final : public aligator::StageFunctionTpl<double>
{
public:
    using Base = aligator::StageFunctionTpl<double>;
    using Data = aligator::StageFunctionDataTpl<double>;

    PlaneFootConeResidual(int ndx,
                          int nu,
                          int foot,
                          double mu,
                          double halfWidth,
                          double front,
                          double back,
                          double yawFriction,
                          double fzLow)
        : Base(ndx, nu, 11),
          foot_(foot),
          fzLow_(fzLow),
          cone_(g1_kin_dyn::roMoCoPlaneFootCone(mu, halfWidth, front, back, yawFriction))
    {
    }

    void evaluate(const ConstVectorRef &, const ConstVectorRef &u, Data &data) const override
    {
        data.value_ = cone_ * u.segment<6>(6 * foot_);
        data.value_(0) += fzLow_;
    }

    void computeJacobians(const ConstVectorRef &, const ConstVectorRef &, Data &data) const override
    {
        data.Jx_.setZero();
        data.Ju_.setZero();
        data.Ju_.block<11, 6>(0, 6 * foot_) = cone_;
    }

private:
    int foot_{0};
    double fzLow_{0.0};
    Eigen::Matrix<double, 11, 6> cone_;
};

class PlaneFootWrenchResidual final : public aligator::StageFunctionTpl<double>
{
public:
    using Base = aligator::StageFunctionTpl<double>;
    using Data = aligator::StageFunctionDataTpl<double>;

    PlaneFootWrenchResidual(int ndx, int nu, int foot)
        : Base(ndx, nu, 6),
          foot_(foot),
          H_(g1_kin_dyn::planeForceToWrenchMap())
    {
    }

    void evaluate(const ConstVectorRef &, const ConstVectorRef &u, Data &data) const override
    {
        data.value_ = H_ * u.segment<6>(6 * foot_);
    }

    void computeJacobians(const ConstVectorRef &, const ConstVectorRef &, Data &data) const override
    {
        data.Jx_.setZero();
        data.Ju_.setZero();
        data.Ju_.block<6, 6>(0, 6 * foot_) = H_;
    }

private:
    int foot_{0};
    Eigen::Matrix<double, 6, 6> H_{Eigen::Matrix<double, 6, 6>::Identity()};
};

} // namespace

G1CentroidalNmpc::G1CentroidalNmpc(int horizon, double dt)
{
    horizon_ = std::max(1, horizon);
    dt_ = std::max(1e-4, dt);
}

bool G1CentroidalNmpc::solveFromDataBus(const DataBus &robotState, double mass)
{
    return solve(buildInputFromDataBus(robotState, mass));
}

G1CentroidalNmpc::Input G1CentroidalNmpc::buildInputFromDataBus(const DataBus &robotState, double mass) const
{
    Input input;
    input.mass = std::max(1.0, mass);
    const Eigen::Vector3d vCom = comVelocityWorld(robotState);
    input.current.head<3>() = robotState.pCoM_W;
    input.current.segment<3>(3) = input.mass * vCom;
    input.current.tail<3>() = angularMomentumWorld(robotState);
    input.contactPositionWorld = {robotState.fe_l_pos_W, robotState.fe_r_pos_W};
    input.contactActive = {{
        robotState.motionState == DataBus::Stand || robotState.walk_left_contact,
        robotState.motionState == DataBus::Stand || robotState.walk_right_contact}};

    input.comReference = robotState.centroidal_nmpc_enabled ?
                             robotState.centroidal_nmpc_com_pos_des :
                             robotState.pCoM_W;
    input.comVelocityReference = robotState.centroidal_nmpc_enabled ?
                                     robotState.centroidal_nmpc_com_vel_des :
                                     Eigen::Vector3d::Zero();
    input.comAccelerationReference = robotState.centroidal_nmpc_enabled ?
                                         robotState.centroidal_nmpc_com_acc_des :
                                         Eigen::Vector3d::Zero();
    return input;
}

Eigen::Matrix<double, G1CentroidalNmpc::kNu, 1>
G1CentroidalNmpc::nominalControl(const Input &input) const
{
    return nominalControlAt(input, 0);
}

Eigen::Matrix<double, G1CentroidalNmpc::kNu, 1>
G1CentroidalNmpc::nominalControlAt(const Input &input, int knot) const
{
    Eigen::Matrix<double, kNu, 1> u = Eigen::Matrix<double, kNu, 1>::Zero();
    const std::array<bool, kNumFeet> active = contactActiveAt(input, knot);
    int activeContacts = 0;
    for (bool isActive : active)
    {
        activeContacts += isActive ? 1 : 0;
    }
    if (activeContacts == 0)
    {
        return u;
    }
    const double nominalFz = input.mass * kGravity / static_cast<double>(activeContacts);
    for (int foot = 0; foot < kNumFeet; ++foot)
    {
        if (active[foot])
        {
            u.segment<6>(foot * kForceSize) = g1_kin_dyn::distributeVerticalFootLoad(nominalFz);
        }
    }
    return u;
}

std::array<bool, G1CentroidalNmpc::kNumFeet>
G1CentroidalNmpc::contactActiveAt(const Input &input, int knot) const
{
    std::array<bool, kNumFeet> active = input.contactActive;
    if (input.contactTable.rows() > 0 && input.contactTable.cols() >= kNumFeet)
    {
        const int row = std::clamp(knot, 0, static_cast<int>(input.contactTable.rows()) - 1);
        for (int foot = 0; foot < kNumFeet; ++foot)
        {
            active[foot] = input.contactTable(row, foot) != 0;
        }
    }
    return active;
}

std::array<Eigen::Vector3d, G1CentroidalNmpc::kNumFeet>
G1CentroidalNmpc::contactPositionsAt(const Input &input, int knot) const
{
    if (!input.contactPositionWorldHorizon.empty())
    {
        const int row = std::clamp(knot, 0, static_cast<int>(input.contactPositionWorldHorizon.size()) - 1);
        return input.contactPositionWorldHorizon[static_cast<std::size_t>(row)];
    }
    return input.contactPositionWorld;
}

Eigen::Matrix<double, G1CentroidalNmpc::kNx, 1>
G1CentroidalNmpc::referenceAt(const Input &input, int knot) const
{
    Eigen::Matrix<double, kNx, 1> ref = Eigen::Matrix<double, kNx, 1>::Zero();
    const double t = static_cast<double>(knot + 1) * dt_;
    ref.head<3>() = input.comReference + t * input.comVelocityReference;
    ref.segment<3>(3) = input.mass * input.comVelocityReference;
    if (input.reference.rows() == kNx && input.reference.cols() > 0)
    {
        const int col = std::clamp(knot, 0, static_cast<int>(input.reference.cols()) - 1);
        ref = input.reference.col(col);
    }
    return ref;
}

void G1CentroidalNmpc::initializeWarmStart(const Input &input)
{
    xsWarm_.assign(static_cast<std::size_t>(horizon_ + 1), input.current);
    usWarm_.resize(static_cast<std::size_t>(horizon_));
    for (int k = 0; k < horizon_; ++k)
    {
        usWarm_[static_cast<std::size_t>(k)] = nominalControlAt(input, k);
    }
}

void G1CentroidalNmpc::shiftWarmStart(const Input &input)
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
    xsWarm_.front() = input.current;
    for (int k = 0; k + 1 < horizon_; ++k)
    {
        usWarm_[k] = usWarm_[k + 1];
    }
    for (int k = 0; k < horizon_; ++k)
    {
        const std::array<bool, kNumFeet> active = contactActiveAt(input, k);
        const Eigen::Matrix<double, kNu, 1> uNomK = nominalControlAt(input, k);
        for (int foot = 0; foot < kNumFeet; ++foot)
        {
            const int offset = foot * kForceSize;
            if (!active[foot])
            {
                usWarm_[static_cast<std::size_t>(k)].segment<kForceSize>(offset).setZero();
            }
            else if (usWarm_[static_cast<std::size_t>(k)].segment<kForceSize>(offset).norm() < 1e-9)
            {
                usWarm_[static_cast<std::size_t>(k)].segment<kForceSize>(offset) =
                    uNomK.segment<kForceSize>(offset);
            }
        }
    }
}

bool G1CentroidalNmpc::solve(const Input &input)
{
    status_ = -1;
    iterations_ = 0;
    solveTime_ = 0.0;
    firstContactForces_.setZero();
    firstPlaneForces_.setZero();
    firstWrenches_.setZero();
    firstComPosition_ = input.current.head<3>();
    firstComVelocity_ = input.current.segment<3>(3) / std::max(1.0, input.mass);
    firstComAcceleration_.setZero();

    if (logSolve)
    {
        const std::array<bool, kNumFeet> activeNow = contactActiveAt(input, 0);
        std::cout << "[CD-NMPC] solve begin"
                  << " horizon=" << horizon_
                  << " dt=" << dt_
                  << " mass=" << input.mass
                  << " contact=" << contactString(activeNow)
                  << " com=" << vector3String(input.current.head<3>())
                  << " ref=" << vector3String(input.comReference)
                  << " vref=" << vector3String(input.comVelocityReference)
                  << std::endl;
    }

    if (!input.current.allFinite() || !input.comReference.allFinite() ||
        !input.comVelocityReference.allFinite() || input.mass <= 0.0)
    {
        if (logSolve)
        {
            std::cerr << "[CD-NMPC] invalid input"
                      << " current_finite=" << input.current.allFinite()
                      << " ref_finite=" << input.comReference.allFinite()
                      << " vref_finite=" << input.comVelocityReference.allFinite()
                      << " mass=" << input.mass
                      << std::endl;
        }
        return false;
    }

    using Space = aligator::VectorSpaceTpl<double>;
    using CostStack = aligator::CostStackTpl<double>;
    using StageModel = aligator::StageModelTpl<double>;
    using TrajOptProblem = aligator::TrajOptProblemTpl<double>;
    using SolverProxDDP = aligator::SolverProxDDPTpl<double>;
    using ContactMap = aligator::ContactMapTpl<double>;
    using IntegratorEuler = aligator::dynamics::IntegratorEulerTpl<double>;

    const auto tStart = std::chrono::steady_clock::now();
    const Space space(kNx);
    const std::vector<std::string> contactNames = {"left_foot", "right_foot"};
    const Eigen::Matrix<double, kNu, 1> uNom = nominalControl(input);

    Eigen::Matrix3d wCom = Eigen::Matrix3d::Zero();
    wCom.diagonal() << 50.0, 50.0, 150.0;
    
    Eigen::Matrix3d wLinearMomentum = Eigen::Matrix3d::Zero();
    wLinearMomentum.diagonal() << 0.5, 0.5, 1.0;
    
    Eigen::Matrix3d wAngularMomentum = Eigen::Matrix3d::Zero();
    wAngularMomentum.diagonal() << 5, 5.0, 2.0; // *可以近似看作roll/pitch/yaw
    
    Eigen::Matrix<double, kNu, kNu> wControl =
        Eigen::Matrix<double, kNu, kNu>::Identity();
    
    for (int foot = 0; foot < kNumFeet; ++foot)
    {
        wControl.block<6, 6>(foot * kForceSize, foot * kForceSize).diagonal()
            << 0.3, 0.3, 0.03, 5.0, 5.0, 5.0;
    }
    
    wControl *= nominalForceWeight;
    
    Eigen::Matrix<double, kNx, kNx> wTerminal =
        Eigen::Matrix<double, kNx, kNx>::Zero();
    
    wTerminal.diagonal() << 100.0, 100.0, 250.0,
        2.0, 2.0, 4.0,
        2.0, 4.0, 2.0;

    const Eigen::Matrix<double, kNx, 1> terminalRef = referenceAt(input, horizon_ - 1);

    std::vector<xyz::polymorphic<StageModel>> stages;
    stages.reserve(static_cast<std::size_t>(horizon_));
    for (int k = 0; k < horizon_; ++k)
    {
        const Eigen::Matrix<double, kNx, 1> xRef = referenceAt(input, k);
        const Eigen::Vector3d comRef = xRef.head<3>();
        const Eigen::Vector3d linMomRef = xRef.segment<3>(3);
        const Eigen::Vector3d angMomRef = xRef.segment<3>(6);
        const std::array<bool, kNumFeet> activeAtK = contactActiveAt(input, k);
        const std::array<Eigen::Vector3d, kNumFeet> positionsAtK = contactPositionsAt(input, k);
        const std::vector<bool> contactStates(activeAtK.begin(), activeAtK.end());
        ContactMap::PoseVec contactPoses;
        contactPoses.reserve(kNumFeet);
        for (const Eigen::Vector3d &p : positionsAtK)
        {
            contactPoses.push_back(firstFiniteOrZero(p));
        }
        const ContactMap contactMap(contactNames, contactStates, contactPoses);
        const Eigen::Matrix<double, kNu, 1> uNomK = nominalControlAt(input, k);

        CostStack runningCost(space, kNu);
        runningCost.addCost("com_cost",
                            aligator::QuadraticResidualCostTpl<double>(
                                space,
                                aligator::CentroidalCoMResidualTpl<double>(kNx, kNu, comRef),
                                wCom));
        runningCost.addCost("linear_momentum_cost",
                            aligator::QuadraticResidualCostTpl<double>(
                                space,
                                aligator::LinearMomentumResidualTpl<double>(kNx, kNu, linMomRef),
                                wLinearMomentum));
        runningCost.addCost("angular_momentum_cost",
                            aligator::QuadraticResidualCostTpl<double>(
                                space,
                                aligator::AngularMomentumResidualTpl<double>(kNx, kNu, angMomRef),
                                wAngularMomentum));
        runningCost.addCost("control_cost",
                            aligator::QuadraticControlCostTpl<double>(space, uNomK, wControl));

        const PlaneCentroidalDynamics ode(space, input.mass, kGravityVector, contactMap);
        const IntegratorEuler dynamics(ode, dt_);
        StageModel stage(runningCost, dynamics);
        for (int foot = 0; foot < kNumFeet; ++foot)
        {
            Eigen::VectorXd lower = Eigen::VectorXd::Constant(6, -fMax);
            Eigen::VectorXd upper = Eigen::VectorXd::Constant(6, fMax);
            if (activeAtK[foot])
            {
                stage.addConstraint(
                    PlaneFootConeResidual(space.ndx(), kNu, foot, mu, footHalfWidth, footFront, footBack, yawFriction, fzLow),
                    aligator::NegativeOrthantTpl<double>());
                lower(2) = 0.0;
                lower(4) = 0.0;
                lower(5) = 0.0;
                upper(2) = fMax;
                upper(4) = fMax;
                upper(5) = fMax;
            }
            else
            {
                lower.setZero();
                upper.setZero();
            }
            stage.addConstraint(
                aligator::FunctionSliceXprTpl<double>(
                    aligator::ControlErrorResidualTpl<double>(space.ndx(), Eigen::VectorXd::Zero(kNu)),
                    {foot * 6 + 0, foot * 6 + 1, foot * 6 + 2,
                     foot * 6 + 3, foot * 6 + 4, foot * 6 + 5}),
                aligator::BoxConstraintTpl<double>(lower, upper));
            if (activeAtK[foot] && std::count(activeAtK.begin(), activeAtK.end(), true) == 1)
            {
                Eigen::Matrix<double, 6, 1> wrenchLower;
                Eigen::Matrix<double, 6, 1> wrenchUpper;
                const double fzSingleLow =
                    std::max(fzLow, singleSupportFzMinSafetyFactor * input.mass * kGravity);
                wrenchLower << -singleSupportTangentialForceMax,
                    -singleSupportTangentialForceMax,
                    fzSingleLow,
                    -singleSupportRollMomentMax,
                    -singleSupportPitchMomentMax,
                    -singleSupportYawMomentMax;
                wrenchUpper << singleSupportTangentialForceMax,
                    singleSupportTangentialForceMax,
                    std::max(fzLow, singleSupportFzSafetyFactor * input.mass * kGravity),
                    singleSupportRollMomentMax,
                    singleSupportPitchMomentMax,
                    singleSupportYawMomentMax;
                stage.addConstraint(
                    PlaneFootWrenchResidual(space.ndx(), kNu, foot),
                    aligator::BoxConstraintTpl<double>(wrenchLower, wrenchUpper));
            }
        }
        stages.emplace_back(stage);
    }

    const aligator::QuadraticStateCostTpl<double> terminalCost(space, kNu, terminalRef, wTerminal);
    TrajOptProblem problem(input.current, stages, terminalCost);
    problem.setInitState(input.current);

    const Eigen::Matrix<double, kNu, 1> uNomNow = nominalControl(input);
    if (xsWarm_.empty() || usWarm_.empty())
    {
        initializeWarmStart(input);
    }
    else
    {
        shiftWarmStart(input);
        usWarm_.back() = uNomNow;
    }

    std::vector<Eigen::VectorXd> xsInit;
    std::vector<Eigen::VectorXd> usInit;
    xsInit.reserve(xsWarm_.size());
    usInit.reserve(usWarm_.size());
    for (const auto &x : xsWarm_)
    {
        xsInit.push_back(x);
    }
    for (const auto &u : usWarm_)
    {
        usInit.push_back(u);
    }

    SolverProxDDP solver(tolerance, muInit, static_cast<std::size_t>(std::max(1, maxIterations)),
                         logSolverIterations ? aligator::VERBOSE : aligator::QUIET);
    solver.rollout_type_ = aligator::RolloutType::LINEAR;
    solver.linear_solver_choice = aligator::LQSolverChoice::SERIAL;
    solver.force_initial_condition_ = true;
    solver.max_al_iters = static_cast<std::size_t>(std::max(1, maxAlIterations));

    try
    {
        solver.setup(problem);
        solver.run(problem, xsInit, usInit);
    }
    catch (const std::exception &e)
    {
        status_ = -2;
        if (logSolve)
        {
            std::cerr << "[CD-NMPC] solver exception: " << e.what() << std::endl;
        }
        return false;
    }
    catch (...)
    {
        status_ = -2;
        if (logSolve)
        {
            std::cerr << "[CD-NMPC] solver exception: unknown" << std::endl;
        }
        return false;
    }

    iterations_ = static_cast<int>(solver.results_.num_iters);
    const auto tEnd = std::chrono::steady_clock::now();
    solveTime_ = std::chrono::duration<double>(tEnd - tStart).count();

    if (solver.results_.us.empty() || solver.results_.xs.empty())
    {
        status_ = -3;
        if (logSolve)
        {
            std::cerr << "[CD-NMPC] solver returned empty trajectory"
                      << " iter=" << iterations_
                      << " time_ms=" << 1000.0 * solveTime_
                      << " conv=" << solver.results_.conv
                      << " prim=" << solver.results_.prim_infeas
                      << " dual=" << solver.results_.dual_infeas
                      << std::endl;
        }
        return false;
    }

    status_ = solver.results_.conv ? 0 : 1;
    updateOutputs(input, solver.results_.xs, solver.results_.us);
    const bool outputFinite = firstContactForces_.allFinite();
    const bool primalFeasible = solver.results_.prim_infeas <= tolerance;
    if (logSolve)
    {
        std::ostringstream out;
        out << std::boolalpha
            << "[CD-NMPC] solve done"
            << " status=" << status_
            << " accepted=" << outputFinite
            << " converged=" << solver.results_.conv
            << " primal_feasible=" << primalFeasible
            << " iter=" << iterations_
            << " al_iter=" << solver.results_.al_iter
            << " time_ms=" << std::fixed << std::setprecision(3) << 1000.0 * solveTime_
            << std::scientific
            << " prim=" << solver.results_.prim_infeas
            << " dual=" << solver.results_.dual_infeas
            << " cost=" << solver.results_.traj_cost_
            << " merit=" << solver.results_.merit_value_
            << std::fixed << std::setprecision(3)
            << " wrench_fz_lr=[" << firstWrenches_(2) << ", " << firstWrenches_(8) << "]"
            << " com_next=" << vector3String(firstComPosition_);
        std::cout << out.str() << std::endl;
    }
    return outputFinite;
}

void G1CentroidalNmpc::updateOutputs(const Input &input,
                                     const std::vector<Eigen::VectorXd> &xs,
                                     const std::vector<Eigen::VectorXd> &us)
{
    if (!us.empty() && us.front().size() == kNu)
    {
        firstContactForces_ = us.front();
    }
    if (static_cast<int>(xs.size()) > 1 && xs[1].size() == kNx)
    {
        firstComPosition_ = xs[1].head<3>();
        firstComVelocity_ = xs[1].segment<3>(3) / std::max(1.0, input.mass);
    }
    firstComAcceleration_ = kGravityVector;
    const Eigen::Matrix<double, 6, 6> HContact = g1_kin_dyn::planeForceToWrenchMap();
    for (int foot = 0; foot < kNumFeet; ++foot)
    {
        const Eigen::Matrix<double, 6, 1> planeForce =
            firstContactForces_.segment<6>(foot * kForceSize);
        const Eigen::Matrix<double, 6, 1> wrench = HContact * planeForce;
        const Eigen::Vector3d f = wrench.head<3>();
        firstComAcceleration_ += f / std::max(1.0, input.mass);
        firstPlaneForces_.segment<6>(6 * foot) = planeForce;
        firstWrenches_.segment<6>(6 * foot) = wrench;
    }

    xsWarm_.clear();
    usWarm_.clear();
    xsWarm_.reserve(xs.size());
    usWarm_.reserve(us.size());
    for (const Eigen::VectorXd &x : xs)
    {
        if (x.size() == kNx)
        {
            xsWarm_.push_back(x);
        }
    }
    for (const Eigen::VectorXd &u : us)
    {
        if (u.size() == kNu)
        {
            usWarm_.push_back(u);
        }
    }
}

void G1CentroidalNmpc::dataBusWrite(DataBus &robotState) const
{
    robotState.centroidal_nmpc_enabled = status_ >= 0;
    robotState.centroidal_nmpc_status = status_;
    robotState.centroidal_nmpc_nWSR = iterations_;
    robotState.centroidal_nmpc_cpuTime = solveTime_;
    robotState.centroidal_nmpc_com_pos_des = firstComPosition_;
    robotState.centroidal_nmpc_com_vel_des = firstComVelocity_;
    robotState.centroidal_nmpc_com_acc_des = firstComAcceleration_;
    robotState.centroidal_nmpc_Fr_des = firstWrenches_;
    robotState.Fr_ff = firstWrenches_;
    robotState.qpStatus_MPC = status_;
}

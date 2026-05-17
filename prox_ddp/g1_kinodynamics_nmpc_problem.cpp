#include "g1_kinodynamics_nmpc.h"

#include "detail/control_slice.hpp"
#include "detail/direct_kino_costs.hpp"
#include "detail/kino_weights.hpp"
#include "detail/mutable_contact_constraints.hpp"
#include "detail/semi_implicit_kinodynamics.hpp"
#include "detail/solver_cache.hpp"

#include <algorithm>
#include <chrono>

#include <aligator/modelling/constraints/box-constraint.hpp>
#include <aligator/modelling/constraints/equality-constraint.hpp>
#include <aligator/modelling/constraints/negative-orthant.hpp>

using g1_kino_detail::ControlSliceResidual;
using g1_kino_detail::DirectKinoCostReferences;
using g1_kino_detail::DirectKinoRunningCost;
using g1_kino_detail::DirectKinoTerminalCost;
using g1_kino_detail::DirectSemiImplicitKinodynamics;
using g1_kino_detail::MaskedFrameVelocityResidual;
using g1_kino_detail::MaskedWrenchConeResidual;
using g1_kino_detail::MutableBoxBounds;
using g1_kino_detail::MutableBoxConstraint;
using g1_kino_detail::Space;
using g1_kino_detail::SolverProxDDP;
using g1_kino_detail::StageModel;
using g1_kino_detail::TrajOptProblem;
using g1_kino_detail::makeControlWeight;
using g1_kino_detail::makeStateWeightDiagonal;

// 函数说明：根据结构签名复用或重建 Aligator problem/solver。
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
    signature.lineSearchAlphaMin = lineSearchAlphaMin;
    signature.maxLineSearchSteps = maxLineSearchSteps;
    signature.numThreads = std::max(1, numThreads);
    signature.useLinearRollout = useLinearRollout;
    signature.useParallelLq = useParallelLq;

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
    // 说明：这里保持 g1_cd_nmpc_ik_walk 的意图，线动量和角动量都参与调节。
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
    references->uRefHorizon = nominalControlHorizon(input);
    references->footPoseReference = input.footPoseReference;
    references->footPoseReferenceHorizon = input.footPoseReferenceHorizon;
    references->stateWeight = wState;
    references->controlWeight = wControl;
    references->terminalWeight = wTerminal;
    references->centroidalWeight = wCent;
    references->centroidalDerivativeWeight = wCentDer;
    references->footPoseWeight = wFrame;

    std::vector<xyz::polymorphic<StageModel>> stages;
    stages.reserve(static_cast<std::size_t>(horizon_));
    const std::vector<pinocchio::FrameIndex> contactIds(footFrameIds_.begin(), footFrameIds_.end());
    cache->contactActivations.resize(static_cast<std::size_t>(horizon_));
    cache->wrenchBounds.resize(static_cast<std::size_t>(horizon_));

    for (int k = 0; k < horizon_; ++k)
    {
        const std::array<bool, kNumFeet> alwaysActive{{true, true}};
        const std::vector<bool> contactStates(alwaysActive.begin(), alwaysActive.end());
        const DirectKinoRunningCost runningCost(
            space, nu_, k, references, model_, gravity_, contactStates, contactIds, footFrameIds_);
        const DirectSemiImplicitKinodynamics dynamics(
            space, model_, gravity_, contactStates, contactIds, kForceSize, dt_);
        StageModel stage(runningCost, dynamics);

        for (int foot = 0; foot < kNumFeet; ++foot)
        {
            const int offset = foot * kForceSize;
            auto activation = std::make_shared<g1_kino_detail::ContactActivation>();
            auto bounds = std::make_shared<MutableBoxBounds>(kForceSize);
            cache->contactActivations[static_cast<std::size_t>(k)][foot] = activation;
            cache->wrenchBounds[static_cast<std::size_t>(k)][foot] = bounds;

            if (useForceCone)
            {
                stage.addConstraint(
                    MaskedWrenchConeResidual(
                        space.ndx(), nu_, foot, mu, footHalfLength, footHalfWidth, activation),
                    aligator::NegativeOrthantTpl<double>());
            }
            if (constrainStandingFeet)
            {
                const pinocchio::Motion zeroVelocity = pinocchio::Motion::Zero();
                stage.addConstraint(
                    MaskedFrameVelocityResidual(
                        space.ndx(), nu_, model_, zeroVelocity, footFrameIds_[foot],
                        pinocchio::LOCAL, activation),
                    aligator::EqualityConstraintTpl<double>());
            }
            stage.addConstraint(
                ControlSliceResidual(space.ndx(), nu_, offset, kForceSize),
                MutableBoxConstraint(bounds));
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
    const bool effectiveLinearRollout = useLinearRollout || useParallelLq;
    cache->solver->rollout_type_ =
        effectiveLinearRollout ? aligator::RolloutType::LINEAR : aligator::RolloutType::NONLINEAR;
    cache->solver->linear_solver_choice =
        useParallelLq ? aligator::LQSolverChoice::PARALLEL : aligator::LQSolverChoice::SERIAL;
    cache->solver->force_initial_condition_ = true;
    cache->solver->max_al_iters = std::max(1, maxAlIterations);
    cache->solver->ls_params.alpha_min = std::max(1e-6, lineSearchAlphaMin);
    cache->solver->ls_params.max_num_steps = static_cast<std::size_t>(std::max(1, maxLineSearchSteps));
    cache->solver->setNumThreads(static_cast<std::size_t>(std::max(1, numThreads)));

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

#include "g1_kinodynamics_nmpc.h"

#include "detail/proxddp_timing.hpp"
#include "detail/solver_cache.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

using g1_kino_detail::ProxDdpTimingAccess;
using g1_kino_detail::SolverProxDDP;

// 函数说明：执行完整 NMPC 求解流程，包括输入检查、warm start、cache 更新和 ProxDDP run。
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
    const std::vector<std::array<bool, kNumFeet>> contactModes = contactModeHorizon(input);

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
    bool resetDualsForContactSwitch = false;
    try
    {
        if (!ensureSolverCache(input, x0, xRef, uRef, solverSetupTime, rebuiltSolverCache))
        {
            status_ = -2;
            return false;
        }
        if (resetDualOnContactSwitch && solverCache_)
        {
            // 说明：接触预览表会随 horizon 滚动自然变化，只在当前 knot 的真实接触切换时重置 dual。
            resetDualsForContactSwitch =
                !solverCache_->lastContactModeHorizon.empty() &&
                !contactModes.empty() &&
                solverCache_->lastContactModeHorizon.front() != contactModes.front();
            solverCache_->lastContactModeHorizon = contactModes;
        }

        std::vector<Eigen::VectorXd> vsInit;
        std::vector<Eigen::VectorXd> lamsInit;
        if (resetDualsForContactSwitch)
        {
            vsInit = solverCache_->solver->results_.vs;
            lamsInit = solverCache_->solver->results_.lams;
            for (Eigen::VectorXd &v : vsInit)
            {
                v.setZero();
            }
            for (Eigen::VectorXd &lambda : lamsInit)
            {
                lambda.setZero();
            }
        }
        const auto solverRunStart = std::chrono::steady_clock::now();
        solverCache_->solver->run(*solverCache_->problem, xsInit, usInit, vsInit, lamsInit);
        solverRunTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - solverRunStart).count();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Kino-NMPC] solve exception: " << e.what() << std::endl;
        resetWarmStart();
        solverCache_.reset();
        status_ = -2;
        return false;
    }
    catch (...)
    {
        std::cerr << "[Kino-NMPC] solve exception: unknown" << std::endl;
        resetWarmStart();
        solverCache_.reset();
        status_ = -2;
        return false;
    }

    SolverProxDDP &solver = *solverCache_->solver;
    iterations_ = static_cast<int>(solver.results_.num_iters);
    const auto tEnd = std::chrono::steady_clock::now();
    solveTime_ = std::chrono::duration<double>(tEnd - tStart).count();
    const double derivativeTime = ProxDdpTimingAccess<SolverProxDDP>::derivatives(solver);
    const double ddpTime = ProxDdpTimingAccess<SolverProxDDP>::ddp(solver);
    const double evaluateTime = ProxDdpTimingAccess<SolverProxDDP>::evaluate(solver);
    const double multipliersTime = ProxDdpTimingAccess<SolverProxDDP>::multipliers(solver);
    const double meritTime = ProxDdpTimingAccess<SolverProxDDP>::merit(solver);
    const double lagrangianTime = ProxDdpTimingAccess<SolverProxDDP>::lagrangian(solver);
    const double criterionTime = ProxDdpTimingAccess<SolverProxDDP>::criterion(solver);
    const double projectedJacobianTime = ProxDdpTimingAccess<SolverProxDDP>::projectedJacobians(solver);
    const double lqUpdateTime = ProxDdpTimingAccess<SolverProxDDP>::lqUpdate(solver);
    const double lqBackwardTime = ProxDdpTimingAccess<SolverProxDDP>::lqBackward(solver);
    const double lqForwardTime = ProxDdpTimingAccess<SolverProxDDP>::lqForward(solver);
    const double feedbackTime = ProxDdpTimingAccess<SolverProxDDP>::feedback(solver);
    const double lineSearchTime = ProxDdpTimingAccess<SolverProxDDP>::lineSearch(solver);
    const double forwardPassTime = ProxDdpTimingAccess<SolverProxDDP>::forwardPass(solver);
    const double rolloutTime = ProxDdpTimingAccess<SolverProxDDP>::rollout(solver);
    const std::size_t forwardPassCount = ProxDdpTimingAccess<SolverProxDDP>::forwardPassCount(solver);
    const double solverTotalTime = solverSetupTime + solverRunTime;
    const double runOtherTime = std::max(0.0, solverRunTime - derivativeTime - ddpTime);
    const double wrapperOtherTime = std::max(0.0, solveTime_ - solverTotalTime);

    if (logTiming)
    {
        std::ostringstream timingOut;
        timingOut << "[Kino-NMPC] prox-ddp timing"
                  << " iter=" << iterations_
                  << " total_ms=" << std::fixed << std::setprecision(3) << solverTotalTime * 1e3
                  << " setup_ms=" << solverSetupTime * 1e3
                  << " run_ms=" << solverRunTime * 1e3
                  << " eval_ms=" << evaluateTime * 1e3
                  << " derivatives_ms=" << derivativeTime * 1e3
                  << " lagrangian_ms=" << lagrangianTime * 1e3
                  << " criterion_ms=" << criterionTime * 1e3
                  << " proj_jac_ms=" << projectedJacobianTime * 1e3
                  << " ddp_ms=" << ddpTime * 1e3
                  << " lq_update_ms=" << lqUpdateTime * 1e3
                  << " lq_backward_ms=" << lqBackwardTime * 1e3
                  << " lq_forward_ms=" << lqForwardTime * 1e3
                  << " feedback_ms=" << feedbackTime * 1e3
                  << " line_search_ms=" << lineSearchTime * 1e3
                  << " forward_pass_ms=" << forwardPassTime * 1e3
                  << " rollout_ms=" << rolloutTime * 1e3
                  << " multipliers_ms=" << multipliersTime * 1e3
                  << " merit_ms=" << meritTime * 1e3
                  << " forward_pass_count=" << forwardPassCount
                  << " run_other_ms=" << runOtherTime * 1e3
                  << " wrapper_other_ms=" << wrapperOtherTime * 1e3
                  << " cache=" << (rebuiltSolverCache ? "rebuild" : "reuse")
                  << " rollout=" << (useLinearRollout || useParallelLq ? "linear" : "nonlinear")
                  << " lq_solver=" << (useParallelLq ? "parallel" : "serial")
                  << " threads=" << std::max(1, numThreads)
                  << " contact_dual_reset=" << std::boolalpha << resetDualsForContactSwitch
                  << " conv=" << std::boolalpha << solver.results_.conv;
        std::cout << timingOut.str() << std::endl;
    }

    if (solver.results_.us.empty() || solver.results_.xs.empty())
    {
        status_ = -3;
        return false;
    }

    status_ = solver.results_.conv ? 0 : 1;
    updateOutputs(solver.results_.xs, solver.results_.us);
    return firstControl_.allFinite() && firstState_.allFinite();
}

// 函数说明：从求解结果提取第一个控制、下一状态，并保存整条轨迹供下次 warm start。
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

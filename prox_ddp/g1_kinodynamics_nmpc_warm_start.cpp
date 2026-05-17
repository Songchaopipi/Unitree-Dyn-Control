#include "g1_kinodynamics_nmpc.h"

#include "detail/direct_kino_costs.hpp"
#include "detail/solver_cache.hpp"

#include <algorithm>

using g1_kino_detail::safeVectorOrZero;

// 函数说明：清空求解器轨迹 warm start，下一次 solve 会从名义轨迹重建。
void G1KinodynamicsNmpc::resetWarmStart()
{
    xsWarm_.clear();
    usWarm_.clear();
}

// 函数说明：将 q/v 拼成 Aligator 使用的 [q; v] 状态。
Eigen::VectorXd G1KinodynamicsNmpc::makeState(const Input &input) const
{
    Eigen::VectorXd x(model_.nq + model_.nv);
    x.head(model_.nq) = safeVectorOrZero(input.q, model_.nq);
    x.tail(model_.nv) = safeVectorOrZero(input.v, model_.nv);
    return x;
}

// 函数说明：将 qReference/vReference 拼成 [q_ref; v_ref]。
Eigen::VectorXd G1KinodynamicsNmpc::makeStateReference(const Input &input) const
{
    Eigen::VectorXd xref(model_.nq + model_.nv);
    xref.head(model_.nq) = safeVectorOrZero(input.qReference, model_.nq);
    xref.tail(model_.nv) = safeVectorOrZero(input.vReference, model_.nv);
    return xref;
}

// 函数说明：返回第 0 个 knot 的名义控制。
Eigen::VectorXd G1KinodynamicsNmpc::nominalControl(const Input &input) const
{
    return nominalControlAt(input, 0);
}

// 函数说明：按 knot 的接触模式生成名义 wrench 和关节加速度。
Eigen::VectorXd G1KinodynamicsNmpc::nominalControlAt(const Input &input, int knot) const
{
    Eigen::VectorXd u = Eigen::VectorXd::Zero(nu_);
    const std::array<bool, kNumFeet> active = contactActiveAt(input, knot);
    int activeContacts = 0;
    double totalReferenceFz = 0.0;
    for (int foot = 0; foot < kNumFeet; ++foot)
    {
        activeContacts += active[foot] ? 1 : 0;
        if (input.wrenchReference[foot].allFinite())
        {
            totalReferenceFz += std::max(0.0, input.wrenchReference[foot](2));
        }
    }
    const double nominalFz = activeContacts > 0 && totalReferenceFz > 1e-9 ?
                                 totalReferenceFz / static_cast<double>(activeContacts) :
                                 0.0;
    for (int foot = 0; foot < kNumFeet; ++foot)
    {
        if (active[foot])
        {
            Eigen::Matrix<double, kForceSize, 1> wrench = input.wrenchReference[foot];
            if (!wrench.allFinite())
            {
                wrench.setZero();
            }
            if (nominalFz > 0.0)
            {
                wrench(2) = nominalFz;
            }
            u.segment<kForceSize>(foot * kForceSize) = wrench;
        }
    }
    return u;
}

// 函数说明：生成整个 horizon 的名义控制，便于 cost reference 和 warm start 使用。
std::vector<Eigen::VectorXd> G1KinodynamicsNmpc::nominalControlHorizon(const Input &input) const
{
    std::vector<Eigen::VectorXd> controls;
    controls.reserve(static_cast<std::size_t>(horizon_));
    for (int k = 0; k < horizon_; ++k)
    {
        controls.push_back(nominalControlAt(input, k));
    }
    return controls;
}

// 函数说明：优先查询 contactTable；如果为空，则退回当前 contactActive。
std::array<bool, G1KinodynamicsNmpc::kNumFeet>
G1KinodynamicsNmpc::contactActiveAt(const Input &input, int knot) const
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

// 函数说明：把 horizon 内接触模式打包出来，只用于判断是否需要重置 dual warm start。
std::vector<std::array<bool, G1KinodynamicsNmpc::kNumFeet>>
G1KinodynamicsNmpc::contactModeHorizon(const Input &input) const
{
    std::vector<std::array<bool, kNumFeet>> modes;
    modes.reserve(static_cast<std::size_t>(horizon_));
    for (int k = 0; k < horizon_; ++k)
    {
        modes.push_back(contactActiveAt(input, k));
    }
    return modes;
}

// 函数说明：以当前状态常值轨迹和名义控制初始化 warm start。
void G1KinodynamicsNmpc::initializeWarmStart(const Input &input)
{
    const Eigen::VectorXd x0 = makeState(input);
    xsWarm_.assign(static_cast<std::size_t>(horizon_ + 1), x0);
    usWarm_ = nominalControlHorizon(input);
}

// 函数说明：复用上一轮求解轨迹，向前平移一格并修正接触 wrench。
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
    usWarm_.back() = nominalControlAt(input, horizon_ - 1);

    const std::vector<Eigen::VectorXd> uNomHorizon = nominalControlHorizon(input);
    for (int k = 0; k < horizon_; ++k)
    {
        const std::array<bool, kNumFeet> active = contactActiveAt(input, k);
        const Eigen::VectorXd &uNom = uNomHorizon[static_cast<std::size_t>(k)];
        for (int foot = 0; foot < kNumFeet; ++foot)
        {
            const int offset = foot * kForceSize;
            if (!active[foot])
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

// 函数说明：检查 solver cache 的核心指针是否有效。
bool G1KinodynamicsNmpc::collectSolverCacheReferences()
{
    return solverCache_ && solverCache_->problem && solverCache_->references;
}

// 函数说明：problem 结构不变时只更新 x0、状态/控制参考和足端 horizon 参考。
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
    solverCache_->references->uRefHorizon = nominalControlHorizon(input);
    solverCache_->references->footPoseReference = input.footPoseReference;
    solverCache_->references->footPoseReferenceHorizon = input.footPoseReferenceHorizon;

    for (int k = 0; k < horizon_; ++k)
    {
        const std::array<bool, kNumFeet> active = contactActiveAt(input, k);
        for (int foot = 0; foot < kNumFeet; ++foot)
        {
            if (static_cast<int>(solverCache_->contactActivations.size()) > k &&
                solverCache_->contactActivations[static_cast<std::size_t>(k)][foot])
            {
                solverCache_->contactActivations[static_cast<std::size_t>(k)][foot]->active = active[foot];
            }

            if (static_cast<int>(solverCache_->wrenchBounds.size()) > k &&
                solverCache_->wrenchBounds[static_cast<std::size_t>(k)][foot])
            {
                auto &bounds = *solverCache_->wrenchBounds[static_cast<std::size_t>(k)][foot];
                if (active[foot])
                {
                    bounds.lower << -forceLimit, -forceLimit, 0.0, -momentLimit, -momentLimit, -momentLimit;
                    bounds.upper << forceLimit, forceLimit, forceLimit, momentLimit, momentLimit, momentLimit;
                }
                else
                {
                    bounds.lower.setZero();
                    bounds.upper.setZero();
                }
            }
        }
    }
}

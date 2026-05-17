#pragma once

#include <cstddef>
#include <type_traits>
#include <utility>

namespace g1_kino_detail
{

// 模板说明：老版本 Aligator 没有公开 timing 字段时返回 0。
template <typename Solver, typename = void>
struct ProxDdpTimingAccess
{
    static double derivatives(const Solver &) { return 0.0; }
    static double ddp(const Solver &) { return 0.0; }
    static double evaluate(const Solver &) { return 0.0; }
    static double multipliers(const Solver &) { return 0.0; }
    static double merit(const Solver &) { return 0.0; }
    static double lagrangian(const Solver &) { return 0.0; }
    static double criterion(const Solver &) { return 0.0; }
    static double projectedJacobians(const Solver &) { return 0.0; }
    static double lqUpdate(const Solver &) { return 0.0; }
    static double lqBackward(const Solver &) { return 0.0; }
    static double lqForward(const Solver &) { return 0.0; }
    static double feedback(const Solver &) { return 0.0; }
    static double lineSearch(const Solver &) { return 0.0; }
    static double forwardPass(const Solver &) { return 0.0; }
    static double rollout(const Solver &) { return 0.0; }
    static std::size_t forwardPassCount(const Solver &) { return 0; }
};

// 模板说明：如果 Solver 暴露 timing 字段，则读取真实分项耗时。
template <typename Solver>
struct ProxDdpTimingAccess<
    Solver,
    std::void_t<decltype(std::declval<const Solver &>().derivatives_time_),
                decltype(std::declval<const Solver &>().ddp_time_),
                decltype(std::declval<const Solver &>().line_search_time_),
                decltype(std::declval<const Solver &>().forward_pass_count_)>>
{
    static double derivatives(const Solver &solver)
    {
        return static_cast<double>(solver.derivatives_time_);
    }

    static double ddp(const Solver &solver)
    {
        return static_cast<double>(solver.ddp_time_);
    }

    static double evaluate(const Solver &solver)
    {
        return static_cast<double>(solver.evaluate_time_);
    }

    static double multipliers(const Solver &solver)
    {
        return static_cast<double>(solver.multipliers_time_);
    }

    static double merit(const Solver &solver)
    {
        return static_cast<double>(solver.merit_time_);
    }

    static double lagrangian(const Solver &solver)
    {
        return static_cast<double>(solver.lagrangian_time_);
    }

    static double criterion(const Solver &solver)
    {
        return static_cast<double>(solver.criterion_time_);
    }

    static double projectedJacobians(const Solver &solver)
    {
        return static_cast<double>(solver.projected_jacobians_time_);
    }

    static double lqUpdate(const Solver &solver)
    {
        return static_cast<double>(solver.lq_update_time_);
    }

    static double lqBackward(const Solver &solver)
    {
        return static_cast<double>(solver.lq_backward_time_);
    }

    static double lqForward(const Solver &solver)
    {
        return static_cast<double>(solver.lq_forward_time_);
    }

    static double feedback(const Solver &solver)
    {
        return static_cast<double>(solver.feedback_time_);
    }

    static double lineSearch(const Solver &solver)
    {
        return static_cast<double>(solver.line_search_time_);
    }

    static double forwardPass(const Solver &solver)
    {
        return static_cast<double>(solver.forward_pass_time_);
    }

    static double rollout(const Solver &solver)
    {
        return static_cast<double>(solver.rollout_time_);
    }

    static std::size_t forwardPassCount(const Solver &solver)
    {
        return static_cast<std::size_t>(solver.forward_pass_count_);
    }
};

} // namespace g1_kino_detail

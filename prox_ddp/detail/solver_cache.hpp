#pragma once

#include "../g1_kinodynamics_nmpc.h"
#include "direct_kino_costs.hpp"
#include "g1_kino_types.hpp"
#include "mutable_contact_constraints.hpp"

#include <array>
#include <memory>
#include <vector>

// 缓存说明：保存当前 problem/solver/reference，并用 Signature 判断是否可以复用。
struct G1KinodynamicsNmpc::SolverCache
{
    // 签名说明：凡是会改变 stage/dynamics/constraint 结构的量都必须放进签名。
    struct Signature
    {
        int horizon{0};
        double dt{0.0};
        int nq{0};
        int nv{0};
        int nu{0};
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
        double lineSearchAlphaMin{0.0};
        int maxLineSearchSteps{0};
        int numThreads{1};
        bool useLinearRollout{false};
        bool useParallelLq{false};

        // 函数说明：判断当前缓存问题是否和新输入结构完全一致。
        bool operator==(const Signature &rhs) const
        {
            return horizon == rhs.horizon &&
                   dt == rhs.dt &&
                   nq == rhs.nq &&
                   nv == rhs.nv &&
                   nu == rhs.nu &&
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
                   maxAlIterations == rhs.maxAlIterations &&
                   lineSearchAlphaMin == rhs.lineSearchAlphaMin &&
                   maxLineSearchSteps == rhs.maxLineSearchSteps &&
                   numThreads == rhs.numThreads &&
                   useLinearRollout == rhs.useLinearRollout &&
                   useParallelLq == rhs.useParallelLq;
        }

        // 函数说明：operator== 的反向判断。
        bool operator!=(const Signature &rhs) const { return !(*this == rhs); }
    };

    Signature signature;
    std::unique_ptr<g1_kino_detail::TrajOptProblem> problem;
    std::unique_ptr<g1_kino_detail::SolverProxDDP> solver;
    std::shared_ptr<g1_kino_detail::DirectKinoCostReferences> references;
    std::vector<std::array<std::shared_ptr<g1_kino_detail::ContactActivation>, kNumFeet>> contactActivations;
    std::vector<std::array<std::shared_ptr<g1_kino_detail::MutableBoxBounds>, kNumFeet>> wrenchBounds;
    std::vector<std::array<bool, kNumFeet>> lastContactModeHorizon;
};

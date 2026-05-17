#pragma once

#include <Eigen/Dense>

#include <aligator/core/cost-abstract.hpp>
#include <aligator/core/explicit-dynamics.hpp>
#include <aligator/core/function-abstract.hpp>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/dynamics/kinodynamics-fwd.hpp>
#include <aligator/modelling/multibody/centroidal-momentum.hpp>
#include <aligator/modelling/multibody/centroidal-momentum-derivative.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>

namespace g1_kino_detail
{

// 类型别名说明：把 Aligator/Pinocchio 的长模板类型收束到一个命名空间里。
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

} // namespace g1_kino_detail

#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/ConstraintOrder.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include <humanoid_cd_nmpc_aligator/direct_stage_cost_pack.hpp>
#include <humanoid_cd_nmpc_aligator/packed_cost_accumulator.hpp>

namespace humanoid_cd_nmpc_aligator {

struct DirectSoftPenaltySettings {
  RelaxedBarrierSettings barrier;
  bool includeExactConstraintHessian{false};
};

class Ocs2StateInputSoftPenaltyTerm final : public DirectStageCostTerm {
 public:
  Ocs2StateInputSoftPenaltyTerm(std::string name, std::unique_ptr<ocs2::StateInputConstraint> constraint,
                                DirectSoftPenaltySettings settings)
      : name_(std::move(name)), constraint_(std::move(constraint)), settings_(settings) {
    if (!constraint_) {
      throw std::invalid_argument("Ocs2StateInputSoftPenaltyTerm requires a valid constraint");
    }
  }

  std::string name() const override { return name_; }

  void add(const DirectStageCostContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
           PackedCostWorkspace& out) const override {
    if (!context.preComputation) {
      throw std::invalid_argument("Ocs2StateInputSoftPenaltyTerm requires pre-computation");
    }
    if (!constraint_->isActive(context.time)) {
      return;
    }

    if (constraint_->getOrder() == ocs2::ConstraintOrder::Linear) {
      const ocs2::VectorFunctionLinearApproximation approximation =
          constraint_->getLinearApproximation(context.time, x, u, *context.preComputation);
      addLinearPenalty(context.dt, approximation.f, approximation.dfdx, approximation.dfdu, out);
    } else {
      const ocs2::VectorFunctionQuadraticApproximation approximation =
          constraint_->getQuadraticApproximation(context.time, x, u, *context.preComputation);
      addQuadraticPenalty(context.dt, approximation, out);
    }
  }

 private:
  void addLinearPenalty(double scale, const Eigen::VectorXd& c, const Eigen::MatrixXd& Cx, const Eigen::MatrixXd& Cu,
                        PackedCostWorkspace& out) const {
    for (Eigen::Index i = 0; i < c.size(); ++i) {
      ScalarPenaltyApproximation penalty = relaxedUpperBoundPenalty(c(i), settings_.barrier);
      penalty.value *= scale;
      penalty.first *= scale;
      penalty.second *= scale;
      addScalarPenaltyGaussNewton(out, c(i), Cx.row(i).transpose(), Cu.row(i).transpose(), penalty);
    }
  }

  void addQuadraticPenalty(double scale, const ocs2::VectorFunctionQuadraticApproximation& approximation,
                           PackedCostWorkspace& out) const {
    for (Eigen::Index i = 0; i < approximation.f.size(); ++i) {
      ScalarPenaltyApproximation penalty = relaxedUpperBoundPenalty(approximation.f(i), settings_.barrier);
      penalty.value *= scale;
      penalty.first *= scale;
      penalty.second *= scale;
      const Eigen::VectorXd cx = approximation.dfdx.row(i).transpose();
      const Eigen::VectorXd cu = approximation.dfdu.row(i).transpose();
      addScalarPenaltyGaussNewton(out, approximation.f(i), cx, cu, penalty);
      if (settings_.includeExactConstraintHessian) {
        out.Lxx.noalias() += penalty.first * approximation.dfdxx[static_cast<std::size_t>(i)];
        out.Lux.noalias() += penalty.first * approximation.dfdux[static_cast<std::size_t>(i)];
        out.Lxu.noalias() += penalty.first * approximation.dfdux[static_cast<std::size_t>(i)].transpose();
        out.Luu.noalias() += penalty.first * approximation.dfduu[static_cast<std::size_t>(i)];
      }
    }
  }

  std::string name_;
  std::unique_ptr<ocs2::StateInputConstraint> constraint_;
  DirectSoftPenaltySettings settings_;
};

class Ocs2StateSoftPenaltyTerm final : public DirectStageCostTerm {
 public:
  Ocs2StateSoftPenaltyTerm(std::string name, std::unique_ptr<ocs2::StateConstraint> constraint,
                           DirectSoftPenaltySettings settings)
      : name_(std::move(name)), constraint_(std::move(constraint)), settings_(settings) {
    if (!constraint_) {
      throw std::invalid_argument("Ocs2StateSoftPenaltyTerm requires a valid constraint");
    }
  }

  std::string name() const override { return name_; }

  void add(const DirectStageCostContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
           PackedCostWorkspace& out) const override {
    (void)u;
    if (!context.preComputation) {
      throw std::invalid_argument("Ocs2StateSoftPenaltyTerm requires pre-computation");
    }
    if (!constraint_->isActive(context.time)) {
      return;
    }

    if (constraint_->getOrder() == ocs2::ConstraintOrder::Linear) {
      const ocs2::VectorFunctionLinearApproximation approximation =
          constraint_->getLinearApproximation(context.time, x, *context.preComputation);
      addLinearPenalty(context.dt, approximation.f, approximation.dfdx, out);
    } else {
      const ocs2::VectorFunctionQuadraticApproximation approximation =
          constraint_->getQuadraticApproximation(context.time, x, *context.preComputation);
      addQuadraticPenalty(context.dt, approximation, out);
    }
  }

 private:
  void addLinearPenalty(double scale, const Eigen::VectorXd& c, const Eigen::MatrixXd& Cx,
                        PackedCostWorkspace& out) const {
    const Eigen::VectorXd zeroU = Eigen::VectorXd::Zero(out.Lu.size());
    for (Eigen::Index i = 0; i < c.size(); ++i) {
      ScalarPenaltyApproximation penalty = relaxedUpperBoundPenalty(c(i), settings_.barrier);
      penalty.value *= scale;
      penalty.first *= scale;
      penalty.second *= scale;
      addScalarPenaltyGaussNewton(out, c(i), Cx.row(i).transpose(), zeroU, penalty);
    }
  }

  void addQuadraticPenalty(double scale, const ocs2::VectorFunctionQuadraticApproximation& approximation,
                           PackedCostWorkspace& out) const {
    const Eigen::VectorXd zeroU = Eigen::VectorXd::Zero(out.Lu.size());
    for (Eigen::Index i = 0; i < approximation.f.size(); ++i) {
      ScalarPenaltyApproximation penalty = relaxedUpperBoundPenalty(approximation.f(i), settings_.barrier);
      penalty.value *= scale;
      penalty.first *= scale;
      penalty.second *= scale;
      const Eigen::VectorXd cx = approximation.dfdx.row(i).transpose();
      addScalarPenaltyGaussNewton(out, approximation.f(i), cx, zeroU, penalty);
      if (settings_.includeExactConstraintHessian) {
        out.Lxx.noalias() += penalty.first * approximation.dfdxx[static_cast<std::size_t>(i)];
      }
    }
  }

  std::string name_;
  std::unique_ptr<ocs2::StateConstraint> constraint_;
  DirectSoftPenaltySettings settings_;
};

}  // namespace humanoid_cd_nmpc_aligator

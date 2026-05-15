#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include <humanoid_cd_nmpc_aligator/packed_cost_accumulator.hpp>

namespace humanoid_cd_nmpc_aligator {

struct DirectConstraintContext {
  double time{0.0};
  const ocs2::PreComputation* preComputation{nullptr};
};

class DirectConstraintTerm {
 public:
  virtual ~DirectConstraintTerm() = default;
  virtual std::string name() const = 0;
  virtual int rows(double time) const = 0;
  virtual void write(const DirectConstraintContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                     int rowOffset, PackedConstraintWorkspace& out) const = 0;
};

class Ocs2StateInputEqualityConstraintTerm final : public DirectConstraintTerm {
 public:
  Ocs2StateInputEqualityConstraintTerm(std::string name, std::unique_ptr<ocs2::StateInputConstraint> constraint)
      : name_(std::move(name)), constraint_(std::move(constraint)) {
    if (!constraint_) {
      throw std::invalid_argument("Ocs2StateInputEqualityConstraintTerm requires a valid constraint");
    }
  }

  std::string name() const override { return name_; }

  int rows(double time) const override { return static_cast<int>(constraint_->getNumConstraints(time)); }

  void write(const DirectConstraintContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
             int rowOffset, PackedConstraintWorkspace& out) const override {
    if (!context.preComputation) {
      throw std::invalid_argument("Ocs2StateInputEqualityConstraintTerm requires pre-computation");
    }

    const int n = rows(context.time);
    if (!constraint_->isActive(context.time) || n == 0) {
      out.activeRows.segment(rowOffset, n).setZero();
      return;
    }

    const ocs2::VectorFunctionLinearApproximation approximation =
        constraint_->getLinearApproximation(context.time, x, u, *context.preComputation);
    out.c.segment(rowOffset, n) = approximation.f;
    out.Cx.middleRows(rowOffset, n) = approximation.dfdx;
    out.Cu.middleRows(rowOffset, n) = approximation.dfdu;
    out.activeRows.segment(rowOffset, n).setOnes();
  }

 private:
  std::string name_;
  std::unique_ptr<ocs2::StateInputConstraint> constraint_;
};

class Ocs2StateEqualityConstraintTerm final : public DirectConstraintTerm {
 public:
  Ocs2StateEqualityConstraintTerm(std::string name, std::unique_ptr<ocs2::StateConstraint> constraint)
      : name_(std::move(name)), constraint_(std::move(constraint)) {
    if (!constraint_) {
      throw std::invalid_argument("Ocs2StateEqualityConstraintTerm requires a valid constraint");
    }
  }

  std::string name() const override { return name_; }

  int rows(double time) const override { return static_cast<int>(constraint_->getNumConstraints(time)); }

  void write(const DirectConstraintContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd&,
             int rowOffset, PackedConstraintWorkspace& out) const override {
    if (!context.preComputation) {
      throw std::invalid_argument("Ocs2StateEqualityConstraintTerm requires pre-computation");
    }

    const int n = rows(context.time);
    if (!constraint_->isActive(context.time) || n == 0) {
      out.activeRows.segment(rowOffset, n).setZero();
      return;
    }

    const ocs2::VectorFunctionLinearApproximation approximation =
        constraint_->getLinearApproximation(context.time, x, *context.preComputation);
    out.c.segment(rowOffset, n) = approximation.f;
    out.Cx.middleRows(rowOffset, n) = approximation.dfdx;
    out.Cu.middleRows(rowOffset, n).setZero();
    out.activeRows.segment(rowOffset, n).setOnes();
  }

 private:
  std::string name_;
  std::unique_ptr<ocs2::StateConstraint> constraint_;
};

class DirectConstraintPack {
 public:
  void addTerm(std::unique_ptr<DirectConstraintTerm> term) {
    if (!term) {
      throw std::invalid_argument("DirectConstraintPack::addTerm received null term");
    }
    terms_.push_back(std::move(term));
  }

  int rows(double time) const {
    int total = 0;
    for (const auto& term : terms_) {
      total += term->rows(time);
    }
    return total;
  }

  void evaluate(const DirectConstraintContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                PackedConstraintWorkspace& out) const {
    out.setZero();
    int row = 0;
    for (const auto& term : terms_) {
      term->write(context, x, u, row, out);
      row += term->rows(context.time);
    }
  }

  std::size_t size() const { return terms_.size(); }

 private:
  std::vector<std::unique_ptr<DirectConstraintTerm>> terms_;
};

}  // namespace humanoid_cd_nmpc_aligator

#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/constraint/StateConstraintCollection.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/constraint/StateInputConstraintCollection.h>

#include <aligator/core/function-abstract.hpp>

#include <humanoid_cd_nmpc_aligator/packed_cost_accumulator.hpp>

namespace humanoid_cd_nmpc_aligator {

struct DirectConstraintContext {
  double time{0.0};
  ocs2::PreComputation* preComputation{nullptr};
};

class DirectConstraintTerm {
 public:
  virtual ~DirectConstraintTerm() = default;
  virtual std::string name() const = 0;
  virtual std::unique_ptr<DirectConstraintTerm> clone() const = 0;
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

  Ocs2StateInputEqualityConstraintTerm(const Ocs2StateInputEqualityConstraintTerm& rhs)
      : name_(rhs.name_), constraint_(rhs.constraint_ ? rhs.constraint_->clone() : nullptr) {}

  std::string name() const override { return name_; }

  std::unique_ptr<DirectConstraintTerm> clone() const override {
    return std::make_unique<Ocs2StateInputEqualityConstraintTerm>(*this);
  }

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

class Ocs2StateInputEqualityConstraintCollectionTerm final : public DirectConstraintTerm {
 public:
  Ocs2StateInputEqualityConstraintCollectionTerm(std::string name,
                                                 std::unique_ptr<ocs2::StateInputConstraintCollection> constraints)
      : name_(std::move(name)), constraints_(std::move(constraints)) {
    if (!constraints_) {
      throw std::invalid_argument("Ocs2StateInputEqualityConstraintCollectionTerm requires a valid collection");
    }
  }

  Ocs2StateInputEqualityConstraintCollectionTerm(const Ocs2StateInputEqualityConstraintCollectionTerm& rhs)
      : name_(rhs.name_), constraints_(rhs.constraints_ ? rhs.constraints_->clone() : nullptr) {}

  std::string name() const override { return name_; }

  std::unique_ptr<DirectConstraintTerm> clone() const override {
    return std::make_unique<Ocs2StateInputEqualityConstraintCollectionTerm>(*this);
  }

  int rows(double time) const override { return static_cast<int>(constraints_->getNumConstraints(time)); }

  void write(const DirectConstraintContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
             int rowOffset, PackedConstraintWorkspace& out) const override {
    if (!context.preComputation) {
      throw std::invalid_argument("Ocs2StateInputEqualityConstraintCollectionTerm requires pre-computation");
    }

    const int n = rows(context.time);
    if (n == 0) {
      return;
    }

    const ocs2::VectorFunctionLinearApproximation approximation =
        constraints_->getLinearApproximation(context.time, x, u, *context.preComputation);
    out.c.segment(rowOffset, n) = approximation.f;
    out.Cx.middleRows(rowOffset, n) = approximation.dfdx;
    out.Cu.middleRows(rowOffset, n) = approximation.dfdu;
    out.activeRows.segment(rowOffset, n).setOnes();
  }

 private:
  std::string name_;
  std::unique_ptr<ocs2::StateInputConstraintCollection> constraints_;
};

class Ocs2StateEqualityConstraintTerm final : public DirectConstraintTerm {
 public:
  Ocs2StateEqualityConstraintTerm(std::string name, std::unique_ptr<ocs2::StateConstraint> constraint)
      : name_(std::move(name)), constraint_(std::move(constraint)) {
    if (!constraint_) {
      throw std::invalid_argument("Ocs2StateEqualityConstraintTerm requires a valid constraint");
    }
  }

  Ocs2StateEqualityConstraintTerm(const Ocs2StateEqualityConstraintTerm& rhs)
      : name_(rhs.name_), constraint_(rhs.constraint_ ? rhs.constraint_->clone() : nullptr) {}

  std::string name() const override { return name_; }

  std::unique_ptr<DirectConstraintTerm> clone() const override {
    return std::make_unique<Ocs2StateEqualityConstraintTerm>(*this);
  }

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

class Ocs2StateEqualityConstraintCollectionTerm final : public DirectConstraintTerm {
 public:
  Ocs2StateEqualityConstraintCollectionTerm(std::string name, std::unique_ptr<ocs2::StateConstraintCollection> constraints)
      : name_(std::move(name)), constraints_(std::move(constraints)) {
    if (!constraints_) {
      throw std::invalid_argument("Ocs2StateEqualityConstraintCollectionTerm requires a valid collection");
    }
  }

  Ocs2StateEqualityConstraintCollectionTerm(const Ocs2StateEqualityConstraintCollectionTerm& rhs)
      : name_(rhs.name_), constraints_(rhs.constraints_ ? rhs.constraints_->clone() : nullptr) {}

  std::string name() const override { return name_; }

  std::unique_ptr<DirectConstraintTerm> clone() const override {
    return std::make_unique<Ocs2StateEqualityConstraintCollectionTerm>(*this);
  }

  int rows(double time) const override { return static_cast<int>(constraints_->getNumConstraints(time)); }

  void write(const DirectConstraintContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd&,
             int rowOffset, PackedConstraintWorkspace& out) const override {
    if (!context.preComputation) {
      throw std::invalid_argument("Ocs2StateEqualityConstraintCollectionTerm requires pre-computation");
    }

    const int n = rows(context.time);
    if (n == 0) {
      return;
    }

    const ocs2::VectorFunctionLinearApproximation approximation =
        constraints_->getLinearApproximation(context.time, x, *context.preComputation);
    out.c.segment(rowOffset, n) = approximation.f;
    out.Cx.middleRows(rowOffset, n) = approximation.dfdx;
    out.Cu.middleRows(rowOffset, n).setZero();
    out.activeRows.segment(rowOffset, n).setOnes();
  }

 private:
  std::string name_;
  std::unique_ptr<ocs2::StateConstraintCollection> constraints_;
};

class DirectConstraintPack {
 public:
  DirectConstraintPack() = default;

  DirectConstraintPack(const DirectConstraintPack& rhs) {
    terms_.reserve(rhs.terms_.size());
    for (const auto& term : rhs.terms_) {
      terms_.push_back(term->clone());
    }
  }

  DirectConstraintPack& operator=(const DirectConstraintPack& rhs) {
    if (this == &rhs) {
      return *this;
    }
    DirectConstraintPack tmp(rhs);
    terms_ = std::move(tmp.terms_);
    return *this;
  }

  DirectConstraintPack(DirectConstraintPack&&) noexcept = default;
  DirectConstraintPack& operator=(DirectConstraintPack&&) noexcept = default;

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

class DirectConstraintPackAligatorFunction final : public aligator::StageFunctionTpl<double> {
 public:
  using Base = aligator::StageFunctionTpl<double>;
  using Data = aligator::StageFunctionDataTpl<double>;

  DirectConstraintPackAligatorFunction(int ndx, int nu, DirectConstraintContext context, DirectConstraintPack pack)
      : Base(ndx, nu, pack.rows(context.time)), context_(std::move(context)), pack_(std::move(pack)) {
    workspace_.resize(this->nr, ndx, nu);
  }

  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    evaluatePack(x, u, data);
  }

  void computeJacobians(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    evaluatePack(x, u, data);
  }

  std::shared_ptr<Data> createData() const override { return std::make_shared<Data>(this->ndx1, this->nu, this->nr); }

 private:
  void evaluatePack(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const {
    const Eigen::VectorXd xVec = x;
    const Eigen::VectorXd uVec = u;
    if (context_.preComputation) {
      constexpr auto request = ocs2::Request::Constraint + ocs2::Request::Approximation;
      context_.preComputation->request(request, context_.time, xVec, uVec);
    }
    pack_.evaluate(context_, xVec, uVec, workspace_);
    data.value_ = workspace_.c;
    data.Jx_ = workspace_.Cx;
    data.Ju_ = workspace_.Cu;
  }

  DirectConstraintContext context_;
  DirectConstraintPack pack_;
  mutable PackedConstraintWorkspace workspace_;
};

}  // namespace humanoid_cd_nmpc_aligator

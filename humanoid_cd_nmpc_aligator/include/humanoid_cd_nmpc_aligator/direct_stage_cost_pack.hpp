#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <aligator/core/cost-abstract.hpp>
#include <aligator/core/vector-space.hpp>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include <humanoid_cd_nmpc_aligator/packed_cost_accumulator.hpp>

namespace humanoid_cd_nmpc_aligator {

struct DirectStageCostContext {
  double time{0.0};
  double dt{1.0};
  const ocs2::TargetTrajectories* targetTrajectories{nullptr};
  const ocs2::PreComputation* preComputation{nullptr};
};

class DirectStageCostTerm {
 public:
  virtual ~DirectStageCostTerm() = default;
  virtual std::string name() const = 0;
  virtual void add(const DirectStageCostContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                   PackedCostWorkspace& out) const = 0;
};

class DirectQuadraticTrackingTerm final : public DirectStageCostTerm {
 public:
  DirectQuadraticTrackingTerm(Eigen::VectorXd xRef, Eigen::VectorXd uRef, Eigen::MatrixXd Q, Eigen::MatrixXd R)
      : xRef_(std::move(xRef)), uRef_(std::move(uRef)), Q_(std::move(Q)), R_(std::move(R)) {}

  std::string name() const override { return "direct_quadratic_tracking"; }

  void add(const DirectStageCostContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
           PackedCostWorkspace& out) const override {
    addQuadraticStateTracking(out, x, xRef_, context.dt * Q_);
    addQuadraticInputTracking(out, u, uRef_, context.dt * R_);
  }

 private:
  Eigen::VectorXd xRef_;
  Eigen::VectorXd uRef_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
};

class DirectTerminalQuadraticTrackingTerm final : public DirectStageCostTerm {
 public:
  DirectTerminalQuadraticTrackingTerm(Eigen::VectorXd xRef, Eigen::MatrixXd Qf)
      : xRef_(std::move(xRef)), Qf_(std::move(Qf)) {}

  std::string name() const override { return "direct_terminal_quadratic_tracking"; }

  void add(const DirectStageCostContext&, const Eigen::VectorXd& x, const Eigen::VectorXd&,
           PackedCostWorkspace& out) const override {
    addQuadraticStateTracking(out, x, xRef_, Qf_);
  }

 private:
  Eigen::VectorXd xRef_;
  Eigen::MatrixXd Qf_;
};

class Ocs2StateInputCostTerm final : public DirectStageCostTerm {
 public:
  Ocs2StateInputCostTerm(std::string name, std::unique_ptr<ocs2::StateInputCost> cost)
      : name_(std::move(name)), cost_(std::move(cost)) {
    if (!cost_) {
      throw std::invalid_argument("Ocs2StateInputCostTerm requires a valid cost");
    }
  }

  std::string name() const override { return name_; }

  void add(const DirectStageCostContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
           PackedCostWorkspace& out) const override {
    if (!context.targetTrajectories || !context.preComputation) {
      throw std::invalid_argument("Ocs2StateInputCostTerm requires target trajectories and pre-computation");
    }
    if (!cost_->isActive(context.time)) {
      return;
    }

    const ocs2::ScalarFunctionQuadraticApproximation q =
        cost_->getQuadraticApproximation(context.time, x, u, *context.targetTrajectories, *context.preComputation);
    addOcs2QuadraticApproximation(out, context.dt, q);
  }

 private:
  static void addOcs2QuadraticApproximation(PackedCostWorkspace& out, double scale,
                                            const ocs2::ScalarFunctionQuadraticApproximation& q) {
    out.value += scale * q.f;
    out.Lx.noalias() += scale * q.dfdx;
    out.Lu.noalias() += scale * q.dfdu;
    out.Lxx.noalias() += scale * q.dfdxx;
    out.Lux.noalias() += scale * q.dfdux;
    out.Lxu.noalias() += scale * q.dfdux.transpose();
    out.Luu.noalias() += scale * q.dfduu;
  }

  std::string name_;
  std::unique_ptr<ocs2::StateInputCost> cost_;
};

class Ocs2StateCostTerm final : public DirectStageCostTerm {
 public:
  Ocs2StateCostTerm(std::string name, std::unique_ptr<ocs2::StateCost> cost, bool scaleByDt)
      : name_(std::move(name)), cost_(std::move(cost)), scaleByDt_(scaleByDt) {
    if (!cost_) {
      throw std::invalid_argument("Ocs2StateCostTerm requires a valid cost");
    }
  }

  std::string name() const override { return name_; }

  void add(const DirectStageCostContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd&,
           PackedCostWorkspace& out) const override {
    if (!context.targetTrajectories || !context.preComputation) {
      throw std::invalid_argument("Ocs2StateCostTerm requires target trajectories and pre-computation");
    }
    if (!cost_->isActive(context.time)) {
      return;
    }

    const double scale = scaleByDt_ ? context.dt : 1.0;
    const ocs2::ScalarFunctionQuadraticApproximation q =
        cost_->getQuadraticApproximation(context.time, x, *context.targetTrajectories, *context.preComputation);
    out.value += scale * q.f;
    out.Lx.noalias() += scale * q.dfdx;
    out.Lxx.noalias() += scale * q.dfdxx;
  }

 private:
  std::string name_;
  std::unique_ptr<ocs2::StateCost> cost_;
  bool scaleByDt_{true};
};

class DirectStageCostPack {
 public:
  void addTerm(std::unique_ptr<DirectStageCostTerm> term) {
    if (!term) {
      throw std::invalid_argument("DirectStageCostPack::addTerm received null term");
    }
    terms_.push_back(std::move(term));
  }

  void evaluate(const DirectStageCostContext& context, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                PackedCostWorkspace& out) const {
    out.setZero();
    for (const auto& term : terms_) {
      term->add(context, x, u, out);
    }
  }

  std::size_t size() const { return terms_.size(); }

 private:
  std::vector<std::unique_ptr<DirectStageCostTerm>> terms_;
};

class DirectStageCostPackAligatorCost final : public aligator::CostAbstractTpl<double> {
 public:
  using Base = aligator::CostAbstractTpl<double>;
  using CostData = aligator::CostDataAbstractTpl<double>;

  DirectStageCostPackAligatorCost(const aligator::VectorSpaceTpl<double>& space, int nu,
                                  DirectStageCostContext context, DirectStageCostPack pack)
      : Base(space, nu), context_(std::move(context)), pack_(std::move(pack)) {
    workspace_.resize(this->ndx(), this->nu);
  }

  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, CostData& data) const override {
    evaluatePack(x, u, data);
  }

  void computeGradients(const ConstVectorRef& x, const ConstVectorRef& u, CostData& data) const override {
    evaluatePack(x, u, data);
  }

  void computeHessians(const ConstVectorRef& x, const ConstVectorRef& u, CostData& data) const override {
    evaluatePack(x, u, data);
  }

  std::shared_ptr<CostData> createData() const override {
    return std::make_shared<CostData>(this->ndx(), this->nu);
  }

 private:
  void evaluatePack(const ConstVectorRef& x, const ConstVectorRef& u, CostData& data) const {
    const Eigen::VectorXd xVec = x;
    const Eigen::VectorXd uVec = u;
    pack_.evaluate(context_, xVec, uVec, workspace_);

    data.value_ = workspace_.value;
    data.Lx_ = workspace_.Lx;
    data.Lu_ = workspace_.Lu;
    data.Lxx_ = workspace_.Lxx;
    data.Lxu_ = workspace_.Lxu;
    data.Lux_ = workspace_.Lux;
    data.Luu_ = workspace_.Luu;
  }

  DirectStageCostContext context_;
  DirectStageCostPack pack_;
  mutable PackedCostWorkspace workspace_;
};

inline DirectStageCostPack makeLayer2AQuadraticRunningPack(const Eigen::VectorXd& xRef, const Eigen::VectorXd& uRef,
                                                           const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
  DirectStageCostPack pack;
  pack.addTerm(std::make_unique<DirectQuadraticTrackingTerm>(xRef, uRef, Q, R));
  return pack;
}

inline DirectStageCostPack makeLayer2ATerminalPack(const Eigen::VectorXd& xRef, const Eigen::MatrixXd& Qf) {
  DirectStageCostPack pack;
  pack.addTerm(std::make_unique<DirectTerminalQuadraticTrackingTerm>(xRef, Qf));
  return pack;
}

}  // namespace humanoid_cd_nmpc_aligator

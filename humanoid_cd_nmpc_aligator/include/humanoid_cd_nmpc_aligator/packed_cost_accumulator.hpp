#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace humanoid_cd_nmpc_aligator {

struct PackedCostWorkspace {
  double value{0.0};
  Eigen::VectorXd Lx;
  Eigen::VectorXd Lu;
  Eigen::MatrixXd Lxx;
  Eigen::MatrixXd Lxu;
  Eigen::MatrixXd Lux;
  Eigen::MatrixXd Luu;

  void resize(int nx, int nu) {
    if (nx <= 0 || nu < 0) {
      throw std::invalid_argument("PackedCostWorkspace::resize received invalid dimensions");
    }
    Lx.resize(nx);
    Lu.resize(nu);
    Lxx.resize(nx, nx);
    Lxu.resize(nx, nu);
    Lux.resize(nu, nx);
    Luu.resize(nu, nu);
    setZero();
  }

  void setZero() {
    value = 0.0;
    Lx.setZero();
    Lu.setZero();
    Lxx.setZero();
    Lxu.setZero();
    Lux.setZero();
    Luu.setZero();
  }
};

inline void addQuadraticStateTracking(PackedCostWorkspace& out, const Eigen::VectorXd& x,
                                      const Eigen::VectorXd& xRef, const Eigen::MatrixXd& Q) {
  if (x.size() != xRef.size() || Q.rows() != x.size() || Q.cols() != x.size() || out.Lx.size() != x.size()) {
    throw std::invalid_argument("addQuadraticStateTracking dimension mismatch");
  }
  const Eigen::VectorXd dx = x - xRef;
  const Eigen::VectorXd Qdx = Q * dx;
  out.value += 0.5 * dx.dot(Qdx);
  out.Lx.noalias() += Qdx;
  out.Lxx.noalias() += Q;
}

inline void addQuadraticInputTracking(PackedCostWorkspace& out, const Eigen::VectorXd& u,
                                      const Eigen::VectorXd& uRef, const Eigen::MatrixXd& R) {
  if (u.size() != uRef.size() || R.rows() != u.size() || R.cols() != u.size() || out.Lu.size() != u.size()) {
    throw std::invalid_argument("addQuadraticInputTracking dimension mismatch");
  }
  const Eigen::VectorXd du = u - uRef;
  const Eigen::VectorXd Rdu = R * du;
  out.value += 0.5 * du.dot(Rdu);
  out.Lu.noalias() += Rdu;
  out.Luu.noalias() += R;
}

inline void addGaussNewtonResidual(PackedCostWorkspace& out, const Eigen::VectorXd& r,
                                   const Eigen::MatrixXd& Jx, const Eigen::MatrixXd& Ju,
                                   const Eigen::MatrixXd& W) {
  if (W.rows() != r.size() || W.cols() != r.size() || Jx.rows() != r.size() || Ju.rows() != r.size() ||
      Jx.cols() != out.Lx.size() || Ju.cols() != out.Lu.size()) {
    throw std::invalid_argument("addGaussNewtonResidual dimension mismatch");
  }

  const Eigen::VectorXd Wr = W * r;
  out.value += 0.5 * r.dot(Wr);
  out.Lx.noalias() += Jx.transpose() * Wr;
  out.Lu.noalias() += Ju.transpose() * Wr;

  const Eigen::MatrixXd WJx = W * Jx;
  const Eigen::MatrixXd WJu = W * Ju;
  out.Lxx.noalias() += Jx.transpose() * WJx;
  out.Lxu.noalias() += Jx.transpose() * WJu;
  out.Lux.noalias() += Ju.transpose() * WJx;
  out.Luu.noalias() += Ju.transpose() * WJu;
}

struct RelaxedBarrierSettings {
  double weight{1.0};
  double delta{1e-3};
  double maxCurvature{1e12};
};

struct ScalarPenaltyApproximation {
  double value{0.0};
  double first{0.0};
  double second{0.0};
};

inline ScalarPenaltyApproximation relaxedUpperBoundPenalty(double c, const RelaxedBarrierSettings& settings) {
  if (!(settings.weight >= 0.0) || !(settings.delta > 0.0)) {
    throw std::invalid_argument("Invalid relaxed barrier settings");
  }

  ScalarPenaltyApproximation out;
  const double delta = settings.delta;
  const double weight = settings.weight;
  if (c < -delta) {
    const double s = -c;
    out.value = -weight * std::log(s);
    out.first = -weight / s;
    out.second = weight / (s * s);
  } else {
    const double shifted = c + delta;
    const double invDelta = 1.0 / delta;
    out.value = -weight * std::log(delta) + weight * invDelta * shifted +
                0.5 * weight * invDelta * invDelta * shifted * shifted;
    out.first = weight * invDelta + weight * invDelta * invDelta * shifted;
    out.second = weight * invDelta * invDelta;
  }
  out.second = std::min(out.second, settings.maxCurvature);
  return out;
}

inline void addScalarPenaltyGaussNewton(PackedCostWorkspace& out, double c,
                                        const Eigen::VectorXd& cx, const Eigen::VectorXd& cu,
                                        const ScalarPenaltyApproximation& penalty) {
  if (cx.size() != out.Lx.size() || cu.size() != out.Lu.size()) {
    throw std::invalid_argument("addScalarPenaltyGaussNewton dimension mismatch");
  }
  out.value += penalty.value;
  out.Lx.noalias() += penalty.first * cx;
  out.Lu.noalias() += penalty.first * cu;
  out.Lxx.noalias() += penalty.second * (cx * cx.transpose());
  out.Lxu.noalias() += penalty.second * (cx * cu.transpose());
  out.Lux.noalias() += penalty.second * (cu * cx.transpose());
  out.Luu.noalias() += penalty.second * (cu * cu.transpose());
}

struct PackedConstraintWorkspace {
  Eigen::VectorXd c;
  Eigen::MatrixXd Cx;
  Eigen::MatrixXd Cu;
  Eigen::VectorXi activeRows;

  void resize(int nc, int nx, int nu) {
    if (nc < 0 || nx <= 0 || nu < 0) {
      throw std::invalid_argument("PackedConstraintWorkspace::resize received invalid dimensions");
    }
    c.resize(nc);
    Cx.resize(nc, nx);
    Cu.resize(nc, nu);
    activeRows.resize(nc);
    setZero();
  }

  void setZero() {
    c.setZero();
    Cx.setZero();
    Cu.setZero();
    activeRows.setOnes();
  }
};

enum class ContactMode { Swing, Stance };

struct ContactModeMask {
  ContactMode left{ContactMode::Stance};
  ContactMode right{ContactMode::Stance};

  bool isStance(int contactIndex) const {
    return contactIndex == 0 ? left == ContactMode::Stance : right == ContactMode::Stance;
  }

  bool isSwing(int contactIndex) const { return !isStance(contactIndex); }
};

struct DirectFrontendNodeParameters {
  double time{0.0};
  double dt{0.02};
  ContactModeMask contactMode;
  Eigen::VectorXd xRef;
  Eigen::VectorXd uRef;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
};

}  // namespace humanoid_cd_nmpc_aligator

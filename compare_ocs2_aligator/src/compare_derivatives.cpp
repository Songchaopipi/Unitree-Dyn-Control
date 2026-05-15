#include <aligator/modelling/dynamics/kinodynamics-fwd.hpp>
#include <aligator/modelling/spaces/multibody.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;
using FrameIndex = pinocchio::FrameIndex;
using AdScalar = ocs2::ad_scalar_t;
using AdVector = ocs2::ad_vector_t;
using AdModel = pinocchio::ModelTpl<AdScalar>;

}  // namespace

namespace pinocchio::internal {

template <>
struct cast_call_normalize_method<pinocchio::SE3Tpl<double, 0>, AdScalar, double> {
  template <typename T>
  static void run(T&) {}
};

}  // namespace pinocchio::internal

namespace {

constexpr int kNumContacts = 2;
constexpr int kForceSize = 6;

struct Options {
  std::string urdf = "../models/g1/g1_29dof.urdf";
  std::string codegenDir = "cppad_codegen";
  std::string modelName = "g1_kinodynamics_flow";
  std::string wrt = "xu";
  int iterations = 1000;
  int warmup = 100;
  bool recompile = false;
  bool verbose = false;
  bool check = true;
};

struct Timings {
  double meanUs = 0.0;
  double minUs = 0.0;
  double maxUs = 0.0;
};

struct Selection {
  std::vector<int> variableIndices;
  std::vector<int> parameterIndices;
  std::vector<int> variableSlot;
  std::vector<int> parameterSlot;
};

void printUsage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n\n"
      << "Options:\n"
      << "  --urdf PATH          URDF path (default: ../models/g1/g1_29dof.urdf)\n"
      << "  --codegen-dir PATH   CppADCodeGen output folder (default: cppad_codegen)\n"
      << "  --model-name NAME    Base generated model name\n"
      << "  --wrt SET            xu|x|u|q|v|wrench|acc|left_wrench|right_wrench or comma list\n"
      << "  --iters N            Timed iterations (default: 1000)\n"
      << "  --warmup N           Warmup iterations (default: 100)\n"
      << "  --recompile          Force CppADCodeGen regeneration/compilation\n"
      << "  --verbose            Print CppADCodeGen messages\n"
      << "  --no-check           Skip value/Jacobian consistency checks\n"
      << "  --help               Show this message\n";
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto requireValue = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) {
        throw std::invalid_argument("Missing value after " + name);
      }
      return argv[++i];
    };

    if (arg == "--urdf") {
      options.urdf = requireValue(arg);
    } else if (arg == "--codegen-dir") {
      options.codegenDir = requireValue(arg);
    } else if (arg == "--model-name") {
      options.modelName = requireValue(arg);
    } else if (arg == "--wrt") {
      options.wrt = requireValue(arg);
    } else if (arg == "--iters") {
      options.iterations = std::stoi(requireValue(arg));
    } else if (arg == "--warmup") {
      options.warmup = std::stoi(requireValue(arg));
    } else if (arg == "--recompile") {
      options.recompile = true;
    } else if (arg == "--verbose") {
      options.verbose = true;
    } else if (arg == "--no-check") {
      options.check = false;
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::invalid_argument("Unknown option: " + arg);
    }
  }

  options.iterations = std::max(1, options.iterations);
  options.warmup = std::max(0, options.warmup);
  return options;
}

template <typename Scalar>
pinocchio::JointModelCompositeTpl<Scalar> makeFloatingBaseJoint() {
  pinocchio::JointModelCompositeTpl<Scalar> base(2);
  base.addJoint(pinocchio::JointModelTranslationTpl<Scalar>());
  base.addJoint(pinocchio::JointModelSphericalZYXTpl<Scalar>());
  return base;
}

void addContactFrameIfMissing(pinocchio::Model& model,
                              const std::string& frameName,
                              const std::string& parentJointName,
                              const Eigen::Vector3d& translation) {
  if (model.existFrame(frameName)) {
    return;
  }
  if (!model.existJointName(parentJointName)) {
    throw std::runtime_error("Missing parent joint for contact frame: " + parentJointName);
  }

  const auto parentJoint = model.getJointId(parentJointName);
  FrameIndex parentFrame = 0;
  if (model.existFrame(parentJointName)) {
    parentFrame = model.getFrameId(parentJointName);
  }

  model.addFrame(pinocchio::Frame(frameName,
                                  parentJoint,
                                  parentFrame,
                                  pinocchio::SE3(Eigen::Matrix3d::Identity(), translation),
                                  pinocchio::FIXED_JOINT));
}

pinocchio::Model loadModel(const std::string& urdfPath) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdfPath, makeFloatingBaseJoint<double>(), model);

  const Eigen::Vector3d contactTranslation(0.035, 0.0, -0.035);
  addContactFrameIfMissing(model, "foot_l_contact", "left_ankle_roll_joint", contactTranslation);
  addContactFrameIfMissing(model, "foot_r_contact", "right_ankle_roll_joint", contactTranslation);
  return model;
}

std::string sanitizeName(std::string name) {
  for (char& c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c))) {
      c = '_';
    }
  }
  return name;
}

std::vector<std::string> splitTokens(const std::string& text) {
  std::vector<std::string> tokens;
  std::stringstream ss(text);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }
  if (tokens.empty()) {
    tokens.push_back(text);
  }
  return tokens;
}

void appendRange(std::set<int>& indices, int begin, int end) {
  for (int i = begin; i < end; ++i) {
    indices.insert(i);
  }
}

Selection makeSelection(const std::string& wrt, int nq, int nv, int nu) {
  const int nx = nq + nv;
  const int zDim = nx + nu;
  const int wrenchOffset = nx;
  const int accOffset = nx + kNumContacts * kForceSize;

  std::set<int> selected;
  for (const auto& token : splitTokens(wrt)) {
    if (token == "xu" || token == "all") {
      appendRange(selected, 0, zDim);
    } else if (token == "x") {
      appendRange(selected, 0, nx);
    } else if (token == "q") {
      appendRange(selected, 0, nq);
    } else if (token == "v") {
      appendRange(selected, nq, nx);
    } else if (token == "u") {
      appendRange(selected, nx, zDim);
    } else if (token == "wrench" || token == "force") {
      appendRange(selected, wrenchOffset, accOffset);
    } else if (token == "left_wrench" || token == "left_force") {
      appendRange(selected, wrenchOffset, wrenchOffset + kForceSize);
    } else if (token == "right_wrench" || token == "right_force") {
      appendRange(selected, wrenchOffset + kForceSize, accOffset);
    } else if (token == "acc" || token == "joint_acc") {
      appendRange(selected, accOffset, zDim);
    } else if (token == "base") {
      appendRange(selected, 0, 6);
      appendRange(selected, nq, nq + 6);
    } else if (token == "joints") {
      appendRange(selected, 6, nq);
      appendRange(selected, nq + 6, nx);
      appendRange(selected, accOffset, zDim);
    } else {
      throw std::invalid_argument("Unknown --wrt token: " + token);
    }
  }

  Selection selection;
  selection.variableIndices.assign(selected.begin(), selected.end());
  if (selection.variableIndices.empty()) {
    throw std::invalid_argument("Derivative selection is empty.");
  }

  selection.variableSlot.assign(zDim, -1);
  selection.parameterSlot.resize(zDim);
  for (int i = 0; i < static_cast<int>(selection.variableIndices.size()); ++i) {
    selection.variableSlot[selection.variableIndices[i]] = i;
  }
  for (int i = 0; i < zDim; ++i) {
    selection.parameterSlot[i] = i;
    selection.parameterIndices.push_back(i);
  }
  return selection;
}

template <typename Scalar>
Eigen::Matrix<Scalar, 6, 6> inverseNoPivot6(Eigen::Matrix<Scalar, 6, 6> a) {
  Eigen::Matrix<Scalar, 6, 6> inv = Eigen::Matrix<Scalar, 6, 6>::Identity();
  for (int pivot = 0; pivot < 6; ++pivot) {
    const Scalar pivotValue = a(pivot, pivot);
    a.row(pivot) /= pivotValue;
    inv.row(pivot) /= pivotValue;
    for (int row = 0; row < 6; ++row) {
      if (row == pivot) {
        continue;
      }
      const Scalar factor = a(row, pivot);
      a.row(row) -= factor * a.row(pivot);
      inv.row(row) -= factor * inv.row(pivot);
    }
  }
  return inv;
}

template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1> kinodynamicsFlow(
    const pinocchio::ModelTpl<Scalar>& model,
    const std::vector<FrameIndex>& contactIds,
    const std::vector<bool>& contactStates,
    double totalMass,
    const Eigen::Matrix<Scalar, 3, 1>& gravity,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& u) {
  using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
  using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;
  using Matrix6X = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;

  pinocchio::DataTpl<Scalar> data(model);
  const int nq = model.nq;
  const int nv = model.nv;
  const auto q = x.head(nq);
  const auto v = x.tail(nv);
  const auto jointAcc = u.tail(nv - 6);

  pinocchio::ccrba(model, data, q, v);
  pinocchio::dccrba(model, data, q, v);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::centerOfMass(model, data, q, v);

  Vector6 contactWrench = Vector6::Zero();
  contactWrench.template head<3>() = Scalar(totalMass) * gravity;

  for (std::size_t i = 0; i < contactStates.size(); ++i) {
    if (!contactStates[i]) {
      continue;
    }

    const int offset = static_cast<int>(i) * kForceSize;
    pinocchio::updateFramePlacement(model, data, contactIds[i]);
    const auto framePosition = data.oMf[contactIds[i]].translation();
    const auto com = data.com[0];
    const auto force = u.template segment<3>(offset);

    contactWrench.template head<3>() += force;
    contactWrench[3] += (framePosition[1] - com[1]) * u[offset + 2] -
                        (framePosition[2] - com[2]) * u[offset + 1];
    contactWrench[4] += (framePosition[2] - com[2]) * u[offset + 0] -
                        (framePosition[0] - com[0]) * u[offset + 2];
    contactWrench[5] += (framePosition[0] - com[0]) * u[offset + 1] -
                        (framePosition[1] - com[1]) * u[offset + 0];
    contactWrench.template tail<3>() += u.template segment<3>(offset + 3);
  }

  const Matrix6 Agu = data.Ag.template leftCols<6>();
  const Matrix6 AguInv = inverseNoPivot6(Agu);
  const Matrix6X Agj = data.Ag.rightCols(nv - 6);

  VectorX xdot = VectorX::Zero(nq + nv);
  xdot.head(nv) = v;
  xdot.segment(nv, 6) = AguInv * (contactWrench - data.dAg * v - Agj * jointAcc);
  xdot.tail(nv - 6) = jointAcc;
  return xdot;
}

VectorXd makeState(const pinocchio::Model& model) {
  VectorXd x = VectorXd::Zero(model.nq + model.nv);
  VectorXd q = VectorXd::Zero(model.nq);
  VectorXd v = VectorXd::Zero(model.nv);

  q[2] = 0.8;
  for (int i = 6; i < model.nq; ++i) {
    q[i] = 0.08 * std::sin(0.37 * static_cast<double>(i));
  }
  for (int i = 0; i < model.nv; ++i) {
    v[i] = 0.03 * std::cos(0.23 * static_cast<double>(i + 1));
  }

  x.head(model.nq) = q;
  x.tail(model.nv) = v;
  return x;
}

VectorXd makeInput(const pinocchio::Model& model, double totalMass) {
  const int nu = kNumContacts * kForceSize + model.nv - 6;
  VectorXd u = VectorXd::Zero(nu);
  const double normalForce = totalMass * 9.80665 / static_cast<double>(kNumContacts);
  for (int contact = 0; contact < kNumContacts; ++contact) {
    const int offset = contact * kForceSize;
    u[offset + 0] = (contact == 0) ? 2.0 : -2.0;
    u[offset + 1] = (contact == 0) ? -1.0 : 1.0;
    u[offset + 2] = normalForce;
    u[offset + 3] = 0.1;
    u[offset + 4] = -0.1;
    u[offset + 5] = 0.05;
  }
  for (int i = kNumContacts * kForceSize; i < nu; ++i) {
    u[i] = 0.2 * std::sin(0.17 * static_cast<double>(i));
  }
  return u;
}

VectorXd gatherByIndex(const VectorXd& z, const std::vector<int>& indices) {
  VectorXd out(indices.size());
  for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
    out[i] = z[indices[i]];
  }
  return out;
}

MatrixXd gatherColumns(const MatrixXd& matrix, const std::vector<int>& indices) {
  MatrixXd out(matrix.rows(), indices.size());
  for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
    out.col(i) = matrix.col(indices[i]);
  }
  return out;
}

Timings benchmark(int warmup, int iterations, const std::function<void()>& fn) {
  for (int i = 0; i < warmup; ++i) {
    fn();
  }

  std::vector<double> samples;
  samples.reserve(iterations);
  for (int i = 0; i < iterations; ++i) {
    const auto start = Clock::now();
    fn();
    const auto stop = Clock::now();
    samples.push_back(std::chrono::duration<double, std::micro>(stop - start).count());
  }

  Timings t;
  t.meanUs = std::accumulate(samples.begin(), samples.end(), 0.0) / static_cast<double>(samples.size());
  t.minUs = *std::min_element(samples.begin(), samples.end());
  t.maxUs = *std::max_element(samples.begin(), samples.end());
  return t;
}

void printTiming(const std::string& name, const Timings& timings) {
  std::cout << std::left << std::setw(32) << name
            << " mean " << std::setw(12) << std::fixed << std::setprecision(3) << timings.meanUs
            << " us  min " << std::setw(12) << timings.minUs
            << " us  max " << std::setw(12) << timings.maxUs << " us\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseOptions(argc, argv);
    const auto urdfPath = std::filesystem::absolute(options.urdf);
    const auto codegenDir = std::filesystem::absolute(options.codegenDir);
    std::filesystem::create_directories(codegenDir);

    pinocchio::Model model = loadModel(urdfPath.string());
    const std::vector<FrameIndex> contactIds{
        model.getFrameId("foot_l_contact"),
        model.getFrameId("foot_r_contact"),
    };
    const std::vector<bool> contactStates{true, true};
    const Eigen::Vector3d gravity(0.0, 0.0, -9.80665);
    const double totalMass = pinocchio::computeTotalMass(model);

    const int nq = model.nq;
    const int nv = model.nv;
    const int nx = nq + nv;
    const int nu = kNumContacts * kForceSize + nv - 6;
    if (nq != nv) {
      throw std::runtime_error("This benchmark expects the composite Euler floating base model with nq == nv.");
    }
    const Selection selection = makeSelection(options.wrt, nq, nv, nu);

    const VectorXd x = makeState(model);
    const VectorXd u = makeInput(model, totalMass);
    VectorXd z(nx + nu);
    z << x, u;
    const VectorXd cppadVariables = VectorXd::Zero(selection.variableIndices.size());
    const VectorXd cppadParameters = z;

    aligator::MultibodyPhaseSpace<double> space(model);
    aligator::dynamics::KinodynamicsFwdDynamicsTpl<double> aligatorDynamics(
        space, model, gravity, contactStates, contactIds, kForceSize);
    auto aligatorData = aligatorDynamics.createData();

    aligatorDynamics.forward(x, u, *aligatorData);
    aligatorDynamics.dForward(x, u, *aligatorData);
    MatrixXd analyticFull(aligatorData->Jx_.rows(), aligatorData->Jx_.cols() + aligatorData->Ju_.cols());
    analyticFull << aligatorData->Jx_, aligatorData->Ju_;
    MatrixXd analyticSelected = gatherColumns(analyticFull, selection.variableIndices);

    const VectorXd manualFlow = kinodynamicsFlow<double>(model, contactIds, contactStates, totalMass, gravity, x, u);
    const double flowErrorInf = (manualFlow - aligatorData->xdot_).lpNorm<Eigen::Infinity>();

    const auto adModel = std::make_shared<const AdModel>(model.cast<AdScalar>());
    const Eigen::Matrix<AdScalar, 3, 1> adGravity = gravity.cast<AdScalar>();
    const auto variableSlot = selection.variableSlot;
    const int zDim = nx + nu;

    ocs2::CppAdInterface::ad_parameterized_function_t adFunction =
        [adModel, contactIds, contactStates, totalMass, adGravity, variableSlot, zDim, nq, nv, nx, nu](
            const AdVector& variables, const AdVector& parameters, AdVector& y) {
          const AdVector xNominal = parameters.head(nx);
          const AdVector uNominal = parameters.tail(nu);
          AdVector dx = AdVector::Zero(nx);
          AdVector du = AdVector::Zero(nu);
          for (int i = 0; i < zDim; ++i) {
            if (variableSlot[i] >= 0) {
              if (i < nx) {
                dx[i] = variables[variableSlot[i]];
              } else {
                du[i - nx] = variables[variableSlot[i]];
              }
            }
          }

          AdVector xAd(nx);
          xAd.head(nq) = xNominal.head(nq) + dx.head(nq);
          xAd.tail(nv) = xNominal.tail(nv) + dx.tail(nv);
          const AdVector uAd = uNominal + du;
          y = kinodynamicsFlow<AdScalar>(*adModel, contactIds, contactStates, totalMass, adGravity, xAd, uAd);
        };

    const std::string generatedName = sanitizeName(options.modelName + "_local_" + options.wrt);
    ocs2::CppAdInterface cppad(adFunction,
                               static_cast<size_t>(selection.variableIndices.size()),
                               static_cast<size_t>(selection.parameterIndices.size()),
                               generatedName,
                               codegenDir.string());

    const auto prepareStart = Clock::now();
    if (options.recompile) {
      cppad.createModels(ocs2::CppAdInterface::ApproximationOrder::First, options.verbose);
    } else {
      cppad.loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, options.verbose);
    }
    const auto prepareStop = Clock::now();
    const double cppadPrepareMs = std::chrono::duration<double, std::milli>(prepareStop - prepareStart).count();

    const MatrixXd cppadJacobian = cppad.getJacobian(cppadVariables, cppadParameters);
    const VectorXd cppadFlow = cppad.getFunctionValue(cppadVariables, cppadParameters);

    const double jacobianErrorInf = (cppadJacobian - analyticSelected).lpNorm<Eigen::Infinity>();
    const double cppadFlowErrorInf = (cppadFlow - aligatorData->xdot_).lpNorm<Eigen::Infinity>();

    MatrixXd sinkJacobian;
    VectorXd sinkValue;

    const Timings aligatorForwardDForward = benchmark(options.warmup, options.iterations, [&]() {
      aligatorDynamics.forward(x, u, *aligatorData);
      aligatorDynamics.dForward(x, u, *aligatorData);
      sinkJacobian.resize(aligatorData->Jx_.rows(), selection.variableIndices.size());
      for (int i = 0; i < static_cast<int>(selection.variableIndices.size()); ++i) {
        const int col = selection.variableIndices[i];
        sinkJacobian.col(i) = (col < nx) ? aligatorData->Jx_.col(col) : aligatorData->Ju_.col(col - nx);
      }
    });

    aligatorDynamics.forward(x, u, *aligatorData);
    const Timings aligatorDForwardOnly = benchmark(options.warmup, options.iterations, [&]() {
      aligatorDynamics.dForward(x, u, *aligatorData);
      sinkJacobian.resize(aligatorData->Jx_.rows(), selection.variableIndices.size());
      for (int i = 0; i < static_cast<int>(selection.variableIndices.size()); ++i) {
        const int col = selection.variableIndices[i];
        sinkJacobian.col(i) = (col < nx) ? aligatorData->Jx_.col(col) : aligatorData->Ju_.col(col - nx);
      }
    });

    const Timings cppadGetJacobian = benchmark(options.warmup, options.iterations, [&]() {
      sinkJacobian = cppad.getJacobian(cppadVariables, cppadParameters);
    });

    const Timings cppadValueAndJacobian = benchmark(options.warmup, options.iterations, [&]() {
      sinkValue = cppad.getFunctionValue(cppadVariables, cppadParameters);
      sinkJacobian = cppad.getJacobian(cppadVariables, cppadParameters);
    });

    volatile double keepAlive = sinkJacobian.size() > 0 ? sinkJacobian(0, 0) : 0.0;
    (void)keepAlive;

    std::cout << "\n=== derivative benchmark ===\n";
    std::cout << "urdf:              " << urdfPath << "\n";
    std::cout << "codegen_dir:       " << codegenDir << "\n";
    std::cout << "generated_model:   " << generatedName << "\n";
    std::cout << "nq/nv/nx/nu:       " << nq << " / " << nv << " / " << nx << " / " << nu << "\n";
    std::cout << "wrt:               " << options.wrt << " (" << selection.variableIndices.size()
              << " variables, " << selection.parameterIndices.size() << " parameters)\n";
    std::cout << "iters/warmup:      " << options.iterations << " / " << options.warmup << "\n";
    std::cout << "cppad_prepare_ms:  " << std::fixed << std::setprecision(3) << cppadPrepareMs << "\n";
    if (options.check) {
      std::cout << "flow_error_inf(manual vs aligator): " << std::scientific << flowErrorInf << "\n";
      std::cout << "flow_error_inf(cppad  vs aligator): " << std::scientific << cppadFlowErrorInf << "\n";
      std::cout << "jac_error_inf(cppad  vs aligator): " << std::scientific << jacobianErrorInf << "\n";
      std::cout << std::fixed;
    }
    std::cout << "\n";
    printTiming("aligator forward+dForward", aligatorForwardDForward);
    printTiming("aligator dForward only", aligatorDForwardOnly);
    printTiming("cppad getJacobian", cppadGetJacobian);
    printTiming("cppad value+getJacobian", cppadValueAndJacobian);
    std::cout << "\n";

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "compare_derivatives failed: " << e.what() << "\n";
    return 1;
  }
}

#include <pinocchio/fwd.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/core/cost-abstract.hpp>
#include <aligator/core/explicit-dynamics.hpp>
#include <aligator/core/stage-data.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/constraints/equality-constraint.hpp>
#include <aligator/modelling/dynamics/integrator-euler.hpp>
#include <aligator/modelling/dynamics/ode-abstract.hpp>
#include <aligator/core/vector-space.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>
#include <ocs2_core/PreComputation.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_sqp/SqpSolver.h>

#include <humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h>
#include <humanoid_centroidal_mpc/constraint/JointMimicKinematicConstraint.h>
#include <humanoid_centroidal_mpc/constraint/NormalVelocityConstraintCppAd.h>
#include <humanoid_centroidal_mpc/constraint/ZeroVelocityConstraintCppAd.h>
#include <humanoid_centroidal_mpc/cost/CentroidalMpcEndEffectorFootCost.h>
#include <humanoid_centroidal_mpc/cost/ICPCost.h>
#include <humanoid_centroidal_mpc/dynamics/CentroidalDynamicsAD.h>
#include <humanoid_common_mpc/HumanoidCostConstraintFactory.h>
#include <humanoid_common_mpc/HumanoidPreComputation.h>
#include <humanoid_common_mpc/constraint/EndEffectorKinematicsTwistConstraint.h>
#include <humanoid_common_mpc/cost/EndEffectorKinematicsQuadraticCost.h>
#include <humanoid_common_mpc/gait/GaitSchedule.h>
#include <humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h>
#include <humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h>
#include <humanoid_cd_nmpc_aligator/direct_constraint_pack.hpp>
#include <humanoid_cd_nmpc_aligator/direct_stage_cost_pack.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace {

using Clock = std::chrono::steady_clock;
using CostStack = aligator::CostStackTpl<double>;
using IntegratorEuler = aligator::dynamics::IntegratorEulerTpl<double>;
using QuadraticControlCost = aligator::QuadraticControlCostTpl<double>;
using QuadraticStateCost = aligator::QuadraticStateCostTpl<double>;
using SolverProxDDP = aligator::SolverProxDDPTpl<double>;
using StageModel = aligator::StageModelTpl<double>;
using TrajOptProblem = aligator::TrajOptProblemTpl<double>;
using VectorSpace = aligator::VectorSpaceTpl<double>;
using DirectStageCostContext = humanoid_cd_nmpc_aligator::DirectStageCostContext;
using DirectStageCostPackAligatorCost = humanoid_cd_nmpc_aligator::DirectStageCostPackAligatorCost;
using DirectConstraintContext = humanoid_cd_nmpc_aligator::DirectConstraintContext;
using DirectConstraintPackAligatorFunction = humanoid_cd_nmpc_aligator::DirectConstraintPackAligatorFunction;

constexpr double kGravity = 9.80665;

enum class ProblemMode {
  Layer1,
  Standard,
};

enum class FrontendMode {
  Legacy,
  Direct,
  Thin,
};

enum class RolloutMode {
  Linear,
  Nonlinear,
};

std::string defaultWsPath() {
#ifdef DEFAULT_WB_HUMANOID_MPC_WS
  return DEFAULT_WB_HUMANOID_MPC_WS;
#else
  return "/home/songchao/OPTControl_env/wb_humanoid_mpc_ws";
#endif
}

std::string joinPath(const std::string& a, const std::string& b) {
  if (a.empty()) {
    return b;
  }
  if (a.back() == '/') {
    return a + b;
  }
  return a + "/" + b;
}

double msSince(const Clock::time_point& start, const Clock::time_point& end) {
  return 1000.0 * std::chrono::duration<double>(end - start).count();
}

bool parseBool(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value == "1" || value == "true" || value == "yes" || value == "on";
}

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

FrontendMode parseFrontendMode(const std::string& text) {
  const std::string value = toLower(text);
  if (value == "legacy") {
    return FrontendMode::Legacy;
  }
  if (value == "direct") {
    return FrontendMode::Direct;
  }
  if (value == "thin") {
    return FrontendMode::Thin;
  }
  throw std::invalid_argument("Unknown frontend mode: " + text + " (expected legacy, direct, or thin)");
}

RolloutMode parseRolloutMode(const std::string& text) {
  const std::string value = toLower(text);
  if (value == "linear") {
    return RolloutMode::Linear;
  }
  if (value == "nonlinear") {
    return RolloutMode::Nonlinear;
  }
  throw std::invalid_argument("Unknown rollout mode: " + text + " (expected linear or nonlinear)");
}

ProblemMode parseProblemMode(const std::string& text) {
  const std::string value = toLower(text);
  if (value == "layer1") {
    return ProblemMode::Layer1;
  }
  if (value == "standard" || value == "ocs2") {
    return ProblemMode::Standard;
  }
  throw std::invalid_argument("Unknown problem mode: " + text + " (expected layer1 or standard)");
}

const char* frontendModeName(FrontendMode mode) {
  switch (mode) {
    case FrontendMode::Legacy:
      return "legacy";
    case FrontendMode::Direct:
      return "direct";
    case FrontendMode::Thin:
      return "thin";
  }
  return "unknown";
}

const char* problemModeName(ProblemMode mode) {
  switch (mode) {
    case ProblemMode::Layer1:
      return "layer1";
    case ProblemMode::Standard:
      return "standard";
  }
  return "unknown";
}

const char* rolloutModeName(RolloutMode mode) {
  switch (mode) {
    case RolloutMode::Linear:
      return "linear";
    case RolloutMode::Nonlinear:
      return "nonlinear";
  }
  return "unknown";
}

template <typename T>
T parseNumber(const std::string& text);

template <>
int parseNumber<int>(const std::string& text) {
  return std::stoi(text);
}

template <>
double parseNumber<double>(const std::string& text) {
  return std::stod(text);
}

struct Options {
  std::string wbWs = defaultWsPath();
  std::string taskFile;
  std::string urdfFile;
  std::string referenceFile;
  std::string codegenDir = "/tmp/ocs2_aligator_centroidal";
  int horizon = 20;
  double dt = 0.02;
  int maxIterations = 10;
  int maxAlIterations = 5;
  int sqpIterations = -1;
  int threads = 1;
  double tolerance = 1e-4;
  double muInit = 1e-2;
  ProblemMode problemMode = ProblemMode::Standard;
  FrontendMode frontend = FrontendMode::Thin;
  RolloutMode rollout = RolloutMode::Nonlinear;
  bool recompileCodegen = false;
  bool verbose = false;
  bool solverVerbose = false;
};

void fillDefaultPaths(Options& options) {
  if (options.taskFile.empty()) {
    options.taskFile = joinPath(options.wbWs, "config/g1_centroidal/mpc/task.info");
  }
  if (options.urdfFile.empty()) {
    options.urdfFile = joinPath(options.wbWs, "models/g1_description/urdf/g1_29dof.urdf");
  }
  if (options.referenceFile.empty()) {
    options.referenceFile = joinPath(options.wbWs, "config/g1_centroidal/command/reference.info");
  }
}

void printUsage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " [options]\n"
      << "\n"
      << "Options:\n"
      << "  --wb-ws PATH              wb_humanoid_mpc_ws path\n"
      << "  --task PATH               OCS2 centroidal task.info\n"
      << "  --urdf PATH               robot URDF\n"
      << "  --reference PATH          OCS2 reference.info\n"
      << "  --codegen-dir PATH        CppADCodeGen output/cache dir\n"
      << "  --horizon N               number of shooting intervals, default 20\n"
      << "  --dt SEC                  Euler step, default 0.02\n"
      << "  --max-iters N             ProxDDP inner iterations, default 10\n"
      << "  --max-al-iters N          ProxDDP AL outer iterations, default 5\n"
      << "  --sqp-iters N             OCS2 SQP iterations, default follows --max-iters\n"
      << "  --threads N               Aligator derivative threads, default 1\n"
      << "  --tol VALUE               ProxDDP tolerance, default 1e-4\n"
      << "  --mu-init VALUE           ProxDDP initial AL/prox penalty, default 1e-2\n"
      << "  --problem MODE            layer1 or standard, default standard\n"
      << "  --frontend MODE           legacy, direct, or thin, default thin\n"
      << "  --rollout MODE            linear or nonlinear, default nonlinear\n"
      << "  --recompile true|false    rebuild OCS2 codegen library, default false\n"
      << "  --verbose true|false      verbose OCS2 model/codegen loading, default false\n"
      << "  --solver-verbose true|false\n"
      << "  --help\n";
}

Options parseOptions(int argc, char** argv) {
  Options options;
  std::unordered_map<std::string, std::string*> stringOptions{
      {"--wb-ws", &options.wbWs},
      {"--task", &options.taskFile},
      {"--urdf", &options.urdfFile},
      {"--reference", &options.referenceFile},
      {"--codegen-dir", &options.codegenDir},
  };

  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (key == "--help" || key == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("Missing value for option: " + key);
    }
    const std::string value(argv[++i]);
    if (const auto it = stringOptions.find(key); it != stringOptions.end()) {
      *it->second = value;
    } else if (key == "--horizon") {
      options.horizon = parseNumber<int>(value);
    } else if (key == "--dt") {
      options.dt = parseNumber<double>(value);
    } else if (key == "--max-iters") {
      options.maxIterations = parseNumber<int>(value);
    } else if (key == "--max-al-iters") {
      options.maxAlIterations = parseNumber<int>(value);
    } else if (key == "--sqp-iters") {
      options.sqpIterations = parseNumber<int>(value);
    } else if (key == "--threads") {
      options.threads = parseNumber<int>(value);
    } else if (key == "--tol") {
      options.tolerance = parseNumber<double>(value);
    } else if (key == "--mu-init") {
      options.muInit = parseNumber<double>(value);
    } else if (key == "--problem") {
      options.problemMode = parseProblemMode(value);
    } else if (key == "--frontend") {
      options.frontend = parseFrontendMode(value);
    } else if (key == "--rollout") {
      options.rollout = parseRolloutMode(value);
    } else if (key == "--recompile") {
      options.recompileCodegen = parseBool(value);
    } else if (key == "--verbose") {
      options.verbose = parseBool(value);
    } else if (key == "--solver-verbose") {
      options.solverVerbose = parseBool(value);
    } else {
      throw std::invalid_argument("Unknown option: " + key);
    }
  }

  fillDefaultPaths(options);
  options.horizon = std::max(1, options.horizon);
  options.maxIterations = std::max(1, options.maxIterations);
  options.maxAlIterations = std::max(1, options.maxAlIterations);
  if (options.sqpIterations < 1) {
    options.sqpIterations = options.maxIterations;
  }
  options.sqpIterations = std::max(1, options.sqpIterations);
  options.threads = std::max(1, options.threads);
  if (!(options.dt > 0.0)) {
    throw std::invalid_argument("--dt must be positive");
  }
  return options;
}

Eigen::MatrixXd loadMatrixOrIdentity(const std::string& taskFile, const std::string& key, int rows, int cols, double diagValue) {
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(rows, cols);
  try {
    ocs2::loadData::loadEigenMatrix(taskFile, key, matrix);
  } catch (const std::exception& e) {
    std::cerr << "[layer1] WARNING: failed to load " << key << " from task file: " << e.what()
              << ". Falling back to " << diagValue << " * I.\n";
    matrix.setIdentity();
    matrix *= diagValue;
  }
  return matrix;
}

double loadScalarOrDefault(const std::string& taskFile, const std::string& key, double fallback) {
  try {
    double value = fallback;
    ocs2::loadData::loadCppDataType(taskFile, key, value);
    return value;
  } catch (const std::exception&) {
    return fallback;
  }
}

Eigen::VectorXd loadInitialState(const std::string& taskFile, int nx) {
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", x0);
  return x0;
}

Eigen::VectorXd makeNominalInput(const ocs2::CentroidalModelInfo& info) {
  Eigen::VectorXd input = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(info.inputDim));
  const std::size_t numContacts = info.numThreeDofContacts + info.numSixDofContacts;
  if (numContacts == 0) {
    return input;
  }

  const double fzPerContact = info.robotMass * kGravity / static_cast<double>(numContacts);
  for (std::size_t i = 0; i < numContacts; ++i) {
    ocs2::centroidal_model::getContactForces(input, i, info).z() = fzPerContact;
  }
  return input;
}

struct Layer1References {
  Eigen::MatrixXd q;
  Eigen::MatrixXd r;
  Eigen::MatrixXd qFinal;
  Eigen::VectorXd xRef;
  Eigen::VectorXd uRef;
};

struct StandardOcs2Bundle {
  std::unique_ptr<ocs2::humanoid::CentroidalMpcRobotModel<ocs2::scalar_t>> mpcRobotModel;
  std::unique_ptr<ocs2::humanoid::CentroidalMpcRobotModel<ocs2::ad_scalar_t>> mpcRobotModelAD;
  std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> referenceManager;
  ocs2::OptimalControlProblem problem;
};

Layer1References makeLayer1References(const Options& options, const ocs2::CentroidalModelInfo& info, const Eigen::VectorXd& x0) {
  Layer1References refs;
  refs.q = loadMatrixOrIdentity(options.taskFile, "Q", static_cast<int>(info.stateDim), static_cast<int>(info.stateDim), 1.0);
  refs.r = loadMatrixOrIdentity(options.taskFile, "R", static_cast<int>(info.inputDim), static_cast<int>(info.inputDim), 1e-3);
  refs.qFinal = loadMatrixOrIdentity(options.taskFile, "Q_final", static_cast<int>(info.stateDim), static_cast<int>(info.stateDim), 10.0);
  refs.qFinal *= loadScalarOrDefault(options.taskFile, "terminalCostScaling", 1.0);
  refs.xRef = x0;
  refs.uRef = makeNominalInput(info);
  return refs;
}

std::unique_ptr<ocs2::StateInputConstraint> makeStanceFootConstraint(
    const ocs2::humanoid::SwitchedModelReferenceManager& referenceManager,
    const ocs2::humanoid::ModelSettings& modelSettings,
    const ocs2::EndEffectorKinematics<ocs2::scalar_t>& eeKinematics,
    std::size_t contactPointIndex) {
  auto eeZeroVelConConfig = [](ocs2::scalar_t positionErrorGain, ocs2::scalar_t orientationErrorGain) {
    ocs2::humanoid::EndEffectorKinematicsTwistConstraint::Config config;
    config.b.setZero(6);
    config.Ax.setZero(6, 6);
    config.Av.setIdentity(6, 6);
    if (!ocs2::numerics::almost_eq(positionErrorGain, 0.0)) {
      config.Ax(2, 2) = positionErrorGain;
    }
    if (!ocs2::numerics::almost_eq(orientationErrorGain, 0.0)) {
      config.Ax.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * orientationErrorGain;
    }
    return config;
  };

  return std::make_unique<ocs2::humanoid::ZeroVelocityConstraintCppAd>(
      referenceManager, eeKinematics, contactPointIndex,
      eeZeroVelConConfig(modelSettings.footConstraintConfig.positionErrorGain_z,
                         modelSettings.footConstraintConfig.orientationErrorGain));
}

std::unique_ptr<ocs2::StateInputConstraint> makeNormalVelocityConstraint(
    const ocs2::humanoid::SwitchedModelReferenceManager& referenceManager,
    const ocs2::EndEffectorKinematics<ocs2::scalar_t>& eeKinematics,
    std::size_t contactPointIndex) {
  return std::make_unique<ocs2::humanoid::NormalVelocityConstraintCppAd>(referenceManager, eeKinematics, contactPointIndex);
}

std::unique_ptr<ocs2::StateInputConstraint> makeJointMimicConstraint(
    const std::string& taskFile,
    const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpcRobotModel,
    std::size_t mimicIndex,
    bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix;
  if (mimicIndex == 0) {
    prefix = "mimicJoints.left_knee.";
  } else if (mimicIndex == 1) {
    prefix = "mimicJoints.right_knee.";
  } else {
    throw std::runtime_error("No mimic joint for index: " + std::to_string(mimicIndex));
  }

  std::string parentJointName;
  std::string childJointName;
  ocs2::scalar_t multiplier = 1.0;
  ocs2::scalar_t positionGain = 0.0;
  ocs2::loadData::loadPtreeValue(pt, parentJointName, prefix + "parentJointName", verbose);
  ocs2::loadData::loadPtreeValue(pt, childJointName, prefix + "childJointName", verbose);
  ocs2::loadData::loadPtreeValue(pt, multiplier, prefix + "multiplier", verbose);
  ocs2::loadData::loadPtreeValue(pt, positionGain, prefix + "positionGain", verbose);

  ocs2::humanoid::JointMimicKinematicConstraint::Config config(
      mpcRobotModel, parentJointName, childJointName, multiplier, positionGain);
  return std::make_unique<ocs2::humanoid::JointMimicKinematicConstraint>(mpcRobotModel, config);
}

bool taskFileHasMimicJoints(const std::string& taskFile) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  return ocs2::loadData::containsPtreeValueFind(pt, "mimicJoints");
}

void addTaskSpaceKinematicsCostsToStandardProblem(
    const Options& options,
    const ocs2::PinocchioInterface& pinocchioInterface,
    const ocs2::CentroidalModelInfo& info,
    const ocs2::humanoid::ModelSettings& modelSettings,
    const ocs2::humanoid::MpcRobotModelBase<ocs2::ad_scalar_t>& mpcRobotModelAD,
    const ocs2::CentroidalModelPinocchioMappingCppAd& pinocchioMappingCppAd,
    const ocs2::PinocchioEndEffectorKinematicsCppAd::update_pinocchio_interface_callback& velocityUpdateCallback,
    ocs2::OptimalControlProblem& problem) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(options.taskFile, pt);
  const boost::property_tree::ptree taskSpaceCostsPt = pt.get_child("task_space_costs");

  for (const auto& taskSpaceCost : taskSpaceCostsPt) {
    const std::string costName = taskSpaceCost.first;
    std::string linkName;
    ocs2::loadData::loadPtreeValue(taskSpaceCostsPt, linkName, costName + ".link_name", options.verbose);

    ocs2::PinocchioEndEffectorKinematicsCppAd eeKinematics(
        pinocchioInterface, pinocchioMappingCppAd, {linkName}, info.stateDim, info.inputDim, velocityUpdateCallback,
        linkName, modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);
    const ocs2::humanoid::EndEffectorKinematicsWeights weights =
        ocs2::humanoid::EndEffectorKinematicsWeights::getWeights(
            options.taskFile, "task_space_costs." + costName + ".weights.", options.verbose);

    problem.costPtr->add(
        costName + "_TaskSpaceKinematicsCost",
        std::make_unique<ocs2::humanoid::EndEffectorKinematicsQuadraticCost>(
            weights, pinocchioInterface, eeKinematics, mpcRobotModelAD, linkName, modelSettings));
  }
}

StandardOcs2Bundle buildStandardOcs2Bundle(const Options& options,
                                           const ocs2::humanoid::ModelSettings& modelSettings,
                                           const ocs2::PinocchioInterface& pinocchioInterface,
                                           const ocs2::CentroidalModelInfo& info) {
  StandardOcs2Bundle bundle;
  bundle.mpcRobotModel = std::make_unique<ocs2::humanoid::CentroidalMpcRobotModel<ocs2::scalar_t>>(
      modelSettings, pinocchioInterface, info);
  bundle.mpcRobotModelAD = std::make_unique<ocs2::humanoid::CentroidalMpcRobotModel<ocs2::ad_scalar_t>>(
      modelSettings, pinocchioInterface.toCppAd(), info.toCppAd());

  auto swingTrajectoryPlanner = std::make_shared<ocs2::humanoid::SwingTrajectoryPlanner>(
      ocs2::humanoid::loadSwingTrajectorySettings(options.taskFile, "swing_trajectory_config", options.verbose),
      ocs2::N_CONTACTS);
  bundle.referenceManager = std::make_shared<ocs2::humanoid::SwitchedModelReferenceManager>(
      ocs2::humanoid::GaitSchedule::loadGaitSchedule(options.referenceFile, modelSettings, options.verbose),
      swingTrajectoryPlanner, pinocchioInterface, *bundle.mpcRobotModel);
  bundle.referenceManager->setArmSwingReferenceActive(true);

  ocs2::humanoid::HumanoidCostConstraintFactory factory(
      options.taskFile, options.referenceFile, *bundle.referenceManager, pinocchioInterface, *bundle.mpcRobotModel,
      *bundle.mpcRobotModelAD, modelSettings, options.verbose);

  bundle.problem.dynamicsPtr = std::make_unique<ocs2::humanoid::CentroidalDynamicsAD>(
      pinocchioInterface, info, "dynamics", modelSettings);
  bundle.problem.costPtr->add("stateInputQuadraticCost", factory.getStateInputQuadraticCost());
  bundle.problem.finalCostPtr->add("terminalCost", factory.getTerminalCost());

  const auto infoCppAd = info.toCppAd();
  const ocs2::CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
  auto velocityUpdateCallback = [&infoCppAd](const ocs2::ad_vector_t& state,
                                             ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
    const ocs2::ad_vector_t q = ocs2::centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
    ocs2::updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
  };

  addTaskSpaceKinematicsCostsToStandardProblem(options, pinocchioInterface, info, modelSettings, *bundle.mpcRobotModelAD,
                                               pinocchioMappingCppAd, velocityUpdateCallback, bundle.problem);

  const ocs2::vector2_t icpWeights =
      ocs2::humanoid::ICPCost::getWeights(options.taskFile, "icp_cost_weights.", options.verbose);
  bundle.problem.costPtr->add(
      "icp_Cost",
      std::make_unique<ocs2::humanoid::ICPCost>(*bundle.referenceManager, icpWeights, pinocchioInterface,
                                                *bundle.mpcRobotModelAD, "icp_Cost", modelSettings));

  bundle.problem.stateSoftConstraintPtr->add("jointLimits", factory.getJointLimitsConstraint());
  bundle.problem.stateSoftConstraintPtr->add("FootCollisionSoftConstraint", factory.getFootCollisionConstraint());

  const ocs2::humanoid::EndEffectorKinematicsWeights footTrackingCostWeights =
      ocs2::humanoid::EndEffectorKinematicsWeights::getWeights(
          options.taskFile, "task_space_foot_cost_weights.", options.verbose);
  const bool hasMimicJoints = taskFileHasMimicJoints(options.taskFile);

  for (std::size_t i = 0; i < ocs2::N_CONTACTS; ++i) {
    const std::string& footName = modelSettings.contactNames[i];
    ocs2::PinocchioEndEffectorKinematicsCppAd eeKinematics(
        pinocchioInterface, pinocchioMappingCppAd, {footName}, info.stateDim, info.inputDim, velocityUpdateCallback,
        footName, modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);

    bundle.problem.softConstraintPtr->add(footName + "_frictionForceCone", factory.getFrictionForceConeConstraint(i));
    bundle.problem.softConstraintPtr->add(footName + "_contactMomentXY",
                                          factory.getContactMomentXYConstraint(i, footName + "_contact_moment_XY_constraint"));
    bundle.problem.equalityConstraintPtr->add(footName + "_zeroWrench", factory.getZeroWrenchConstraint(i));
    bundle.problem.equalityConstraintPtr->add(footName + "_zeroVelocity",
                                              makeStanceFootConstraint(*bundle.referenceManager, modelSettings, eeKinematics, i));
    bundle.problem.equalityConstraintPtr->add(footName + "_normalVelocity",
                                              makeNormalVelocityConstraint(*bundle.referenceManager, eeKinematics, i));
    if (hasMimicJoints) {
      bundle.problem.equalityConstraintPtr->add(footName + "_kneeJointMimic",
                                                makeJointMimicConstraint(options.taskFile, *bundle.mpcRobotModel, i, options.verbose));
    }

    const std::string footTrackingCostName = footName + "_TaskSpaceKinematicsCost";
    bundle.problem.costPtr->add(
        footTrackingCostName,
        std::make_unique<ocs2::humanoid::CentroidalMpcEndEffectorFootCost>(
            *bundle.referenceManager, footTrackingCostWeights, pinocchioInterface, *bundle.mpcRobotModelAD, i,
            footTrackingCostName, modelSettings));
    bundle.problem.costPtr->add(footName + "_ExternalTorqueQuadraticCost", factory.getExternalTorqueQuadraticCost(i));
  }

  bundle.problem.preComputationPtr = std::make_unique<ocs2::humanoid::HumanoidPreComputation>(
      pinocchioInterface, *bundle.referenceManager->getSwingTrajectoryPlanner(), *bundle.mpcRobotModel);
  return bundle;
}

class Ocs2CentroidalOde final : public aligator::dynamics::ODEAbstractTpl<double> {
 public:
  using Base = aligator::dynamics::ODEAbstractTpl<double>;
  using Data = aligator::dynamics::ContinuousDynamicsDataTpl<double>;

  Ocs2CentroidalOde(const VectorSpace& space, int nu, std::shared_ptr<ocs2::PinocchioCentroidalDynamicsAD> dynamics, double time)
      : Base(space, nu), dynamics_(std::move(dynamics)), time_(time) {
    if (!dynamics_) {
      throw std::invalid_argument("Ocs2CentroidalOde requires a valid dynamics object");
    }
  }

  void forward(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    data.xdot_ = dynamics_->getValue(time_, x, u);
  }

  void dForward(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    const ocs2::VectorFunctionLinearApproximation linearization = dynamics_->getLinearApproximation(time_, x, u);
    data.xdot_ = linearization.f;
    data.Jx_ = linearization.dfdx;
    data.Ju_ = linearization.dfdu;
  }

  std::shared_ptr<Data> createData() const override {
    return std::make_shared<Data>(space_->ndx(), nu_);
  }

 private:
  std::shared_ptr<ocs2::PinocchioCentroidalDynamicsAD> dynamics_;
  double time_{0.0};
};

class DirectCentroidalEulerDynamics final : public aligator::ExplicitDynamicsModelTpl<double> {
 public:
  using Base = aligator::ExplicitDynamicsModelTpl<double>;
  using Data = aligator::ExplicitDynamicsDataTpl<double>;

  DirectCentroidalEulerDynamics(const VectorSpace& space, int nu,
                                std::shared_ptr<ocs2::PinocchioCentroidalDynamicsAD> dynamics,
                                double time, double dt)
      : Base(space, nu), dynamics_(std::move(dynamics)), time_(time), dt_(dt) {
    if (!dynamics_) {
      throw std::invalid_argument("DirectCentroidalEulerDynamics requires a valid dynamics object");
    }
    if (!(dt_ > 0.0)) {
      throw std::invalid_argument("DirectCentroidalEulerDynamics requires positive dt");
    }
  }

  void forward(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    data.xnext_ = x;
    data.xnext_.noalias() += dt_ * dynamics_->getValue(time_, x, u);
  }

  void dForward(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    const ocs2::VectorFunctionLinearApproximation linearization = dynamics_->getLinearApproximation(time_, x, u);
    data.xnext_ = x;
    data.xnext_.noalias() += dt_ * linearization.f;
    data.Jx().setIdentity();
    data.Jx().noalias() += dt_ * linearization.dfdx;
    data.Ju().noalias() = dt_ * linearization.dfdu;
  }

 private:
  std::shared_ptr<ocs2::PinocchioCentroidalDynamicsAD> dynamics_;
  double time_{0.0};
  double dt_{0.0};
};

class ThinCentroidalStageModel final : public StageModel {
 public:
  ThinCentroidalStageModel(const DirectStageCostPackAligatorCost& cost, const DirectCentroidalEulerDynamics& dynamics)
      : StageModel(cost, dynamics) {}

  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    if (this->numConstraints() != 0) {
      StageModel::evaluate(x, u, data);
      return;
    }
    directDynamics().forward(x, u, *data.dynamics_data);
    directCost().evaluate(x, u, *data.cost_data);
  }

  void computeFirstOrderDerivatives(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    if (this->numConstraints() != 0) {
      StageModel::computeFirstOrderDerivatives(x, u, data);
      return;
    }
    directDynamics().dForward(x, u, *data.dynamics_data);
    directCost().computeGradients(x, u, *data.cost_data);
  }

  void computeSecondOrderDerivatives(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    if (this->numConstraints() != 0) {
      StageModel::computeSecondOrderDerivatives(x, u, data);
      return;
    }
    directCost().computeHessians(x, u, *data.cost_data);
  }

 private:
  const DirectCentroidalEulerDynamics& directDynamics() const {
    return static_cast<const DirectCentroidalEulerDynamics&>(*this->dynamics_);
  }

  const DirectStageCostPackAligatorCost& directCost() const {
    return static_cast<const DirectStageCostPackAligatorCost&>(*this->cost_);
  }
};

class Ocs2CentroidalSystemDynamics final : public ocs2::SystemDynamicsBase {
 public:
  explicit Ocs2CentroidalSystemDynamics(std::shared_ptr<ocs2::PinocchioCentroidalDynamicsAD> dynamics)
      : dynamics_(std::move(dynamics)) {
    if (!dynamics_) {
      throw std::invalid_argument("Ocs2CentroidalSystemDynamics requires a valid dynamics object");
    }
  }

  Ocs2CentroidalSystemDynamics* clone() const override {
    return new Ocs2CentroidalSystemDynamics(*this);
  }

  ocs2::vector_t computeFlowMap(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                                const ocs2::PreComputation&) override {
    return dynamics_->getValue(time, state, input);
  }

  ocs2::VectorFunctionLinearApproximation linearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                              const ocs2::vector_t& input,
                                                              const ocs2::PreComputation&) override {
    return dynamics_->getLinearApproximation(time, state, input);
  }

 private:
  Ocs2CentroidalSystemDynamics(const Ocs2CentroidalSystemDynamics& rhs) = default;

  std::shared_ptr<ocs2::PinocchioCentroidalDynamicsAD> dynamics_;
};

class Ocs2SystemEulerDynamics final : public aligator::ExplicitDynamicsModelTpl<double> {
 public:
  using Base = aligator::ExplicitDynamicsModelTpl<double>;
  using Data = aligator::ExplicitDynamicsDataTpl<double>;

  Ocs2SystemEulerDynamics(const VectorSpace& space, int nu, std::unique_ptr<ocs2::SystemDynamicsBase> dynamics,
                          ocs2::PreComputation* preComputation, double time, double dt)
      : Base(space, nu),
        dynamics_(std::move(dynamics)),
        preComputation_(preComputation),
        time_(time),
        dt_(dt) {
    if (!dynamics_) {
      throw std::invalid_argument("Ocs2SystemEulerDynamics requires a valid dynamics object");
    }
    if (!preComputation_) {
      throw std::invalid_argument("Ocs2SystemEulerDynamics requires pre-computation");
    }
    if (!(dt_ > 0.0)) {
      throw std::invalid_argument("Ocs2SystemEulerDynamics requires positive dt");
    }
  }

  Ocs2SystemEulerDynamics(const Ocs2SystemEulerDynamics& rhs)
      : Base(rhs.space_, rhs.nu),
        dynamics_(rhs.dynamics_ ? rhs.dynamics_->clone() : nullptr),
        preComputation_(rhs.preComputation_),
        time_(rhs.time_),
        dt_(rhs.dt_) {}

  void forward(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    const Eigen::VectorXd xVec = x;
    const Eigen::VectorXd uVec = u;
    preComputation_->request(ocs2::Request::Dynamics, time_, xVec, uVec);
    data.xnext_ = xVec;
    data.xnext_.noalias() += dt_ * dynamics_->computeFlowMap(time_, xVec, uVec, *preComputation_);
  }

  void dForward(const ConstVectorRef& x, const ConstVectorRef& u, Data& data) const override {
    const Eigen::VectorXd xVec = x;
    const Eigen::VectorXd uVec = u;
    preComputation_->request(ocs2::Request::Dynamics + ocs2::Request::Approximation, time_, xVec, uVec);
    const ocs2::VectorFunctionLinearApproximation linearization =
        dynamics_->linearApproximation(time_, xVec, uVec, *preComputation_);
    data.xnext_ = xVec;
    data.xnext_.noalias() += dt_ * linearization.f;
    data.Jx().setIdentity();
    data.Jx().noalias() += dt_ * linearization.dfdx;
    data.Ju().noalias() = dt_ * linearization.dfdu;
  }

 private:
  std::unique_ptr<ocs2::SystemDynamicsBase> dynamics_;
  ocs2::PreComputation* preComputation_{nullptr};
  double time_{0.0};
  double dt_{0.0};
};

struct ProblemBundle {
  TrajOptProblem problem;
  std::vector<Eigen::VectorXd> xsInit;
  std::vector<Eigen::VectorXd> usInit;
  std::vector<std::unique_ptr<ocs2::PreComputation>> preComputations;

  ProblemBundle(TrajOptProblem&& p, std::vector<Eigen::VectorXd>&& xs, std::vector<Eigen::VectorXd>&& us,
                std::vector<std::unique_ptr<ocs2::PreComputation>>&& preComp = {})
      : problem(std::move(p)), xsInit(std::move(xs)), usInit(std::move(us)), preComputations(std::move(preComp)) {}
};

ProblemBundle buildLayer1Problem(const Options& options, const Layer1References& refs,
                                 const std::shared_ptr<ocs2::PinocchioCentroidalDynamicsAD>& dynamics) {
  const int nx = static_cast<int>(refs.xRef.size());
  const int nu = static_cast<int>(refs.uRef.size());
  const VectorSpace space(nx);

  std::vector<xyz::polymorphic<StageModel>> stages;
  stages.reserve(static_cast<std::size_t>(options.horizon));
  for (int k = 0; k < options.horizon; ++k) {
    if (options.frontend == FrontendMode::Legacy) {
      CostStack runningCost(space, nu);
      runningCost.addCost("state_quadratic", QuadraticStateCost(space, nu, refs.xRef, options.dt * refs.q));
      runningCost.addCost("input_quadratic", QuadraticControlCost(space, refs.uRef, options.dt * refs.r));

      const Ocs2CentroidalOde ode(space, nu, dynamics, static_cast<double>(k) * options.dt);
      const IntegratorEuler discreteDynamics(ode, options.dt);
      stages.emplace_back(StageModel(runningCost, discreteDynamics));
    } else {
      const DirectStageCostContext costContext{static_cast<double>(k) * options.dt, options.dt, nullptr, nullptr};
      const DirectStageCostPackAligatorCost runningCost(
          space, nu, costContext,
          humanoid_cd_nmpc_aligator::makeLayer2AQuadraticRunningPack(refs.xRef, refs.uRef, refs.q, refs.r));
      const DirectCentroidalEulerDynamics discreteDynamics(space, nu, dynamics, static_cast<double>(k) * options.dt, options.dt);
      if (options.frontend == FrontendMode::Thin) {
        stages.emplace_back(ThinCentroidalStageModel(runningCost, discreteDynamics));
      } else {
        stages.emplace_back(StageModel(runningCost, discreteDynamics));
      }
    }
  }

  xyz::polymorphic<aligator::CostAbstractTpl<double>> terminalCost =
      [&]() -> xyz::polymorphic<aligator::CostAbstractTpl<double>> {
    if (options.frontend == FrontendMode::Legacy) {
      CostStack legacyTerminalCost(space, nu);
      legacyTerminalCost.addCost("terminal_state_quadratic", QuadraticStateCost(space, nu, refs.xRef, refs.qFinal));
      return legacyTerminalCost;
    }
    const DirectStageCostContext terminalContext{static_cast<double>(options.horizon) * options.dt, 1.0, nullptr, nullptr};
    return DirectStageCostPackAligatorCost(
        space, nu, terminalContext, humanoid_cd_nmpc_aligator::makeLayer2ATerminalPack(refs.xRef, refs.qFinal));
  }();

  TrajOptProblem problem(refs.xRef, stages, terminalCost);
  problem.setInitState(refs.xRef);

  std::vector<Eigen::VectorXd> xsInit(static_cast<std::size_t>(options.horizon) + 1, refs.xRef);
  std::vector<Eigen::VectorXd> usInit(static_cast<std::size_t>(options.horizon), refs.uRef);
  for (int k = 0; k < options.horizon; ++k) {
    xsInit[static_cast<std::size_t>(k + 1)] =
        xsInit[static_cast<std::size_t>(k)] +
        options.dt * dynamics->getValue(static_cast<double>(k) * options.dt, xsInit[static_cast<std::size_t>(k)], refs.uRef);
  }
  return ProblemBundle(std::move(problem), std::move(xsInit), std::move(usInit));
}

std::unique_ptr<ocs2::PreComputation> clonePreComputation(const ocs2::OptimalControlProblem& ocp) {
  if (ocp.preComputationPtr) {
    return std::unique_ptr<ocs2::PreComputation>(ocp.preComputationPtr->clone());
  }
  return std::make_unique<ocs2::PreComputation>();
}

humanoid_cd_nmpc_aligator::DirectStageCostPack makeStandardRunningCostPack(const ocs2::OptimalControlProblem& ocp) {
  humanoid_cd_nmpc_aligator::DirectStageCostPack pack;
  pack.addTerm(std::make_unique<humanoid_cd_nmpc_aligator::Ocs2StateInputCostCollectionTerm>(
      "ocs2_running_cost_collection", std::unique_ptr<ocs2::StateInputCostCollection>(ocp.costPtr->clone())));
  pack.addTerm(std::make_unique<humanoid_cd_nmpc_aligator::Ocs2StateInputCostCollectionTerm>(
      "ocs2_soft_constraint_collection", std::unique_ptr<ocs2::StateInputCostCollection>(ocp.softConstraintPtr->clone())));
  pack.addTerm(std::make_unique<humanoid_cd_nmpc_aligator::Ocs2StateCostCollectionTerm>(
      "ocs2_state_soft_constraint_collection", std::unique_ptr<ocs2::StateCostCollection>(ocp.stateSoftConstraintPtr->clone()), true));
  return pack;
}

humanoid_cd_nmpc_aligator::DirectStageCostPack makeStandardTerminalCostPack(const ocs2::OptimalControlProblem& ocp) {
  humanoid_cd_nmpc_aligator::DirectStageCostPack pack;
  pack.addTerm(std::make_unique<humanoid_cd_nmpc_aligator::Ocs2StateCostCollectionTerm>(
      "ocs2_terminal_cost_collection", std::unique_ptr<ocs2::StateCostCollection>(ocp.finalCostPtr->clone()), false));
  return pack;
}

humanoid_cd_nmpc_aligator::DirectConstraintPack makeStandardEqualityConstraintPack(const ocs2::OptimalControlProblem& ocp) {
  humanoid_cd_nmpc_aligator::DirectConstraintPack pack;
  pack.addTerm(std::make_unique<humanoid_cd_nmpc_aligator::Ocs2StateInputEqualityConstraintCollectionTerm>(
      "ocs2_equality_constraint_collection",
      std::unique_ptr<ocs2::StateInputConstraintCollection>(ocp.equalityConstraintPtr->clone())));
  return pack;
}

ProblemBundle buildStandardProblem(const Options& options, const Layer1References& refs,
                                   const ocs2::OptimalControlProblem& standardOcp,
                                   const ocs2::TargetTrajectories& targetTrajectories) {
  if (!standardOcp.dynamicsPtr) {
    throw std::invalid_argument("Standard OCS2 problem has no dynamics");
  }

  const int nx = static_cast<int>(refs.xRef.size());
  const int nu = static_cast<int>(refs.uRef.size());
  const VectorSpace space(nx);

  std::vector<std::unique_ptr<ocs2::PreComputation>> preComputations;
  preComputations.reserve(static_cast<std::size_t>(options.horizon) + 1);

  std::vector<xyz::polymorphic<StageModel>> stages;
  stages.reserve(static_cast<std::size_t>(options.horizon));
  for (int k = 0; k < options.horizon; ++k) {
    preComputations.push_back(clonePreComputation(standardOcp));
    ocs2::PreComputation* preComp = preComputations.back().get();
    const double time = static_cast<double>(k) * options.dt;

    const DirectStageCostContext costContext{time, options.dt, &targetTrajectories, preComp, false};
    const DirectStageCostPackAligatorCost runningCost(space, nu, costContext, makeStandardRunningCostPack(standardOcp));
    const Ocs2SystemEulerDynamics discreteDynamics(
        space, nu, std::unique_ptr<ocs2::SystemDynamicsBase>(standardOcp.dynamicsPtr->clone()), preComp, time, options.dt);
    StageModel stage(runningCost, discreteDynamics);

    auto equalityPack = makeStandardEqualityConstraintPack(standardOcp);
    const DirectConstraintContext constraintContext{time, preComp};
    const DirectConstraintPackAligatorFunction equalityConstraint(nx, nu, constraintContext, std::move(equalityPack));
    if (equalityConstraint.nr > 0) {
      stage.addConstraint(equalityConstraint, aligator::EqualityConstraintTpl<double>());
    }

    stages.emplace_back(stage);
  }

  preComputations.push_back(clonePreComputation(standardOcp));
  ocs2::PreComputation* terminalPreComp = preComputations.back().get();
  const DirectStageCostContext terminalContext{
      static_cast<double>(options.horizon) * options.dt, 1.0, &targetTrajectories, terminalPreComp, true};
  xyz::polymorphic<aligator::CostAbstractTpl<double>> terminalCost =
      DirectStageCostPackAligatorCost(space, nu, terminalContext, makeStandardTerminalCostPack(standardOcp));

  TrajOptProblem problem(refs.xRef, stages, terminalCost);
  problem.setInitState(refs.xRef);

  std::vector<Eigen::VectorXd> xsInit(static_cast<std::size_t>(options.horizon) + 1, refs.xRef);
  std::vector<Eigen::VectorXd> usInit(static_cast<std::size_t>(options.horizon), refs.uRef);
  std::unique_ptr<ocs2::SystemDynamicsBase> rolloutDynamics(standardOcp.dynamicsPtr->clone());
  std::unique_ptr<ocs2::PreComputation> rolloutPreComp = clonePreComputation(standardOcp);
  for (int k = 0; k < options.horizon; ++k) {
    const double time = static_cast<double>(k) * options.dt;
    rolloutPreComp->request(ocs2::Request::Dynamics, time, xsInit[static_cast<std::size_t>(k)], refs.uRef);
    xsInit[static_cast<std::size_t>(k + 1)] =
        xsInit[static_cast<std::size_t>(k)] +
        options.dt * rolloutDynamics->computeFlowMap(time, xsInit[static_cast<std::size_t>(k)], refs.uRef, *rolloutPreComp);
  }

  return ProblemBundle(std::move(problem), std::move(xsInit), std::move(usInit), std::move(preComputations));
}

ocs2::TargetTrajectories makeTargetTrajectories(const Options& options, const Layer1References& refs) {
  const double finalTime = static_cast<double>(options.horizon) * options.dt;
  return ocs2::TargetTrajectories({0.0, finalTime}, {refs.xRef, refs.xRef}, {refs.uRef, refs.uRef});
}

ocs2::PrimalSolution makeOcs2InitialPrimal(const Options& options, const ProblemBundle& bundle,
                                           ocs2::ModeSchedule modeSchedule = ocs2::ModeSchedule({}, {0})) {
  ocs2::PrimalSolution primal;
  primal.timeTrajectory_.reserve(static_cast<std::size_t>(options.horizon) + 1);
  for (int k = 0; k <= options.horizon; ++k) {
    primal.timeTrajectory_.push_back(static_cast<double>(k) * options.dt);
  }
  primal.stateTrajectory_ = bundle.xsInit;
  primal.inputTrajectory_.reserve(static_cast<std::size_t>(options.horizon) + 1);
  for (const auto& u : bundle.usInit) {
    primal.inputTrajectory_.push_back(u);
  }
  primal.inputTrajectory_.push_back(bundle.usInit.back());
  primal.modeSchedule_ = std::move(modeSchedule);
  return primal;
}

ocs2::OptimalControlProblem buildOcs2Layer1Problem(const Layer1References& refs,
                                                   const std::shared_ptr<ocs2::PinocchioCentroidalDynamicsAD>& dynamics,
                                                   const ocs2::TargetTrajectories& targetTrajectories) {
  ocs2::OptimalControlProblem problem;
  problem.dynamicsPtr = std::make_unique<Ocs2CentroidalSystemDynamics>(dynamics);
  problem.costPtr->add("state_input_quadratic", std::make_unique<ocs2::QuadraticStateInputCost>(refs.q, refs.r));
  problem.finalCostPtr->add("terminal_state_quadratic", std::make_unique<ocs2::QuadraticStateCost>(refs.qFinal));
  problem.targetTrajectoriesPtr = &targetTrajectories;
  return problem;
}

struct SqpRunResult {
  double totalMs{0.0};
  ocs2::SqpSolver::Benchmarks benchmarks{};
  std::size_t iterations{0};
  ocs2::PerformanceIndex performance{};
  ocs2::PrimalSolution solution{};
};

SqpRunResult runOcs2SqpLayer1(const Options& options, const Layer1References& refs,
                              const std::shared_ptr<ocs2::PinocchioCentroidalDynamicsAD>& dynamics,
                              const ProblemBundle& initialGuess) {
  ocs2::TargetTrajectories targetTrajectories = makeTargetTrajectories(options, refs);
  ocs2::OptimalControlProblem problem = buildOcs2Layer1Problem(refs, dynamics, targetTrajectories);
  auto referenceManager = std::make_shared<ocs2::ReferenceManager>(targetTrajectories, ocs2::ModeSchedule({}, {0}));

  ocs2::sqp::Settings settings = ocs2::sqp::loadSettings(options.taskFile, "multiple_shooting", false);
  settings.dt = options.dt;
  settings.sqpIteration = static_cast<std::size_t>(options.sqpIterations);
  settings.deltaTol = options.tolerance;
  settings.integratorType = ocs2::SensitivityIntegratorType::EULER;
  settings.nThreads = static_cast<std::size_t>(options.threads);
  settings.printSolverStatus = options.solverVerbose;
  settings.printSolverStatistics = options.solverVerbose;
  settings.printLinesearch = options.solverVerbose;
  settings.enableLogging = false;
  settings.useFeedbackPolicy = false;
  settings.createValueFunction = false;

  ocs2::OperatingPoints initializer(refs.xRef, refs.uRef);
  ocs2::SqpSolver solver(settings, problem, initializer);
  solver.setReferenceManager(referenceManager);

  const ocs2::PrimalSolution primalGuess = makeOcs2InitialPrimal(options, initialGuess);
  const double finalTime = static_cast<double>(options.horizon) * options.dt;
  const auto start = Clock::now();
  solver.run(0.0, refs.xRef, 0, finalTime, primalGuess);
  const auto end = Clock::now();

  SqpRunResult result;
  result.totalMs = msSince(start, end);
  result.benchmarks = solver.getBenchmarks();
  result.iterations = solver.getNumIterations();
  result.performance = solver.getPerformanceIndeces();
  solver.getPrimalSolution(finalTime, &result.solution);
  return result;
}

SqpRunResult runOcs2SqpStandard(const Options& options, const Layer1References& refs,
                                const ocs2::OptimalControlProblem& standardOcp,
                                const ocs2::Initializer& initializer,
                                const std::shared_ptr<ocs2::ReferenceManagerInterface>& referenceManager,
                                const ProblemBundle& initialGuess) {
  ocs2::sqp::Settings settings = ocs2::sqp::loadSettings(options.taskFile, "multiple_shooting", false);
  settings.dt = options.dt;
  settings.sqpIteration = static_cast<std::size_t>(options.sqpIterations);
  settings.deltaTol = options.tolerance;
  settings.integratorType = ocs2::SensitivityIntegratorType::EULER;
  settings.nThreads = static_cast<std::size_t>(options.threads);
  settings.printSolverStatus = options.solverVerbose;
  settings.printSolverStatistics = options.solverVerbose;
  settings.printLinesearch = options.solverVerbose;
  settings.enableLogging = false;
  settings.useFeedbackPolicy = false;
  settings.createValueFunction = false;

  ocs2::SqpSolver solver(settings, standardOcp, initializer);
  solver.setReferenceManager(referenceManager);

  const ocs2::PrimalSolution primalGuess =
      makeOcs2InitialPrimal(options, initialGuess, referenceManager->getModeSchedule());
  const double finalTime = static_cast<double>(options.horizon) * options.dt;
  const auto start = Clock::now();
  solver.run(0.0, refs.xRef, 0, finalTime, primalGuess);
  const auto end = Clock::now();

  SqpRunResult result;
  result.totalMs = msSince(start, end);
  result.benchmarks = solver.getBenchmarks();
  result.iterations = solver.getNumIterations();
  result.performance = solver.getPerformanceIndeces();
  solver.getPrimalSolution(finalTime, &result.solution);
  return result;
}

double benchmarkRawCodegenLinearizationMs(const Options& options,
                                          const std::shared_ptr<ocs2::PinocchioCentroidalDynamicsAD>& dynamics,
                                          const ProblemBundle& bundle,
                                          int repeats) {
  repeats = std::max(1, repeats);
  Eigen::Index checksumRows = 0;
  const auto start = Clock::now();
  for (int r = 0; r < repeats; ++r) {
    for (int k = 0; k < options.horizon; ++k) {
      const auto approximation = dynamics->getLinearApproximation(static_cast<double>(k) * options.dt,
                                                                  bundle.xsInit[static_cast<std::size_t>(k)],
                                                                  bundle.usInit[static_cast<std::size_t>(k)]);
      checksumRows += approximation.dfdx.rows();
    }
  }
  const auto end = Clock::now();
  if (checksumRows == 0) {
    throw std::runtime_error("Raw codegen benchmark produced no linearizations");
  }
  return msSince(start, end) / static_cast<double>(repeats);
}

std::string vecHeadString(const Eigen::VectorXd& v, int n) {
  std::ostringstream out;
  out << "[";
  const int count = std::min<int>(n, static_cast<int>(v.size()));
  for (int i = 0; i < count; ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << std::fixed << std::setprecision(4) << v(i);
  }
  if (v.size() > count) {
    out << ", ...";
  }
  out << "]";
  return out.str();
}

double maxAbsDiff(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  if (a.size() != b.size()) {
    return std::numeric_limits<double>::infinity();
  }
  return (a - b).lpNorm<Eigen::Infinity>();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseOptions(argc, argv);
    std::cout << std::boolalpha;
    const char* label = problemModeName(options.problemMode);
    std::cout << "[" << label << "] task=" << options.taskFile << "\n"
              << "[" << label << "] urdf=" << options.urdfFile << "\n"
              << "[" << label << "] reference=" << options.referenceFile << "\n"
              << "[" << label << "] codegen_dir=" << options.codegenDir
              << " recompile=" << options.recompileCodegen
              << " problem=" << problemModeName(options.problemMode)
              << " frontend=" << frontendModeName(options.frontend)
              << " rollout=" << rolloutModeName(options.rollout) << "\n";

    const auto modelStart = Clock::now();
    ocs2::humanoid::ModelSettings modelSettings(options.taskFile, options.urdfFile, "centroidal_mpc_", options.verbose);
    ocs2::PinocchioInterface pinocchioInterface =
        ocs2::humanoid::createCustomPinocchioInterface(options.taskFile, options.urdfFile, modelSettings, false, 1.0, options.verbose);
    const ocs2::vector_t defaultJointState =
        ocs2::centroidal_model::loadDefaultJointState(pinocchioInterface.getModel().nq - 6, options.referenceFile);
    const ocs2::CentroidalModelInfo info = ocs2::centroidal_model::createCentroidalModelInfo(
        pinocchioInterface, ocs2::centroidal_model::loadCentroidalType(options.taskFile), defaultJointState,
        modelSettings.contactNames3DoF, modelSettings.contactNames6DoF);
    const Eigen::VectorXd x0 = loadInitialState(options.taskFile, static_cast<int>(info.stateDim));
    const auto modelEnd = Clock::now();

    const auto codegenStart = Clock::now();
    auto dynamics = std::make_shared<ocs2::PinocchioCentroidalDynamicsAD>(
        pinocchioInterface, info, "aligator_centroidal_layer1_dynamics", options.codegenDir,
        options.recompileCodegen, options.verbose);
    const auto codegenEnd = Clock::now();

    const auto buildStart = Clock::now();
    const Layer1References refs = makeLayer1References(options, info, x0);
    std::unique_ptr<StandardOcs2Bundle> standardBundle;
    std::unique_ptr<ocs2::TargetTrajectories> standardTargetTrajectories;
    ProblemBundle bundle = [&]() {
      if (options.problemMode == ProblemMode::Layer1) {
        return buildLayer1Problem(options, refs, dynamics);
      }

      standardBundle = std::make_unique<StandardOcs2Bundle>(
          buildStandardOcs2Bundle(options, modelSettings, pinocchioInterface, info));
      standardTargetTrajectories = std::make_unique<ocs2::TargetTrajectories>(makeTargetTrajectories(options, refs));
      auto referenceManager = standardBundle->referenceManager;
      referenceManager->setTargetTrajectories(*standardTargetTrajectories);
      referenceManager->preSolverRun(0.0, static_cast<double>(options.horizon) * options.dt, refs.xRef, 0);
      *standardTargetTrajectories = referenceManager->getTargetTrajectories();
      return buildStandardProblem(options, refs, standardBundle->problem, *standardTargetTrajectories);
    }();
    const auto buildEnd = Clock::now();

    const double rawCodegenLinearizationMs = benchmarkRawCodegenLinearizationMs(options, dynamics, bundle, 10);

    SolverProxDDP solver(options.tolerance, options.muInit, static_cast<std::size_t>(options.maxIterations),
                         options.solverVerbose ? aligator::VERBOSE : aligator::QUIET);
    solver.rollout_type_ =
        options.rollout == RolloutMode::Nonlinear ? aligator::RolloutType::NONLINEAR : aligator::RolloutType::LINEAR;
    solver.linear_solver_choice = aligator::LQSolverChoice::SERIAL;
    solver.force_initial_condition_ = true;
    solver.max_al_iters = static_cast<std::size_t>(options.maxAlIterations);
    solver.setNumThreads(static_cast<std::size_t>(options.threads));

    const auto setupStart = Clock::now();
    solver.setup(bundle.problem);
    const auto setupEnd = Clock::now();

    const auto solveStart = Clock::now();
    const bool converged = solver.run(bundle.problem, bundle.xsInit, bundle.usInit);
    const auto solveEnd = Clock::now();

    const auto sqpStart = Clock::now();
    ocs2::OperatingPoints standardInitializer(refs.xRef, refs.uRef);
    const SqpRunResult sqpResult =
        options.problemMode == ProblemMode::Standard
            ? runOcs2SqpStandard(options, refs, standardBundle->problem, standardInitializer,
                                 standardBundle->referenceManager, bundle)
            : runOcs2SqpLayer1(options, refs, dynamics, bundle);
    const auto sqpEnd = Clock::now();

    const Eigen::VectorXd u0 = solver.results_.us.empty() ? bundle.usInit.front() : solver.results_.us.front();
    const Eigen::VectorXd x1 = solver.results_.xs.size() > 1 ? solver.results_.xs[1] : bundle.xsInit[1];
    const Eigen::VectorXd sqpU0 = sqpResult.solution.inputTrajectory_.empty() ? bundle.usInit.front() : sqpResult.solution.inputTrajectory_.front();
    const Eigen::VectorXd sqpX1 = sqpResult.solution.stateTrajectory_.size() > 1 ? sqpResult.solution.stateTrajectory_[1] : bundle.xsInit[1];

    std::cout << "[" << label << "] model dims"
              << " nx=" << info.stateDim
              << " nu=" << info.inputDim
              << " contacts3=" << info.numThreeDofContacts
              << " contacts6=" << info.numSixDofContacts
              << " mass=" << info.robotMass << "\n";
    std::cout << "[" << label << "] horizon"
              << " N=" << options.horizon
              << " dt=" << options.dt
              << " threads=" << options.threads
              << " proxddp_max_iters=" << options.maxIterations
              << " sqp_iters=" << options.sqpIterations << "\n";
    std::cout << "[" << label << "] timing_ms"
              << " model_load=" << std::fixed << std::setprecision(3) << msSince(modelStart, modelEnd)
              << " codegen_load=" << msSince(codegenStart, codegenEnd)
              << " build_problem=" << msSince(buildStart, buildEnd)
              << " raw_codegen_linearization=" << rawCodegenLinearizationMs
              << " solver_setup=" << msSince(setupStart, setupEnd)
              << " proxddp_solve=" << msSince(solveStart, solveEnd)
              << " proxddp_derivatives=" << 1000.0 * solver.derivatives_time_
              << " proxddp_ddp=" << 1000.0 * solver.ddp_time_
              << " sqp_total=" << sqpResult.totalMs
              << " sqp_scope=" << msSince(sqpStart, sqpEnd)
              << " sqp_lq=" << sqpResult.benchmarks.linearQuadraticApproximationTime
              << " sqp_qp=" << sqpResult.benchmarks.solveQpTime
              << " sqp_linesearch=" << sqpResult.benchmarks.linesearchTime
              << " sqp_controller=" << sqpResult.benchmarks.computeControllerTime << "\n";
    std::cout << std::scientific
              << "[" << label << "] proxddp"
              << " run_return=" << converged
              << " converged=" << solver.results_.conv
              << " iter=" << solver.results_.num_iters
              << " al_iter=" << solver.results_.al_iter
              << " cost=" << solver.results_.traj_cost_
              << " merit=" << solver.results_.merit_value_
              << " prim=" << solver.results_.prim_infeas
              << " dual=" << solver.results_.dual_infeas << "\n";
    std::cout << std::scientific
              << "[" << label << "] ocs2_sqp"
              << " iter=" << sqpResult.iterations
              << " cost=" << sqpResult.performance.cost
              << " merit=" << sqpResult.performance.merit
              << " dyn_sse=" << sqpResult.performance.dynamicsViolationSSE
              << " eq_sse=" << sqpResult.performance.equalityConstraintsSSE
              << " ineq_sse=" << sqpResult.performance.inequalityConstraintsSSE << "\n";
    std::cout << std::scientific
              << "[" << label << "] strict_diff"
              << " cost_abs=" << std::abs(solver.results_.traj_cost_ - sqpResult.performance.cost)
              << " first_u_inf=" << maxAbsDiff(u0, sqpU0)
              << " next_x_inf=" << maxAbsDiff(x1, sqpX1)
              << " math_ocp=" << (options.problemMode == ProblemMode::Standard ? "ocs2_standard_collections" : "same")
              << " aligator_frontend=" << frontendModeName(options.frontend)
              << " aligator_rollout=" << rolloutModeName(options.rollout)
              << " ocs2_frontend=sqp_direct" << "\n";
    std::cout << std::fixed << std::setprecision(4)
              << "[" << label << "] first_u=" << vecHeadString(u0, 12) << "\n"
              << "[" << label << "] next_x=" << vecHeadString(x1, 12) << "\n"
              << "[" << label << "] sqp_first_u=" << vecHeadString(sqpU0, 12) << "\n"
              << "[" << label << "] sqp_next_x=" << vecHeadString(sqpX1, 12) << "\n";
    return solver.results_.us.empty() || solver.results_.xs.empty() || sqpResult.solution.inputTrajectory_.empty() ? 2 : 0;
  } catch (const std::exception& e) {
    std::cerr << "[proxddp_centroidal] ERROR: " << e.what() << "\n";
    return 1;
  }
}

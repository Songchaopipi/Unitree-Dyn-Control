#include "centroidal_mpc_mujoco_utils.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <Eigen/Geometry>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_common_mpc/command/WalkingVelocityCommand.h>
#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

namespace wb_humanoid::centroidal_mpc_mujoco {
namespace {

constexpr double kMrtKp = 1200.0;
constexpr double kMrtKd = 10.0;
constexpr double kFixedJointKp = 100.0;
constexpr double kFixedJointKd = 1.0;

std::string pathJoin(const std::string& lhs, const std::string& rhs) {
  if (lhs.empty()) {
    return rhs;
  }
  if (lhs.back() == '/') {
    return lhs + rhs;
  }
  return lhs + "/" + rhs;
}

std::unordered_map<std::string, size_t> makeJointIndexMap(const std::vector<std::string>& names) {
  std::unordered_map<std::string, size_t> out;
  for (size_t i = 0; i < names.size(); ++i) {
    out.emplace(names[i], i);
  }
  return out;
}

Eigen::VectorXd mpcJointVectorFromFull(const std::vector<double>& fullJoints, const std::vector<size_t>& mpcToG1) {
  Eigen::VectorXd out(mpcToG1.size());
  for (size_t i = 0; i < mpcToG1.size(); ++i) {
    out[i] = fullJoints[mpcToG1[i]];
  }
  return out;
}

bool containsIndex(const std::vector<size_t>& values, size_t value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

}  // namespace

double JointAction::torque(double q, double qd) const {
  return kp * (qDes - q) + kd * (qdDes - qd) + feedForwardEffort;
}

SimulationStats::SimulationStats()
    : minBaseZ(std::numeric_limits<double>::infinity()),
      maxBaseZ(-std::numeric_limits<double>::infinity()),
      lastPolicyMode(std::numeric_limits<size_t>::max()) {}

void SimulationStats::setInitialBasePosition(const Eigen::Vector3d& position) {
  initialBasePosition = position;
  finalBasePosition = position;
}

void SimulationStats::update(const mujoco_sim::RobotState& robotState,
                             const std::vector<double>& rawTorques,
                             const std::vector<double>& appliedTorques,
                             const std::vector<JointAction>& actions) {
  const double baseZ = robotState.basePosition.z();
  const Eigen::Vector3d euler = mujoco_sim::quatToEulerZyx(robotState.baseQuat);
  finalBasePosition = robotState.basePosition;
  minBaseZ = std::min(minBaseZ, baseZ);
  maxBaseZ = std::max(maxBaseZ, baseZ);
  maxAbsRollPitch = std::max({maxAbsRollPitch, std::abs(euler.y()), std::abs(euler.z())});
  for (double tau : rawTorques) {
    maxAbsRawTorque = std::max(maxAbsRawTorque, std::abs(tau));
  }
  for (double tau : appliedTorques) {
    maxAbsAppliedTorque = std::max(maxAbsAppliedTorque, std::abs(tau));
  }
  for (size_t i = 0; i < actions.size(); ++i) {
    maxAbsJointPositionError = std::max(maxAbsJointPositionError, std::abs(actions[i].qDes - robotState.jointPosition[i]));
  }
}

void SimulationStats::recordPolicyMode(size_t mode) {
  if (mode < policyModeCounts.size()) {
    ++policyModeCounts[mode];
  }
  if (lastPolicyMode != std::numeric_limits<size_t>::max() && lastPolicyMode != mode) {
    ++policyModeTransitions;
  }
  lastPolicyMode = mode;
}

std::string defaultProjectRoot() {
#ifdef WB_HUMANOID_MPC_ROOT
  return WB_HUMANOID_MPC_ROOT;
#else
  return std::filesystem::current_path().string();
#endif
}

DemoOptions parseDemoOptions(int argc, const char** argv) {
  DemoOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--precompile-only") {
      options.precompileOnly = true;
      options.headless = true;
    } else if (arg == "--headless") {
      options.headless = true;
    } else if (arg == "--walk") {
      options.enableWalking = true;
      options.commandedVx = 0.2;
    } else if (arg == "--vx" && i + 1 < argc) {
      options.enableWalking = true;
      options.commandedVx = std::stod(argv[++i]);
    } else if (arg == "--vy" && i + 1 < argc) {
      options.enableWalking = true;
      options.commandedVy = std::stod(argv[++i]);
    } else if (arg == "--yaw" && i + 1 < argc) {
      options.enableWalking = true;
      options.commandedYaw = std::stod(argv[++i]);
    } else if (arg == "--time" && i + 1 < argc) {
      options.simEndTime = std::stod(argv[++i]);
    } else if (arg == "--warmup-solves" && i + 1 < argc) {
      options.warmupSolves = std::stoi(argv[++i]);
    }
  }
  return options;
}

DemoPaths makeDemoPaths(const std::string& projectRoot) {
  return {
      pathJoin(projectRoot, "config/g1_centroidal/mpc/task.info"),
      pathJoin(projectRoot, "config/g1_centroidal/command/reference.info"),
      pathJoin(projectRoot, "config/g1_centroidal/command/gait.info"),
      pathJoin(projectRoot, "models/g1_description/urdf/g1_29dof.urdf"),
      pathJoin(projectRoot, "models/g1_description/urdf/g1_29dof.xml"),
  };
}

std::vector<double> makeG1StandPose() {
  std::vector<double> q(mujoco_sim::g1JointNames().size(), 0.0);
  q[0] = -0.05;
  q[3] = 0.10;
  q[4] = -0.05;
  q[6] = -0.05;
  q[9] = 0.10;
  q[10] = -0.05;
  return q;
}

std::vector<size_t> makeMpcToG1Map(const ocs2::humanoid::ModelSettings& settings) {
  const auto g1Index = makeJointIndexMap(mujoco_sim::g1JointNames());
  std::vector<size_t> map(settings.mpcModelJointNames.size());
  for (size_t i = 0; i < settings.mpcModelJointNames.size(); ++i) {
    const auto it = g1Index.find(settings.mpcModelJointNames[i]);
    if (it == g1Index.end()) {
      throw std::runtime_error("MPC joint is missing in G1 joint order: " + settings.mpcModelJointNames[i]);
    }
    map[i] = it->second;
  }
  return map;
}

void writeInitialStateToMujoco(const ocs2::vector_t& initialState,
                               const ocs2::humanoid::CentroidalMpcRobotModel<ocs2::scalar_t>& mpcRobotModel,
                               const std::vector<size_t>& mpcToG1,
                               mujoco_sim::MujocoSimInterface& simInterface) {
  const ocs2::vector_t initialBasePose = mpcRobotModel.getBasePose(initialState);
  const Eigen::Vector3d basePosition = initialBasePose.head<3>();
  const Eigen::Quaterniond quat = mujoco_sim::eulerZyxToQuat(initialBasePose[3], initialBasePose[4], initialBasePose[5]);

  const ocs2::vector_t initialMpcJointAngles = mpcRobotModel.getJointAngles(initialState);
  std::vector<double> initialFullJointAngles(mujoco_sim::g1JointNames().size(), 0.0);
  for (size_t i = 0; i < mpcToG1.size(); ++i) {
    initialFullJointAngles[mpcToG1[i]] = initialMpcJointAngles[i];
  }
  simInterface.setFloatingBaseAndJoints(basePosition, quat, initialFullJointAngles);
}

ocs2::TargetTrajectories currentObservationToResetTrajectory(
    const ocs2::SystemObservation& currentObservation,
    const ocs2::humanoid::CentroidalMpcRobotModel<ocs2::scalar_t>& mpcRobotModel) {
  ocs2::vector_t targetState = currentObservation.state;
  targetState.tail(mpcRobotModel.getGenCoordinatesDim()) = ocs2::vector_t::Zero(mpcRobotModel.getGenCoordinatesDim());
  targetState.segment<2>(4) = ocs2::vector_t::Zero(2);
  return ocs2::TargetTrajectories({currentObservation.time},
                                  {targetState},
                                  {ocs2::vector_t::Zero(currentObservation.input.size())});
}

ocs2::SystemObservation makeObservation(const mujoco_sim::RobotState& robotState,
                                        ocs2::humanoid::CentroidalMpcInterface& interface,
                                        const std::vector<size_t>& mpcToG1) {
  auto& pinocchioInterface = interface.getPinocchioInterface();
  const auto& info = interface.getCentroidalModelInfo();
  const auto& mpcRobotModel = interface.getMpcRobotModel();

  const Eigen::Vector3d eulerZyx = mujoco_sim::quatToEulerZyx(robotState.baseQuat);

  ocs2::vector_t qPinocchio(info.generalizedCoordinatesNum);
  qPinocchio.head<3>() = robotState.basePosition;
  qPinocchio.segment<3>(3) = eulerZyx;
  qPinocchio.tail(mpcRobotModel.getJointDim()) = mpcJointVectorFromFull(robotState.jointPosition, mpcToG1);

  ocs2::vector_t vPinocchio(info.generalizedCoordinatesNum);
  vPinocchio.head<3>() = robotState.baseLinearVelocityWorld;
  vPinocchio.segment<3>(3) =
      ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(eulerZyx, robotState.baseAngularVelocityLocal);
  vPinocchio.tail(mpcRobotModel.getJointDim()) = mpcJointVectorFromFull(robotState.jointVelocity, mpcToG1);

  ocs2::vector_t state = ocs2::vector_t::Zero(info.stateDim);
  ocs2::centroidal_model::getGeneralizedCoordinates(state, info) = qPinocchio;
  ocs2::updateCentroidalDynamics(pinocchioInterface, info, qPinocchio);
  const auto& A = ocs2::getCentroidalMomentumMatrix(pinocchioInterface);
  ocs2::centroidal_model::getNormalizedMomentum(state, info).noalias() = A * vPinocchio / info.robotMass;

  ocs2::SystemObservation observation;
  observation.time = robotState.time;
  observation.state = std::move(state);
  observation.input = ocs2::vector_t::Zero(mpcRobotModel.getInputDim());
  observation.input.tail(mpcRobotModel.getJointDim()) = mpcJointVectorFromFull(robotState.jointVelocity, mpcToG1);
  observation.mode = ocs2::humanoid::STANCE;
  return observation;
}

ocs2::humanoid::WalkingVelocityCommand makeWalkingVelocityCommand(const DemoOptions& options, double time) {
  const bool active = options.enableWalking && time > 2.0;
  const double rawVx = active ? std::clamp(options.commandedVx / 2.4, -1.0, 1.0) : 0.0;
  const double rawVy = active ? std::clamp(options.commandedVy / 1.2, -1.0, 1.0) : 0.0;
  const double rawYaw = active ? std::clamp(options.commandedYaw, -1.0, 1.0) : 0.0;
  return ocs2::humanoid::WalkingVelocityCommand(rawVx, rawVy, kDefaultBaseHeight, rawYaw);
}

std::vector<JointAction> makeHoldActions(const std::vector<double>& holdPose) {
  std::vector<JointAction> actions(mujoco_sim::g1JointNames().size());
  for (size_t i = 0; i < actions.size(); ++i) {
    actions[i].qDes = holdPose[i];
    actions[i].qdDes = 0.0;
    actions[i].kp = kFixedJointKp;
    actions[i].kd = kFixedJointKd;
  }
  return actions;
}

void advanceMpcAndPrintTiming(ocs2::MPC_MRT_Interface& mrt, double observationTime, const std::string& label) {
  const auto startTime = std::chrono::steady_clock::now();
  mrt.advanceMpc();
  const auto endTime = std::chrono::steady_clock::now();
  const double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

  const auto oldFlags = std::cout.flags();
  const auto oldPrecision = std::cout.precision();
  std::cout << std::fixed << std::setprecision(3)
            << "[nmpc_timing] " << label
            << " t=" << observationTime << "s total=" << elapsedMs << " ms" << std::endl;
  std::cout.flags(oldFlags);
  std::cout.precision(oldPrecision);
}

std::vector<JointAction> makeMrtActions(const ocs2::vector_t& mpcJointPosition,
                                        const ocs2::vector_t& mpcJointVelocity,
                                        const ocs2::vector_t& mpcJointTorque,
                                        const std::vector<size_t>& mpcToG1) {
  std::vector<JointAction> actions(mujoco_sim::g1JointNames().size());
  for (size_t i = 0; i < mpcToG1.size(); ++i) {
    const size_t id = mpcToG1[i];
    actions[id].qDes = mpcJointPosition[i];
    actions[id].qdDes = mpcJointVelocity[i];
    actions[id].kp = kMrtKp;
    actions[id].kd = kMrtKd;
    actions[id].feedForwardEffort = mpcJointTorque[i];
  }

  for (size_t id = 0; id < mujoco_sim::g1JointNames().size(); ++id) {
    if (!containsIndex(mpcToG1, id)) {
      actions[id].qDes = 0.0;
      actions[id].qdDes = 0.0;
      actions[id].kp = kFixedJointKp;
      actions[id].kd = kFixedJointKd;
      actions[id].feedForwardEffort = 0.0;
    }
  }
  return actions;
}

std::vector<JointAction> makeMrtActionsFromPolicy(double policyTime,
                                                  const ocs2::SystemObservation& observation,
                                                  ocs2::MPC_MRT_Interface& mrt,
                                                  ocs2::humanoid::CentroidalMpcInterface& interface,
                                                  const std::vector<size_t>& mpcToG1,
                                                  SimulationStats& stats) {
  ocs2::vector_t policyState;
  ocs2::vector_t policyInput;
  size_t policyMode = ocs2::humanoid::STANCE;
  mrt.evaluatePolicy(policyTime, observation.state, policyState, policyInput, policyMode);
  stats.recordPolicyMode(policyMode);

  const auto& robotModel = interface.getMpcRobotModel();
  const ocs2::vector_t qDes = robotModel.getJointAngles(policyState);
  const ocs2::vector_t qdDes = robotModel.getJointVelocities(policyState, policyInput);
  const ocs2::vector_t qCur = robotModel.getGeneralizedCoordinates(observation.state);
  auto& mutableRobotModel = const_cast<ocs2::humanoid::CentroidalMpcRobotModel<ocs2::scalar_t>&>(robotModel);
  const ocs2::vector_t qdCur = mutableRobotModel.getGeneralizedVelocities(observation.state, observation.input);
  const ocs2::vector_t qddJointDes = ocs2::vector_t::Zero(robotModel.getJointDim());
  const std::array<ocs2::vector6_t, 2> footWrenches{
      robotModel.getContactWrench(policyInput, 0),
      robotModel.getContactWrench(policyInput, 1)};
  const ocs2::vector_t tauMpc =
      ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(qCur, qdCur, qddJointDes, footWrenches, interface.getPinocchioInterface());
  return makeMrtActions(qDes, qdDes, tauMpc, mpcToG1);
}

std::vector<double> evaluateTorques(const mujoco_sim::RobotState& robotState, const std::vector<JointAction>& actions) {
  std::vector<double> torques(actions.size(), 0.0);
  for (size_t i = 0; i < actions.size(); ++i) {
    torques[i] = actions[i].torque(robotState.jointPosition[i], robotState.jointVelocity[i]);
  }
  return torques;
}

void printAnkleTorques(double time, const std::vector<double>& rawTorques, const std::vector<double>& appliedTorques) {
  static constexpr std::array<size_t, 4> kAnkleJointIndices{4, 5, 10, 11};
  static constexpr std::array<const char*, 4> kAnkleJointShortNames{
      "left_ankle_pitch", "left_ankle_roll", "right_ankle_pitch", "right_ankle_roll"};

  const auto oldFlags = std::cout.flags();
  const auto oldPrecision = std::cout.precision();
  std::cout << std::fixed << std::setprecision(4)
            << "[ankle_torque] t=" << time << "s applied(raw) Nm:";
  for (size_t i = 0; i < kAnkleJointIndices.size(); ++i) {
    const size_t jointIndex = kAnkleJointIndices[i];
    if (jointIndex < appliedTorques.size() && jointIndex < rawTorques.size()) {
      std::cout << " " << kAnkleJointShortNames[i]
                << "=" << appliedTorques[jointIndex]
                << "(" << rawTorques[jointIndex] << ")";
    }
  }
  std::cout << std::endl;
  std::cout.flags(oldFlags);
  std::cout.precision(oldPrecision);
}

bool stateLooksInvalid(const mujoco_sim::RobotState& robotState, const std::vector<double>& torques, std::string& reason) {
  const Eigen::Vector3d euler = mujoco_sim::quatToEulerZyx(robotState.baseQuat);
  const double baseZ = robotState.basePosition.z();
  if (!std::isfinite(baseZ) || !std::isfinite(euler.y()) || !std::isfinite(euler.z())) {
    reason = "non-finite base state";
    return true;
  }
  if (baseZ < 0.25 || baseZ > 1.20) {
    reason = "base height out of validation range: z=" + std::to_string(baseZ);
    return true;
  }
  if (std::abs(euler.y()) > 0.70 || std::abs(euler.z()) > 0.70) {
    reason = "base roll/pitch out of validation range";
    return true;
  }
  for (double tau : torques) {
    if (!std::isfinite(tau)) {
      reason = "non-finite joint torque";
      return true;
    }
    if (std::abs(tau) > 600.0) {
      reason = "joint torque out of validation range: tau=" + std::to_string(tau);
      return true;
    }
  }
  return false;
}

void printStats(const SimulationStats& stats) {
  const Eigen::Vector3d displacement = stats.finalBasePosition - stats.initialBasePosition;
  std::cout << "[centroidal_mpc_mujoco] validation stats: min_z=" << stats.minBaseZ
            << ", max_z=" << stats.maxBaseZ
            << ", max_abs_roll_pitch=" << stats.maxAbsRollPitch
            << ", max_abs_raw_tau=" << stats.maxAbsRawTorque
            << ", max_abs_applied_tau=" << stats.maxAbsAppliedTorque
            << ", max_abs_q_err=" << stats.maxAbsJointPositionError
            << ", displacement_xy=(" << displacement.x() << ", " << displacement.y() << ")"
            << ", policy_modes={FLY:" << stats.policyModeCounts[ocs2::humanoid::FLY]
            << ", RF:" << stats.policyModeCounts[ocs2::humanoid::RF]
            << ", LF:" << stats.policyModeCounts[ocs2::humanoid::LF]
            << ", STANCE:" << stats.policyModeCounts[ocs2::humanoid::STANCE]
            << "}, policy_mode_transitions=" << stats.policyModeTransitions << std::endl;
}

}  // namespace wb_humanoid::centroidal_mpc_mujoco

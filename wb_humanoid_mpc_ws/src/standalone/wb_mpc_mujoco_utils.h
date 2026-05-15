#pragma once

#include "mj_sim_interface.h"

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_wb_mpc/mrt/WBMpcMrtJointController.h>

namespace ocs2 {
class MPC_MRT_Interface;

namespace humanoid {
class WBMpcInterface;
class WalkingVelocityCommand;
template <typename SCALAR_T>
class WBAccelMpcRobotModel;
}  // namespace humanoid
}  // namespace ocs2

namespace wb_humanoid::wb_mpc_mujoco {

constexpr double kSimDt = mujoco_sim::kDefaultSimDt;
constexpr double kMrtDt = 0.002;
constexpr double kPolicyLookahead = 0.005;
constexpr double kDefaultBaseHeight = 0.7925;
constexpr double kDefaultJointDamping = 10.0;
constexpr double kAnkleTorquePrintDt = 0.02;

struct DemoOptions {
  bool precompileOnly = false;
  bool enableWalking = false;
  bool headless = false;
  bool threadedMpc = false;
  int warmupSolves = 8;
  double commandedVx = 0.0;
  double commandedVy = 0.0;
  double commandedYaw = 0.0;
  double simEndTime = 0.05;
};

struct DemoPaths {
  std::string taskFile;
  std::string referenceFile;
  std::string gaitFile;
  std::string urdfFile;
  std::string mjcfFile;
};

struct SimulationStats {
  Eigen::Vector3d initialBasePosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d finalBasePosition = Eigen::Vector3d::Zero();
  double minBaseZ = 0.0;
  double maxBaseZ = 0.0;
  double maxAbsRollPitch = 0.0;
  double maxAbsRawTorque = 0.0;
  double maxAbsAppliedTorque = 0.0;
  double maxAbsJointPositionError = 0.0;
  std::array<size_t, 4> policyModeCounts{0, 0, 0, 0};
  size_t policyModeTransitions = 0;
  size_t lastPolicyMode = 0;

  SimulationStats();
  void setInitialBasePosition(const Eigen::Vector3d& position);
  void update(const mujoco_sim::RobotState& robotState,
              const std::vector<double>& rawTorques,
              const std::vector<double>& appliedTorques,
              const std::vector<ocs2::humanoid::WBMpcJointAction>& actions);
  void recordPolicyMode(size_t mode);
};

std::string defaultProjectRoot();
DemoOptions parseDemoOptions(int argc, const char** argv);
DemoPaths makeDemoPaths(const std::string& projectRoot);

std::vector<double> makeG1StandPose();
std::vector<size_t> makeMpcToG1Map(const ocs2::humanoid::ModelSettings& settings);

void writeInitialStateToMujoco(const ocs2::vector_t& initialState,
                               const ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t>& mpcRobotModel,
                               const std::vector<size_t>& mpcToG1,
                               mujoco_sim::MujocoSimInterface& simInterface);

ocs2::TargetTrajectories currentObservationToResetTrajectory(
    const ocs2::SystemObservation& currentObservation,
    const ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t>& mpcRobotModel);

ocs2::SystemObservation makeObservation(const mujoco_sim::RobotState& robotState,
                                        ocs2::humanoid::WBMpcInterface& interface,
                                        const std::vector<size_t>& mpcToG1);

ocs2::humanoid::WBMpcRobotState makeWbRobotState(const mujoco_sim::RobotState& robotState,
                                                const ocs2::humanoid::ModelSettings& settings);

ocs2::humanoid::WalkingVelocityCommand makeWalkingVelocityCommand(const DemoOptions& options, double time);

std::vector<ocs2::humanoid::WBMpcJointAction> makeHoldActions(const std::vector<double>& holdPose);
void advanceMpcAndPrintTiming(ocs2::MPC_MRT_Interface& mrt, double observationTime, const std::string& label);
std::vector<ocs2::humanoid::WBMpcJointAction> makeMrtActionsFromPolicy(double policyTime,
                                                                       const ocs2::SystemObservation& observation,
                                                                       ocs2::MPC_MRT_Interface& mrt,
                                                                       ocs2::humanoid::WBMpcInterface& interface,
                                                                       const std::vector<size_t>& mpcToG1,
                                                                       SimulationStats& stats);
std::vector<double> evaluateTorques(const mujoco_sim::RobotState& robotState,
                                    const std::vector<ocs2::humanoid::WBMpcJointAction>& actions);

void printAnkleTorques(double time, const std::vector<double>& rawTorques, const std::vector<double>& appliedTorques);
bool stateLooksInvalid(const mujoco_sim::RobotState& robotState, const std::vector<double>& torques, std::string& reason);
void printStats(const SimulationStats& stats);

}  // namespace wb_humanoid::wb_mpc_mujoco

#include "centroidal_mpc_mujoco_utils.h"
#include "mj_sim_interface.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_sqp/SqpMpc.h>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_common_mpc/command/WalkingVelocityCommand.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>

namespace demo = wb_humanoid::centroidal_mpc_mujoco;
namespace mj_sim = wb_humanoid::mujoco_sim;

int main(int argc, const char** argv) {
  const std::string projectRoot = demo::defaultProjectRoot();
  std::filesystem::current_path(projectRoot);

  const demo::DemoOptions options = demo::parseDemoOptions(argc, argv);
  const demo::DemoPaths paths = demo::makeDemoPaths(projectRoot);

  try {
    mj_sim::MujocoModelData mujoco(paths.mjcfFile, demo::kSimDt);
    mj_sim::MujocoSimInterface simInterface(mujoco.model(), mujoco.data());
    simInterface.applyDefaultJointDamping(demo::kDefaultJointDamping);

    ocs2::humanoid::CentroidalMpcInterface interface(paths.taskFile, paths.urdfFile, paths.referenceFile);
    ocs2::SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());

    ocs2::humanoid::CentroidalMpcTargetTrajectoriesCalculator targetCalculator(
        paths.referenceFile,
        interface.getMpcRobotModel(),
        interface.getPinocchioInterface(),
        interface.getCentroidalModelInfo(),
        interface.mpcSettings().timeHorizon_);

    auto targetTrajectoriesFunc =
        [&targetCalculator](const ocs2::vector4_t& velocityTarget,
                            ocs2::scalar_t initTime,
                            ocs2::scalar_t finalTime,
                            const ocs2::vector_t& initState) mutable {
          (void)finalTime;
          return targetCalculator.commandedVelocityToTargetTrajectories(velocityTarget, initTime, initState);
        };

    auto motionManager = std::make_shared<ocs2::humanoid::ProceduralMpcMotionManager>(
        paths.gaitFile,
        paths.referenceFile,
        interface.getSwitchedModelReferenceManagerPtr(),
        interface.getMpcRobotModel(),
        targetTrajectoriesFunc);

    mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
    mpc.getSolverPtr()->addSynchronizedModule(motionManager);

    ocs2::MPC_MRT_Interface mrt(mpc);
    mrt.initRollout(&interface.getRollout());

    const std::vector<size_t> mpcToG1 = demo::makeMpcToG1Map(interface.modelSettings());
    const std::vector<double> standPose = demo::makeG1StandPose();
    std::cout << "[centroidal_mpc_mujoco] controller: inverse dynamics torque mapper" << std::endl;

    demo::writeInitialStateToMujoco(interface.getInitialState(), interface.getMpcRobotModel(), mpcToG1, simInterface);
    mj_sim::RobotState robotState = simInterface.readState();

    ocs2::SystemObservation observation = demo::makeObservation(robotState, interface, mpcToG1);
    mrt.setCurrentObservation(observation);
    mrt.resetMpcNode(demo::currentObservationToResetTrajectory(observation, interface.getMpcRobotModel()));

    motionManager->setAndScaleVelocityCommand(ocs2::humanoid::WalkingVelocityCommand(0.0, 0.0, demo::kDefaultBaseHeight, 0.0));
    std::cout << "Running first centroidal NMPC solve..." << std::endl;
    demo::advanceMpcAndPrintTiming(mrt, observation.time, "first");
    mrt.updatePolicy();

    for (int i = 0; i < options.warmupSolves; ++i) {
      mrt.setCurrentObservation(observation);
      demo::advanceMpcAndPrintTiming(mrt, observation.time, "warmup#" + std::to_string(i + 1));
      if (!mrt.updatePolicy()) {
        throw std::runtime_error("MPC warmup failed to publish a policy");
      }
    }

    if (options.precompileOnly) {
      std::cout << "[centroidal_mpc_mujoco] precompile/load-only path completed. base_z="
                << robotState.basePosition.z() << std::endl;
      return 0;
    }

    double nextMrtTime = 0.0;
    double nextMpcTime = 0.0;
    double nextAnkleTorquePrintTime = 0.0;
    const double mpcUpdateDt = interface.mpcSettings().mpcDesiredFrequency_ > 0.0
                                   ? 1.0 / interface.mpcSettings().mpcDesiredFrequency_
                                   : 0.0;
    std::vector<demo::JointAction> jointActions = demo::makeHoldActions(standPose);
    demo::SimulationStats stats;
    stats.setInitialBasePosition(robotState.basePosition);

    std::unique_ptr<mj_sim::MujocoViewer> viewer;
    if (!options.headless) {
      viewer = std::make_unique<mj_sim::MujocoViewer>(mujoco.model(), mujoco.data());
      viewer->initialize();
      viewer->render(true);
    }

    while (simInterface.time() <= options.simEndTime && (viewer == nullptr || !viewer->shouldClose())) {
      if (viewer != nullptr && !viewer->simulationActive()) {
        viewer->render(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }

      robotState = simInterface.readState();
      observation = demo::makeObservation(robotState, interface, mpcToG1);
      mrt.setCurrentObservation(observation);
      motionManager->setAndScaleVelocityCommand(demo::makeWalkingVelocityCommand(options, robotState.time));

      if (mpcUpdateDt > 0.0 && robotState.time + 0.5 * demo::kSimDt >= nextMpcTime) {
        demo::advanceMpcAndPrintTiming(mrt, robotState.time, "runtime");
        if (!mrt.updatePolicy()) {
          std::cerr << "[centroidal_mpc_mujoco] MPC did not publish a new policy at t=" << robotState.time << std::endl;
        }
        nextMpcTime += mpcUpdateDt;
      }

      if (robotState.time + 0.5 * demo::kSimDt >= nextMrtTime && mrt.initialPolicyReceived()) {
        jointActions = demo::makeMrtActionsFromPolicy(robotState.time + demo::kPolicyLookahead,
                                                      observation,
                                                      mrt,
                                                      interface,
                                                      mpcToG1,
                                                      stats);
        nextMrtTime += demo::kMrtDt;
      } else if (!mrt.initialPolicyReceived()) {
        jointActions = demo::makeHoldActions(standPose);
      }

      const std::vector<double> rawTorques = demo::evaluateTorques(robotState, jointActions);
      const std::vector<double> appliedTorques = simInterface.clampJointTorques(rawTorques);
      if (robotState.time + 0.5 * demo::kSimDt >= nextAnkleTorquePrintTime) {
        demo::printAnkleTorques(robotState.time, rawTorques, appliedTorques);
        nextAnkleTorquePrintTime += demo::kAnkleTorquePrintDt;
      }

      stats.update(robotState, rawTorques, appliedTorques, jointActions);
      std::string invalidReason;
      if (demo::stateLooksInvalid(robotState, appliedTorques, invalidReason)) {
        std::cerr << "[centroidal_mpc_mujoco] validation failed at t=" << robotState.time << ": " << invalidReason << std::endl;
        demo::printStats(stats);
        return 2;
      }

      simInterface.setJointTorques(appliedTorques);
      simInterface.step();
      if (viewer != nullptr) {
        viewer->finishStep();
        viewer->render();
        viewer->paceToRealTime();
      }
    }

    if (viewer != nullptr) {
      viewer->render(true);
    }
    demo::printStats(stats);
  } catch (const std::exception& e) {
    std::cerr << "centroidal_mpc_mujoco failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

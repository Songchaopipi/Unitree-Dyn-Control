/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "humanoid_wb_mpc/mrt/WBMpcMrtJointController.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>
#include "humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {

WBMpcMrtJointController::WBMpcMrtJointController(const ModelSettings& modelSettings,
                                                 MPC_BASE& mpc,
                                                 PinocchioInterface pinocchioInterface,
                                                 scalar_t mpcDesiredFrequency)
    : mcpMrtInterface_(mpc),
      pinocchioInterface_(pinocchioInterface),
      mpcRobotModel_(modelSettings),
      mpcDeltaTMicroSeconds_(mpcDesiredFrequency > 0 ? static_cast<size_t>(1000000 / mpcDesiredFrequency) : 0),
      realtime_(mpcDesiredFrequency <= 0) {
  mpcJointIndices_ = modelSettings.mpcModelToFullJointsIndices;
  otherJointIndices_.reserve(modelSettings.fixedJointNames.size());
  for (const auto& fixedJointName : modelSettings.fixedJointNames) {
    const auto it = std::find(modelSettings.fullJointNames.begin(), modelSettings.fullJointNames.end(), fixedJointName);
    if (it == modelSettings.fullJointNames.end()) {
      throw std::invalid_argument("Fixed joint is missing from full joint list: " + fixedJointName);
    }
    otherJointIndices_.push_back(static_cast<size_t>(std::distance(modelSettings.fullJointNames.begin(), it)));
  }
  currentMpcObservation_.state = vector_t::Zero(mpcRobotModel_.getStateDim());
  currentMpcObservation_.input = vector_t::Zero(mpcRobotModel_.getInputDim());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

WBMpcMrtJointController::~WBMpcMrtJointController() {
  // Signal the solver thread to terminate
  terminateThread_.store(true);

  // Wait for the solver thread to finish if it's joinable
  if (solver_worker_.joinable()) {
    solver_worker_.join();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void WBMpcMrtJointController::startMpcThread(const WBMpcRobotState& initRobotState) {
  terminateThread_.store(false);
  updateMpcObservation(currentMpcObservation_, initRobotState);
  // Set observation to MPC
  mcpMrtInterface_.setCurrentObservation(currentMpcObservation_);
  solver_worker_ = std::jthread(&WBMpcMrtJointController::solverWorker, this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void WBMpcMrtJointController::updateMpcState(vector_t& mpcState, const WBMpcRobotState& robotState) {
  if (static_cast<size_t>(robotState.jointPositions.size()) < mpcRobotModel_.getFullModelJointDim()) {
    throw std::invalid_argument("WBMpcRobotState has fewer joint positions than the full model joint dimension");
  }
  if (static_cast<size_t>(robotState.jointVelocities.size()) < mpcRobotModel_.getFullModelJointDim()) {
    throw std::invalid_argument("WBMpcRobotState has fewer joint velocities than the full model joint dimension");
  }

  mpcRobotModel_.setBasePosition(mpcState, robotState.basePositionWorld);
  mpcRobotModel_.setBaseOrientationEulerZYX(mpcState, quaternionToEulerZYX(robotState.baseOrientationLocalToWorld));

  vector_t mpcJointPositions(mpcJointIndices_.size());
  vector_t mpcJointVelocities(mpcJointIndices_.size());
  for (size_t i = 0; i < mpcJointIndices_.size(); ++i) {
    mpcJointPositions[i] = robotState.jointPositions[mpcJointIndices_[i]];
    mpcJointVelocities[i] = robotState.jointVelocities[mpcJointIndices_[i]];
  }
  mpcRobotModel_.setJointAngles(mpcState, mpcJointPositions);

  // currently we send local angular and linear velocity
  mpcRobotModel_.setBaseLinearVelocity(mpcState, robotState.baseLinearVelocityWorld);
  mpcRobotModel_.setBaseOrientationEulerZYXDerivatives(
      mpcState, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(mpcRobotModel_.getBaseOrientationEulerZYX(mpcState),
                                                                               robotState.baseAngularVelocityLocal));

  vector_t dummyInput = vector_t::Zero(mpcRobotModel_.getInputDim());
  mpcRobotModel_.setJointVelocities(mpcState, dummyInput, mpcJointVelocities);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void WBMpcMrtJointController::updateMpcObservation(ocs2::SystemObservation& mpcObservation, const WBMpcRobotState& robotState) {
  updateMpcState(mpcObservation.state, robotState);
  mpcObservation.time = robotState.time;
  mpcObservation.input = vector_t::Zero(mpcRobotModel_.getInputDim());  // Add contact forces later.
  mpcObservation.mode = stanceLeg2ModeNumber(robotState.contactFlags);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<WBMpcJointAction> WBMpcMrtJointController::computeJointControlAction(const WBMpcRobotState& robotState) {
  std::vector<WBMpcJointAction> robotJointAction(mpcRobotModel_.getFullModelJointDim());
  computeJointControlAction(robotState, robotJointAction);
  return robotJointAction;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void WBMpcMrtJointController::computeJointControlAction(const WBMpcRobotState& robotState,
                                                        std::vector<WBMpcJointAction>& robotJointAction) {
  if (robotJointAction.size() != mpcRobotModel_.getFullModelJointDim()) {
    robotJointAction.assign(mpcRobotModel_.getFullModelJointDim(), WBMpcJointAction{});
  }

  updateMpcObservation(currentMpcObservation_, robotState);
  // Set observation to MPC
  mcpMrtInterface_.setCurrentObservation(currentMpcObservation_);

  vector_t mpcPolicyState;
  vector_t mpcPolicyInput;
  size_t mpcPolicyMode;

  if (mcpMrtInterface_.initialPolicyReceived()) {
    // Evaluate policy with feedback if activated in config
    mcpMrtInterface_.evaluatePolicy(currentMpcObservation_.time + 0.005, currentMpcObservation_.state, mpcPolicyState, mpcPolicyInput,
                                    mpcPolicyMode);

    vector_t mpcJointTorques = computeJointTorques<scalar_t>(mpcPolicyState, mpcPolicyInput, pinocchioInterface_, mpcRobotModel_);
    vector_t mpc_q_desired = mpcRobotModel_.getJointAngles(mpcPolicyState);
    vector_t mpc_qd_desired = mpcRobotModel_.getJointVelocities(mpcPolicyState, mpcPolicyInput);

    // std::cout << "mpcJointTorques: " << mpcJointTorques.transpose() << std::endl;

    for (size_t i = 0; i < mpcJointIndices_.size(); i++) {
      size_t index = mpcJointIndices_[i];
      WBMpcJointAction& action = robotJointAction.at(index);

      action.qDes = mpc_q_desired[i];
      action.qdDes = mpc_qd_desired[i];
      action.kp = 1200.0;
      action.kd = 10.0;
      action.feedForwardEffort = mpcJointTorques[i];

      // std::cerr << "MPCtorque!: " << mpcJointTorques[i] << std::endl;
    };
  }

  else {
    std::cerr << "Apply weight compensating torque..." << std::endl;
    //   Apply weight compensated input around current state
    mpcPolicyState = currentMpcObservation_.state;
    mpcPolicyInput = weightCompensatingInput(pinocchioInterface_, {true, true}, mpcRobotModel_);
    vector_t weightCompensatingTorques = computeJointTorques<scalar_t>(mpcPolicyState, mpcPolicyInput, pinocchioInterface_, mpcRobotModel_);

    for (size_t i = 0; i < mpcJointIndices_.size(); i++) {
      size_t index = mpcJointIndices_[i];
      WBMpcJointAction& action = robotJointAction.at(index);

      action.qDes = 0;
      action.qdDes = 0;
      action.kp = 0;
      action.kd = 0;
      action.feedForwardEffort = weightCompensatingTorques[i];
    };
  }

  for (size_t i = 0; i < otherJointIndices_.size(); i++) {
    size_t index = otherJointIndices_[i];
    WBMpcJointAction& action = robotJointAction.at(index);

    action.qDes = 0;
    action.qdDes = 0;
    action.kp = 100;
    action.kd = 1.0;
    action.feedForwardEffort = 0.0;
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void WBMpcMrtJointController::solverWorker() {
  mcpMrtInterface_.resetMpcNode(currentObservationToResetTrajectory(mcpMrtInterface_.getCurrentObservation()));
  std::cerr << "MPC is reset. NMPC solver started!" << std::endl;

  while (!terminateThread_.load()) {
    auto targetTimeForNextIteration = std::chrono::steady_clock::now() + std::chrono::microseconds(mpcDeltaTMicroSeconds_);

    mcpMrtInterface_.advanceMpc();

    // Publish if Policy has been updated
    if (!mcpMrtInterface_.updatePolicy()) {
      std::cerr << "The solver has failed to update!!" << std::endl;
      return;
    }

    // std::cerr << "MPC policy computed!" << std::endl;

    if (!realtime_ && !terminateThread_.load()) {
      auto currentTime = std::chrono::steady_clock::now();
      if (currentTime > targetTimeForNextIteration) {
        auto delay = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - targetTimeForNextIteration).count();

        std::cerr << "Warning: MPC loop running slow by " << delay << " microseconds." << std::endl;
      } else {
        // Sleep in case sim loop is faster than specified
        std::this_thread::sleep_until(targetTimeForNextIteration);
      }
    }
  }
  std::cerr << "Shutting down NMPC" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectories WBMpcMrtJointController::currentObservationToResetTrajectory(const SystemObservation& currentObservation) {
  vector_t targetState = currentObservation.state;

  // zero out velocities
  targetState.tail(mpcRobotModel_.getGenCoordinatesDim()) = vector_t::Zero(mpcRobotModel_.getGenCoordinatesDim());

  // zero out pitch + roll angles
  targetState.segment<2>(4) = vector_t::Zero(2);

  const TargetTrajectories resetTargetTrajectories({currentObservation.time}, {targetState},
                                                   {vector_t::Zero(currentObservation.input.size())});

  std::cerr << "Resetting MPC to current state: \n" << targetState << std::endl;
  return resetTargetTrajectories;
}

}  // namespace ocs2::humanoid

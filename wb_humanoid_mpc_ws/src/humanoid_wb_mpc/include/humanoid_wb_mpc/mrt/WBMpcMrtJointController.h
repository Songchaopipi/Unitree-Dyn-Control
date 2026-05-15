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

#pragma once

#include <atomic>
#include <cstddef>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"

namespace ocs2::humanoid {

struct WBMpcRobotState {
  scalar_t time = 0.0;
  vector3_t basePositionWorld = vector3_t::Zero();
  quaternion_t baseOrientationLocalToWorld = quaternion_t::Identity();
  vector3_t baseLinearVelocityWorld = vector3_t::Zero();
  vector3_t baseAngularVelocityLocal = vector3_t::Zero();
  vector_t jointPositions;
  vector_t jointVelocities;
  contact_flag_t contactFlags{{true, true}};
};

struct WBMpcJointAction {
  scalar_t qDes = 0.0;
  scalar_t qdDes = 0.0;
  scalar_t kp = 0.0;
  scalar_t kd = 0.0;
  scalar_t feedForwardEffort = 0.0;

  scalar_t torque(scalar_t q, scalar_t qd) const { return kp * (qDes - q) + kd * (qdDes - qd) + feedForwardEffort; }
};

class WBMpcMrtJointController final {
 public:
  /**
   * Constructor.
   *
   * @param [in] mpc: The underlying MPC class to be used.
   * @param [in] mpcDesiredFrequency: The max frequency to run the mpc at.
   */
  WBMpcMrtJointController(const ModelSettings& modelSettings,
                          MPC_BASE& mpc,
                          PinocchioInterface pinocchioInterface,
                          scalar_t mpcDesiredFrequency = -1);

  /**
   * Destructor.
   */
  ~WBMpcMrtJointController();

  bool ready() const { return mcpMrtInterface_.initialPolicyReceived(); }

  /**
   * Handles the low level controller loop that updates the mpc observation, reads out the latest policy and sets the joint control action.
   */

  void computeJointControlAction(const WBMpcRobotState& robotState, std::vector<WBMpcJointAction>& robotJointAction);

  std::vector<WBMpcJointAction> computeJointControlAction(const WBMpcRobotState& robotState);

  void startMpcThread(const WBMpcRobotState& initRobotState);

 private:
  /**
   * Handles the MPC solver thread.
   */
  void solverWorker();

  /**
   * Method to convert the latest observation msg to a stable desired trajectory (current position, zero velocity and
   * acceleration)
   *
   * @param [in] msg: The observation message.
   */
  TargetTrajectories currentObservationToResetTrajectory(const SystemObservation& currentMpcObservation);

  void updateMpcState(vector_t& mpcState, const WBMpcRobotState& robotState);
  void updateMpcObservation(ocs2::SystemObservation& mpcObservation, const WBMpcRobotState& robotState);

  MPC_MRT_Interface mcpMrtInterface_;

  PinocchioInterface pinocchioInterface_;
  ocs2::SystemObservation currentMpcObservation_;
  WBAccelMpcRobotModel<scalar_t> mpcRobotModel_;
  std::vector<size_t> mpcJointIndices_;
  std::vector<size_t> otherJointIndices_;

  size_t mpcDeltaTMicroSeconds_;
  bool realtime_;  // True if MPC is to be run as fast as possible

  std::atomic_bool terminateThread_{false};
  std::jthread solver_worker_;
};

}  // namespace ocs2::humanoid

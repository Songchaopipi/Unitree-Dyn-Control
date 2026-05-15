# wb_humanoid_mpc_ws Centroidal OCS2 -> ProxDDP Direct Mapping

This branch now uses the uploaded `wb_humanoid_mpc_ws` source tree as the source of truth for the full OCS2 centroidal MPC problem.

## Source of truth

The full OCS2 centroidal problem is assembled in:

```text
wb_humanoid_mpc_ws/src/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp
```

The factory-created common costs and constraints are implemented in:

```text
wb_humanoid_mpc_ws/src/humanoid_common_mpc/include/humanoid_common_mpc/HumanoidCostConstraintFactory.h
wb_humanoid_mpc_ws/src/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp
```

## OCS2 problem terms found in CentroidalMpcInterface

`CentroidalMpcInterface::setupOptimalControlProblem()` builds the following OCP:

```text
Dynamics:
  CentroidalDynamicsAD

Running costs:
  stateInputQuadraticCost
  task_space_costs.*_TaskSpaceKinematicsCost
  icp_Cost
  <foot>_TaskSpaceKinematicsCost
  <foot>_ExternalTorqueQuadraticCost

Terminal costs:
  terminalCost

State soft constraints:
  jointLimits
  FootCollisionSoftConstraint

State-input soft constraints:
  <foot>_frictionForceCone
  <foot>_contactMomentXY

Equality constraints:
  <foot>_zeroWrench
  <foot>_zeroVelocity
  <foot>_normalVelocity
  <foot>_kneeJointMimic, when mimicJoints exists in task.info

Precomputation:
  HumanoidPreComputation

Reference / contact schedule:
  SwitchedModelReferenceManager
  GaitSchedule
  SwingTrajectoryPlanner
```

## Factory mapping

`HumanoidCostConstraintFactory` provides:

```text
getStateInputQuadraticCost()
  Q, R from task.info
  OCS2 StateInputQuadraticCost
  ProxDDP direct mapping: direct quadratic state/input accumulation

getTerminalCost()
  Q_final * terminalCostScaling
  OCS2 QuadraticStateCost
  ProxDDP direct mapping: terminal direct quadratic state cost

getFrictionForceConeConstraint(i)
  FrictionForceConeConstraint + RelaxedBarrierPenalty
  ProxDDP direct mapping: soft inequality direct penalty

getContactMomentXYConstraint(i, name)
  ContactMomentXYConstraintCppAd + RelaxedBarrierPenalty
  ProxDDP direct mapping: soft inequality direct penalty, using OCS2 codegen value/Jacobian

getZeroWrenchConstraint(i)
  ZeroWrenchConstraint
  ProxDDP direct mapping: packed hard equality rows; active in swing

getExternalTorqueQuadraticCost(i)
  ExternalTorqueQuadraticCostAD
  ProxDDP direct mapping: Gauss-Newton residual packed into direct stage cost

getJointLimitsConstraint()
  JointLimitsSoftConstraint + PieceWisePolynomialBarrierPenalty
  ProxDDP direct mapping: state-only soft penalty in direct cost pack

getFootCollisionConstraint()
  FootCollisionConstraint + PieceWisePolynomialBarrierPenalty
  ProxDDP direct mapping: state-only soft penalty in direct cost pack
```

## ProxDDP implementation target

The final implementation under `humanoid_cd_nmpc_aligator` should not rebuild the OCS2 `OptimalControlProblem` as the hot-path object. It should use the same source objects to construct a packed direct frontend:

```text
DirectCentroidalEulerDynamics
  xnext = x + dt * f_ocs2_codegen(x, u)
  Jx    = I + dt * dfdx
  Ju    = dt * dfdu

DirectCentroidalStageCostPack
  state/input quadratic
  task-space kinematics cost
  ICP cost
  foot cost
  external torque cost
  joint limits soft penalty
  foot collision soft penalty
  friction cone soft penalty
  contact moment soft penalty

DirectCentroidalConstraintPack
  zero wrench equality
  zero velocity equality
  normal velocity equality
  mimic equality, if enabled

ThinCentroidalStageModel
  calls direct dynamics, direct cost pack, and direct constraint pack with minimum dispatch

SolverProxDDP
  keeps rollout, line search, AL updates, and Riccati/DDP
```

## Layer order after reading real wb_humanoid_mpc_ws source

### Layer2A: replace current quadratic direct cost with reusable pack

Use `PackedCostWorkspace` and direct quadratic accumulation, but keep the math identical to current layer1.

Expected result:

```text
strict_diff against current layer1 ~= numerical noise
proxddp_derivatives should not increase
```

### Layer2B: add OCS2 task-space and foot costs as packed Gauss-Newton terms

Terms:

```text
task_space_costs.*_TaskSpaceKinematicsCost
<foot>_TaskSpaceKinematicsCost
```

Implementation should call the same kinematics/codegen objects used by:

```text
EndEffectorKinematicsQuadraticCost
CentroidalMpcEndEffectorFootCost
```

Then merge the residual approximation with:

```text
addGaussNewtonResidual()
```

Do not instantiate Aligator `QuadraticResidualCost`.

### Layer2C: add ICP and external torque cost

Terms:

```text
icp_Cost
<foot>_ExternalTorqueQuadraticCost
```

Implementation should reuse the same OCS2 reference manager and robot model semantics, but expose only a packed residual approximation to the direct cost pack.

### Layer3: packed hard constraints

Terms:

```text
<foot>_zeroWrench
<foot>_zeroVelocity
<foot>_normalVelocity
<foot>_kneeJointMimic, if configured
```

Use one packed hard-constraint data block per stage. Keep active row masks instead of rebuilding the problem when contact mode changes.

### Layer4: direct soft penalties

Terms:

```text
jointLimits
FootCollisionSoftConstraint
<foot>_frictionForceCone
<foot>_contactMomentXY
```

The first implementation should use Gauss-Newton / relaxed-barrier Hessian approximations. Exact second-order terms should be optional and profiling-driven.

## Realtime rule

Do not rebuild for ordinary gait/contact updates.

Update these as parameters:

```text
initial state
state reference
input reference
foot target trajectory
contact mode mask
cost weights
barrier parameters
```

Rebuild only when the structural signature changes:

```text
horizon length
state dimension
input dimension
maximum constraint rows
frontend type
```

## Files added in this branch

```text
include/humanoid_cd_nmpc_aligator/packed_cost_accumulator.hpp
  direct accumulation utilities shared by all layers
```

Next source file to add:

```text
include/humanoid_cd_nmpc_aligator/direct_ocs2_cost_terms.hpp
  wrappers around OCS2 StateInputCost / StateCost approximations
  initial version: stateInputQuadratic + terminal cost only
```

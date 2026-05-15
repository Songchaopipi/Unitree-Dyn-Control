# WB Humanoid MPC Standalone

This workspace contains two ROS2-free G1 MPC pipelines built with plain CMake:

- `centroidal_mpc_mujoco`: centroidal NMPC + MRT + MuJoCo simulation.
- `wb_mpc_mujoco`: whole-body acceleration NMPC + MRT + MuJoCo simulation.

The CMake file still points Pinocchio/urdfdom at `/opt/ros/humble` for ABI
compatibility with the copied OCS2 libraries, but no ROS2, ament, colcon, or
ROS nodes are used at runtime.

## Layout

- `src/humanoid_common_mpc`: shared gait, reference manager, costs, constraints, Pinocchio helpers, and model settings.
- `src/humanoid_centroidal_mpc`: centroidal dynamics/interface/target calculator.
- `src/humanoid_wb_mpc`: whole-body dynamics/interface/target calculator and ROS2-free MRT joint controller.
- `src/standalone`: MuJoCo entry points and simulator glue for both pipelines.
- `config/g1_centroidal`: standalone centroidal task, reference, and gait files.
- `config/g1_wb`: standalone whole-body task, reference, and gait files.
- `models/g1_description`: G1 URDF, MJCF, and meshes used by Pinocchio and MuJoCo.
- `cppad_code_gen`: generated CppAD shared libraries.
- `third_party`: copied OCS2, HPIPM, BLASFEO, and MuJoCo headers/libraries.

## Build

```bash
cd /home/songchao/OPTControl_env/wb_humanoid_mpc_ws
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j1 --target centroidal_mpc_mujoco
cmake --build build -j1 --target wb_mpc_mujoco
```

## Whole-Body Run

First run the load/precompile path. The first WB run can take several minutes
because it generates `cppad_code_gen/cppad_wb_mpc_g1`; later runs reuse the
generated libraries.

```bash
./build/wb_mpc_mujoco --precompile-only --headless
```

Short headless simulation:

```bash
./build/wb_mpc_mujoco --headless --time 0.05
```

Viewer simulation:

```bash
./build/wb_mpc_mujoco --time 1.0
```

Optional walking command starts after 2 seconds:

```bash
./build/wb_mpc_mujoco --time 8.0 --walk
./build/wb_mpc_mujoco --time 6.0 --vx 0.18 --yaw 0.15
```

The standalone entry point can also use the migrated ROS2-free MRT joint
controller thread:

```bash
./build/wb_mpc_mujoco --threaded-mpc --headless --time 0.05
```

## Centroidal Run

First run the load/precompile path:

```bash
./build/centroidal_mpc_mujoco --precompile-only
```

Short simulation with the MuJoCo viewer:

```bash
./build/centroidal_mpc_mujoco --time 0.05
```

Headless smoke test:

```bash
./build/centroidal_mpc_mujoco --headless --time 1.0
```

Optional walking command starts after 2 seconds:

```bash
./build/centroidal_mpc_mujoco --time 8.0 --walk
./build/centroidal_mpc_mujoco --time 6.0 --vx 0.18 --yaw 0.15
```

Viewer controls: left-drag rotates, right-drag pans, scroll zooms, Space pauses,
Right steps once, `S` toggles real-time pacing, and Esc exits.

## Control Chain

Each MuJoCo tick reads the floating base and joints, converts them to the active
MPC model, advances SQP through MRT at the configured MPC frequency, evaluates
the policy at `time + 0.005 s`, maps active-joint policy outputs back to the
29-DoF G1 joint order, applies PD plus feedforward torque, clamps to MJCF
actuator limits, and steps MuJoCo once.

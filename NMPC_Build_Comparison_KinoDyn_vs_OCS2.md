# Kino-Dyn NMPC 与 OCS2 Centroidal MPC 构建方式对比

本文对比当前仓库的 `algorithm/g1_kinodynamics_nmpc.cpp` 与
`/home/songchao/OPTControl_env/wb_humanoid_mpc_ws/src/humanoid_centroidal_mpc`
里 OCS2 NMPC 的构建方式，重点看 cost、解析导数、Hessian 和约束。

## 1. 总体结论

OCS2 快很多的核心原因不只是求解器实现不同，而是两边在线计算路径不一样。  
如果只对比 `humanoid_centroidal_mpc`，低维 centroidal OCP 是很大的因素；但你补充的
`humanoid_wb_mpc` 也比 Aligator kino-dyn 快 5 倍，这说明还有更关键的工程因素：
OCS2 whole-body 版本虽然也是接近 full-body acceleration model，但它把 dynamics、足端动力学 cost/constraint
都放进 CppAD codegen，在线阶段直接调用生成后的 value/Jacobian；而当前 Aligator kino-dyn 在线每个 node
都在跑 Pinocchio centroidal derivative 和 constrained ProxDDP/AL。

1. OpenLoong 这边是 kinodynamics/full-body reduced model，状态是完整 `q, v`，控制是双脚 6D wrench 加关节加速度。
2. OCS2 那边是 centroidal model，状态是归一化质心动量、base pose、关节位置，控制是双脚 6D wrench 加关节速度。
3. OpenLoong 每个 shooting node 都要做更重的 Pinocchio centroidal derivative，包括 `ccrba/dccrba`、CoM/Jcom、frame Jacobian、`computeCentroidalDynamicsDerivatives` 等。
4. OCS2 大量动态和任务空间导数通过 CppAD codegen 预编译成库，在线阶段调用的是生成后的 value/Jacobian 接口。
5. OpenLoong/Aligator 当前把接触 cone、wrench box、站立足零速度作为硬约束给 ProxDDP/AL 处理；OCS2 里摩擦锥、接触力矩多是 soft constraint，零 wrench/零速度是 equality constraint，约束处理路径不同。
6. OpenLoong 当前 `SolverProxDDP` 使用 `LQSolverChoice::SERIAL`，默认线程数也是 1；OCS2 工程通常已经按它自己的 MPC/SQP 框架做了 problem 复用、预计算和 codegen 缓存。

所以“OCS2 比 ProxDDP 快 4/5 倍”不能直接理解为 OCS2 求解器本身强 4/5 倍。更准确地说：
OCS2 的 centroidal 版本问题更轻；OCS2 的 whole-body 版本虽然维度接近，但导数生成、约束软化、
problem 复用、SQP/MPC 工程化和多线程路径都更成熟。

## 2. 状态和输入

### OpenLoong `G1KinodynamicsNmpc`

代码位置：

- `algorithm/g1_kinodynamics_nmpc.h`
- `algorithm/g1_kinodynamics_nmpc.cpp`

状态：

```text
x = [q; v]
```

其中 `q` 是 Pinocchio generalized coordinates，`v` 是 generalized velocity。Aligator 使用 `MultibodyPhaseSpace`，切空间维度是 `2 * nv`。

控制：

```text
u = [W_l(6); W_r(6); qdd_j(nv - 6)]
W = [fx, fy, fz, mx, my, mz]
```

所以控制维度：

```text
nu = 12 + nv - 6
```

### OCS2 `humanoid_centroidal_mpc`

代码位置：

- `humanoid_centroidal_mpc/include/humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h`
- `humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp`

状态：

```text
x = [h; q_b; q_j]
h   = [v_com_x, v_com_y, v_com_z, L_x / mass, L_y / mass, L_z / mass]
q_b = [base_x, base_y, base_z, yaw, pitch, roll]
q_j = active joint positions
```

控制：

```text
u = [W_l(6); W_r(6); qdot_j]
```

OCS2 的状态里没有完整 `v`，关节速度直接是输入。因此同样机器人下，OCS2 的 shooting state 通常明显小于 `q, v` 全状态模型。

### OCS2 `humanoid_wb_mpc`

代码位置：

- `humanoid_wb_mpc/include/humanoid_wb_mpc/common/WBAccelMpcRobotModel.h`
- `humanoid_wb_mpc/src/WBMpcInterface.cpp`
- `humanoid_wb_mpc/src/dynamics/WBAccelDynamicsAD.cpp`

状态：

```text
x = [q_b; q_j; qd_b; qd_j]
q_b  = [base_x, base_y, base_z, yaw, pitch, roll]
qd_b = [base_linear_velocity, euler_zyx_derivatives]
```

输入：

```text
u = [W_l(6); W_r(6); qdd_j]
```

这个维度和 OpenLoong kino-dyn 很接近，甚至语义上更像“full-body acceleration MPC”。所以如果
`humanoid_wb_mpc` 仍然快 5 倍，主要差距就不是状态维度，而是导数/约束/求解器实现路径。

## 3. 动力学构建

### OpenLoong kino-dyn 动力学

OpenLoong 使用：

```cpp
aligator::dynamics::KinodynamicsFwdDynamicsTpl<double>
aligator::dynamics::IntegratorSemiImplEulerTpl<double>
```

连续动力学核心在 `third_party/aligator/include/aligator/modelling/dynamics/kinodynamics-fwd.hxx`。

它做的事情可以概括为：

```text
Ag = centroidal momentum matrix
dAg = time derivative of Ag

u = [contact wrench; joint acceleration]

base_acc =
  Ag_base^{-1} * (
    contact_wrench + mass * gravity
    - dAg * v
    - Ag_joint * joint_acc
  )

xdot = [v; base_acc; joint_acc]
```

这里的关键点是：base acceleration 不是独立控制量，而是通过 centroidal momentum balance 解出来。这比 SRBD/centroidal 模型重，因为它依赖当前完整构型、速度、CoM、接触点位置和 centroidal matrix。

### OCS2 centroidal 动力学

OCS2 使用：

```cpp
CentroidalDynamicsAD
PinocchioCentroidalDynamicsAd
```

对应文件：

- `humanoid_centroidal_mpc/src/dynamics/CentroidalDynamicsAD.cpp`

`computeFlowMap()` 调用 CppAD codegen 的 value，`linearApproximation()` 调用 CppAD codegen 的 linear approximation。

它求的是 centroidal state 的 dynamics，不再显式优化完整 `v`：

```text
hdot       <- contact wrench + gravity
base_dot   <- centroidal mapping(h, q, qdot_j)
joint_dot  <- qdot_j
```

在线求解时主要拿到：

```text
f(x, u), df/dx, df/du
```

没有看到当前工程在 NMPC 主路径里使用 exact dynamics Hessian。

### OCS2 whole-body acceleration 动力学

`humanoid_wb_mpc` 使用：

```cpp
WBAccelDynamicsAD
SystemDynamicsBaseAD
```

对应文件：

- `humanoid_wb_mpc/src/dynamics/WBAccelDynamicsAD.cpp`
- `humanoid_wb_mpc/src/dynamics/DynamicsHelperFunctions.cpp`

其状态导数是：

```text
xdot = [qd; qdd_base; qdd_j]
```

其中 `qdd_j` 来自输入，`qdd_base` 由浮动基动力学解出。代码路径是：

```text
crba(q) -> M(q)
nonLinearEffects(q, qd) -> nle(q, qd)
computeFrameJacobian(q) -> J_foot
baseExternalForces = J_foot_base^T * W_foot
qdd_base = solve floating-base block from M, nle, qdd_j, baseExternalForces
```

关键差异：这套 dynamics 被包进 `SystemDynamicsBaseAD`，由 CppAD codegen 生成 value/Jacobian。
在线求解时 OCS2 调用生成后的动态库，而不是像当前 Aligator `KinodynamicsFwdDynamicsTpl::dForward()`
那样每个 shooting node 现场跑多次 Pinocchio centroidal derivative。

## 4. Cost 对比

### OpenLoong cost

OpenLoong 在每个 stage 构建 `CostStack`，主要包括：

| Cost | 残差 | Hessian 来源 |
| --- | --- | --- |
| `state_cost` | `x - x_ref`，完整 `q/v` 状态跟踪 | 二次 cost，线性残差，Hessian 精确 |
| `control_cost` | `u - u_ref`，wrench 和关节加速度正则 | 二次 cost，线性残差，Hessian 精确 |
| `centroidal_cost` | centroidal momentum 追踪 0 | Aligator residual Jacobian + `J^T W J` |
| `centroidal_derivative_cost` | centroidal momentum derivative 正则 | Aligator residual Jacobian + `J^T W J` |
| `left/right_foot_pose_cost` | 足端 frame placement 跟踪 | Pinocchio frame residual Jacobian + `J^T W J` |
| terminal `state_cost` | terminal state | 精确二次 |
| terminal `centroidal_cost` | terminal centroidal momentum | `J^T W J` |

当前权重在 `g1_kinodynamics_nmpc.cpp` 里硬编码：

```text
base position:      [50, 50, 150]
base orientation:   [250, 250, 120]
joint position:     legs 1, waist 20, upper body 2
base linear vel:    [4, 4, 8]
base angular vel:   [20, 20, 8]
joint velocity:     legs 0.2, others 1

wrench force:       [2e-4, 2e-4, 2e-5]
wrench moment:      [2e-2, 2e-2, 2e-2]
joint acc:          legs 0.2, others 0.5

centroidal momentum:
  [0.5, 0.5, 1.0, 5.0, 5.0, 2.0]

centroidal derivative:
  [0.05, 0.05, 0.1, 0.5, 0.5, 0.2]

foot pose:
  position [4000, 4000, 7000], rotation [800, 800, 400]
```

Aligator 的 `QuadraticResidualCostTpl` 默认：

```cpp
bool gauss_newton = true;
```

所以当前这些非线性 residual cost 的 Hessian 是 Gauss-Newton：

```text
H ~= J^T W J
```

不会使用 residual 的二阶项，除非显式把 `gauss_newton=false`，并且 residual 实现了 `computeVectorHessianProducts()`。

### OCS2 cost

OCS2 在 `CentroidalMpcInterface::setupOptimalControlProblem()` 里挂 cost：

| Cost | 文件/类 | 导数/Hessian |
| --- | --- | --- |
| `stateInputQuadraticCost` | `StateInputQuadraticCost` | exact Q/R Hessian |
| `terminalCost` | `QuadraticStateCost` | exact Qf Hessian |
| task-space kinematics costs | `EndEffectorKinematicsQuadraticCost` | CppAD kinematics Jacobian，Gauss-Newton Hessian |
| `icp_Cost` | `ICPCost` | `StateInputCostGaussNewtonAd`，AD residual Jacobian，GN Hessian |
| foot task-space cost | `CentroidalMpcEndEffectorFootCost` | `StateInputCostGaussNewtonAd`，AD residual Jacobian，GN Hessian |
| external torque cost | `ExternalTorqueQuadraticCostAD` | `StateInputCostGaussNewtonAd`，AD residual Jacobian，GN Hessian |

其中 foot cost 的 residual 包括：

```text
foot position error
foot orientation-to-plane error
foot linear velocity error
foot angular velocity error
```

external torque cost 计算：

```text
tau_ext = J_ee(q)^T * contact_wrench
```

然后对指定 active joints 的外力矩做二次正则。

OCS2 这边很多 cost 是从 task file 读权重，问题构建更参数化；OpenLoong 当前权重主要写死在 C++ 里。

### OCS2 whole-body cost

`humanoid_wb_mpc` 的 cost 构成和 centroidal 版本相似，但足端项改成 dynamics/acceleration 版本：

| Cost | 文件/类 | 导数/Hessian |
| --- | --- | --- |
| `stateInputQuadraticCost` | common `StateInputQuadraticCost` | exact Q/R Hessian |
| `terminalCost` | common `QuadraticStateCost` | exact Qf Hessian |
| foot tracking cost | `EndEffectorDynamicsFootCost` | `StateInputCostGaussNewtonAd`，CppAD residual Jacobian，GN Hessian |
| optional task dynamics cost | `EndEffectorDynamicsQuadraticCost` | `StateInputCostGaussNewtonAd`，CppAD residual Jacobian，GN Hessian |
| optional joint torque cost | `JointTorqueCostCppAd` | `StateInputCostGaussNewtonAd` |

`EndEffectorDynamicsFootCost` 里会计算：

```text
q, qd, qdd
foot orientation error
foot linear/angular velocity
foot linear/angular acceleration
```

它比 OpenLoong 的 foot placement residual 更“动态”，但导数仍是 CppAD codegen + Gauss-Newton，不是在线手写
Pinocchio 二阶链式求导。

## 5. 约束对比

### OpenLoong 约束

OpenLoong 每个 stage 对每只脚加：

1. wrench box hard constraint

```text
Fx, Fy in [-forceLimit, forceLimit]
Fz     in [0, forceLimit]
Mx, My, Mz in [-momentLimit, momentLimit]
```

inactive foot 时 wrench box 退化成全零。

2. 17 维 wrench cone hard constraint

使用：

```cpp
aligator::CentroidalWrenchConeResidualTpl<double>
aligator::NegativeOrthantTpl<double>
```

它包含：

```text
unilateral contact
Coulomb pyramid
CoP limits
z torque limits
```

这是一个线性 pyramid/wrench cone。Jacobian 是常数矩阵，Hessian 为 0。

3. standing foot zero velocity equality

使用：

```cpp
aligator::FrameVelocityResidualTpl<double>
aligator::EqualityConstraintTpl<double>
```

当 `constrainStandingFeet=true` 时启用。

4. joint acceleration box

使用自定义 `ControlSliceResidual` 提取控制向量里的关节加速度，再接 `BoxConstraint`。

### OCS2 约束

OCS2 在每只脚上加：

| 约束 | 激活条件 | 形式 | 导数 |
| --- | --- | --- | --- |
| friction force cone | stance | soft constraint | 手写 value/J/H |
| contact moment XY | stance | soft constraint | CppAD value/J |
| zero wrench | swing | equality | 手写线性 value/J |
| zero velocity | stance | equality | end-effector kinematics CppAD/J |
| normal velocity | swing | equality | end-effector kinematics CppAD/J |
| knee mimic | optional | equality | 手写线性 value/J |
| joint limits | always | state soft constraint | barrier/soft |
| foot collision | always | state soft constraint | CppAD/soft |

OCS2 的 friction cone 在 `FrictionForceConeConstraint.cpp` 中是一个平滑标量约束：

```text
mu * (Fz + gripperForce) - sqrt(Fx^2 + Fy^2 + regularization)
```

它显式提供：

```text
df/du
d2f/du2
d2f/dx2 = diagonal shift
```

OCS2 的 `ContactMomentXYConstraintCppAd` 则把 wrench 旋到 foot local frame，再约束 CoP 矩形：

```text
Mx - y_min * Fz
-Mx + y_max * Fz
-My - x_min * Fz
 My + x_max * Fz
```

注意：这与 OpenLoong 的 17 维 `CentroidalWrenchConeResidual` 不完全等价。OpenLoong 的 residual 直接作用在控制向量的 wrench 分量上，包含 yaw torque 相关行；OCS2 检查到的主路径里 force cone 和 contact moment XY 是分开的，且没有看到同样的 17 维 yaw torque hard cone。

### OCS2 whole-body 约束

`humanoid_wb_mpc` 基本沿用 common factory 的接触约束：

```text
frictionForceCone: soft constraint
contactMomentXY: soft constraint
zeroWrench: swing equality
jointLimits / footCollision: soft constraint
```

但 stance foot 约束从 centroidal 版本的 zero velocity 换成：

```cpp
ZeroAccelerationConstraintCppAd
EndEffectorDynamicsAccelerationsConstraint
```

约束 residual 包含足端 pose、twist、acceleration 的线性组合：

```text
b + Ax * foot_pose + Av * foot_twist + Aa * foot_acceleration
```

线性化由 `PinocchioEndEffectorDynamicsCppAd` 的 codegen 接口给出。也就是说 WB 版本的接触运动学/动力学约束虽然更丰富，
但在线导数仍然主要是“调用预编译 Jacobian”。

## 6. 求解时分别计算哪些导数

### OpenLoong / Aligator 当前求解

每个 shooting node 主要计算：

1. dynamics value

```text
x_{k+1} = integrate(x_k, u_k)
```

2. dynamics Jacobian

```text
Fx = df/dx
Fu = df/du
```

由 `KinodynamicsFwdDynamicsTpl::dForward()` 给出，内部是 Pinocchio 解析算法加手写矩阵链式法则。

3. cost gradient

```text
lx, lu
```

对 residual cost：

```text
grad = J^T W r
```

4. cost Hessian

当前默认 Gauss-Newton：

```text
lxx, lxu, luu ~= J^T W J
```

线性 residual 的二次 cost 是精确 Hessian；非线性 residual 当前不包含二阶 residual 项。

5. constraint value/Jacobian

```text
c(x, u), dc/dx, dc/du
```

ProxDDP 当前构建 LQ 子问题时使用约束 Jacobian。Aligator `StageModel::computeSecondOrderDerivatives()` 只调用 cost Hessian，不调用 constraint Hessian。

6. dynamics Hessian

`SolverProxDDP` 默认 `HessianApprox::GAUSS_NEWTON`。只有设置为 `EXACT` 时才会把 dynamics Hessian 加入 LQ knot：

```cpp
if (hess_approx_ == HessianApprox::EXACT) {
  knot.Q += dd.Hxx_;
  knot.S += dd.Hxu_;
  knot.R += dd.Huu_;
}
```

当前 `G1KinodynamicsNmpc` 构造 solver 时没有改这个选项，所以可以认为在线主要用 dynamics 一阶线性化，不用 exact dynamics Hessian。

### OCS2 当前求解

OCS2 主路径计算：

1. dynamics value and linear approximation

```text
f(x, u), df/dx, df/du
```

由 `PinocchioCentroidalDynamicsAd` 的 CppAD codegen 接口给出。

2. quadratic state/input cost 的 exact gradient/Hessian

```text
Q, R, Q_final
```

3. Gauss-Newton AD cost 的 residual/Jacobian/Hessian

```text
r(x, u)
J = dr/d[x,u]
H ~= J^T J
```

包括 foot cost、ICP cost、external torque cost 和一些 task-space kinematics cost。

4. constraints 的 value/Jacobian 或 quadratic approximation

`FrictionForceConeConstraint` 声明为 `ConstraintOrder::Quadratic`，提供二阶近似。

`ZeroWrenchConstraint`、`ZeroVelocityConstraintCppAd`、`NormalVelocityConstraintCppAd`、`ContactMomentXYConstraintCppAd` 多数按 `ConstraintOrder::Linear` 给线性近似。

5. soft constraint penalty 组合

摩擦锥、接触力矩、joint limits、foot collision 等 soft constraint 通过 penalty 变成 cost/soft cost 的贡献。这样很多不等式不会像 OpenLoong 当前那样全部进入硬约束 AL 子问题。

### OCS2 whole-body 当前求解

`humanoid_wb_mpc` 每个 node 主要计算：

```text
WBAccelDynamicsAD:
  f(x, u), df/dx, df/du    <- CppAD codegen

StateInputQuadraticCost:
  exact Q/R gradient/Hessian

EndEffectorDynamicsFootCost:
  residual r, Jacobian J, Hessian ~= J^T J    <- CppAD codegen + Gauss-Newton

ZeroAccelerationConstraint:
  c(x, u), dc/dx, dc/du    <- CppAD codegen end-effector dynamics

Friction/contact constraints:
  soft penalty or linear/quadratic approximation
```

这和 OpenLoong 当前最大不同是：OCS2 WB 版本没有在线调用一个手写 `dForward()` 去拼完整 dynamics Jacobian；
它在初始化阶段把 AD 函数编译好，运行时直接查 value/Jacobian。

## 7. Aligator 这边已经能给出的解析导数

当前 OpenLoong/Aligator 里已经能直接得到的解析一阶导数：

| 模块 | 当前导数能力 | 说明 |
| --- | --- | --- |
| `KinodynamicsFwdDynamicsTpl` | 解析 `df/dx`, `df/du` | Pinocchio centroidal derivatives + 手写链式法则 |
| `CentroidalWrenchConeResidualTpl` | 解析 value/J | 17 维线性 cone，Jacobian 常数 |
| `ControlSliceResidual` | 解析 value/J | 提取 wrench 或 joint acc slice，Jacobian 是 identity block |
| `FramePlacementResidualTpl` | 解析 value/J | Pinocchio frame placement residual |
| `FrameVelocityResidualTpl` | 解析 value/J | Pinocchio frame velocity residual |
| `CentroidalMomentumResidualTpl` | 解析 value/J | Pinocchio centroidal momentum |
| `CentroidalMomentumDerivativeResidualTpl` | 解析 value/J | Pinocchio centroidal momentum derivative |
| `QuadraticStateCostTpl` | 精确 gradient/Hessian | 线性 state residual |
| `QuadraticControlCostTpl` | 精确 gradient/Hessian | 线性 control residual |

当前不默认使用的部分：

1. exact residual Hessian  
   `QuadraticResidualCostTpl` 默认 `gauss_newton=true`，所以非线性 residual cost 的 Hessian 是 `J^T W J`。如果想用 exact Hessian，需要设置 `gauss_newton=false`，并确保对应 residual 实现了 `computeVectorHessianProducts()`。

2. exact dynamics Hessian  
   `SolverProxDDP` 默认 `HessianApprox::GAUSS_NEWTON`。即使 dynamics data 里有 Hessian buffer，当前求解也不会使用它。要走 exact DDP，需要 solver 设置 exact Hessian，并且 dynamics 模型提供正确二阶导数。

3. constraint Hessian  
   当前 Aligator stage 的二阶导数路径主要是 cost Hessian。硬约束进入 LQ 子问题时主要使用 value/Jacobian。线性 cone 和 box 本身 Hessian 也为 0。

## 8. 为什么 OpenLoong 当前会慢

### 8.1 模型维度更大

OpenLoong 的切空间维度是：

```text
ndx = 2 * nv
```

OCS2 是：

```text
nx = 12 + active_joint_dim
```

如果 active joint 数相近，OpenLoong 多了一整套 generalized velocity 状态；LQ 子问题和所有 residual/Jacobian/Hessian 都随维度变大。

这个解释只适用于对比 `humanoid_centroidal_mpc`。对比 `humanoid_wb_mpc` 时，维度已经很接近，因此 5 倍差距主要不来自维度。

### 8.2 动力学导数更重

OpenLoong 的 `dForward()` 每个 node 会多次调用 Pinocchio centroidal derivative。尤其这几类操作比较贵：

```text
centerOfMass / jacobianCenterOfMass
computeJointJacobians / getFrameJacobian
computeCentroidalDynamicsDerivatives
ccrba / dccrba
```

OCS2 使用的是 reduced centroidal dynamics + CppAD codegen，在线计算更像调用预编译函数。

### 8.3 hard constraints + ProxDDP/AL 成本

OpenLoong 每个 stage 对双脚都可能有：

```text
2 * wrench box
2 * 17D wrench cone
2 * foot velocity equality
1 * joint acceleration box
```

这些硬约束会进入 ProxDDP 的 constrained LQ/AL 流程。即使 cone 本身是线性的，AL 外层、constraint projection 和 LQ 子问题都会增加时间。

OCS2 把 friction cone、contact moment 等作为 soft constraint，很多时候等价于加 penalty cost，避免了大量硬不等式直接进 constrained LQ。

### 8.4 当前 Aligator 配置偏保守

当前 OpenLoong 代码里：

```cpp
cache->solver->linear_solver_choice = aligator::LQSolverChoice::SERIAL;
cache->solver->max_al_iters = maxAlIterations;
```

并且没有调用 `setNumThreads()`。如果 horizon 较长，derivatives 和 LQ 都是可能被串行路径限制的。

### 8.5 problem 重建问题已处理，但不是全部

当前 `G1KinodynamicsNmpc` 已经加了 `SolverCache`，相同 signature 下只更新：

```text
initial state
state/control target
foot pose reference
```

不会每次重建 stages/problem/solver setup。这个能去掉一部分 overhead，但如果主要时间在 `derivatives_ms` 和 `ddp_ms`，仍然会慢于 OCS2。

### 8.6 为什么 OCS2 whole-body 也更快

`humanoid_wb_mpc` 快 5 倍时，主要看这几个原因：

1. dynamics Jacobian 路径不同  
   OpenLoong Aligator 当前在线走 `KinodynamicsFwdDynamicsTpl::dForward()`，每个 node 都现场调用 Pinocchio 的
   CoM、frame Jacobian、centroidal dynamics derivatives。  
   OCS2 WB 走 `WBAccelDynamicsAD -> SystemDynamicsBaseAD`，初始化时 CppAD codegen，在线调用生成后的 value/Jacobian。

2. 动力学公式不同  
   Aligator kino-dyn 使用 centroidal momentum balance：

```text
qdd_base = Ag_base^{-1} * (contact_wrench + gravity - dAg*v - Ag_j*qdd_j)
```

   OCS2 WB 使用 mass matrix / floating-base block：

```text
M(q), nle(q, qd), J_foot(q)^T W
qdd_base = floating-base dynamics solve
```

   两者都合理，但导数复杂度和 codegen 后的在线代价不一样。Aligator 这边还要显式处理 `dAg`、`computeCentroidalDynamicsDerivatives`。

3. hard constraint 数量不同  
   OpenLoong 把 17 维 wrench cone、wrench box、joint acc box、foot velocity equality 都作为 hard constraints。  
   OCS2 WB 把 friction cone/contact moment/joint limit/foot collision 多数作为 soft constraints，只有 zero wrench、zero acceleration 等作为 equality。

4. LQ/约束处理不同  
   当前 OpenLoong 设置：

```cpp
linear_solver_choice = SERIAL
max_al_iters = maxAlIterations
```

   ProxDDP 会为 hard constraints 做 AL/projection/constrained LQ。OCS2 的 SQP/MPC 实现对 linear/quadratic approximation、
   soft penalties、warm start 和多线程做得更完整。

5. foot dynamics residual 的 Hessian 是 GN  
   OCS2 WB 的足端 dynamics cost 虽然包括 acceleration，但 Hessian 仍是 `J^T J`，不追 exact residual Hessian。
   OpenLoong 如果尝试 exact dynamics/cost 二阶项，可能更慢；当前虽然也是 GN，但 dynamics Jacobian 本身已经重。

因此，和 `humanoid_wb_mpc` 比时，最优先要验证的是：

```text
OpenLoong derivatives_ms 是否远大于 OCS2 dynamics/cost/constraint approximation 时间
OpenLoong ddp_ms 是否被 hard constraints / SERIAL LQ 拉大
OpenLoong cache 是否稳定显示 reuse
```

## 9. 后续优化建议

如果目标是让 Aligator kino-dyn 更接近 OCS2 速度，建议按优先级做：

1. 先把 timing 分开看  
   当前日志已经有：

```text
setup_ms
run_ms
derivatives_ms
ddp_ms
run_other_ms
cache=reuse/rebuild
```

如果 `cache=reuse` 后 `derivatives_ms` 仍然大，主要瓶颈是 dynamics/residual 导数。  
如果 `ddp_ms` 大，主要瓶颈是 LQ/AL/约束规模。

2. 做一个 Aligator SRBD/centroidal 版本对齐 OCS2  
   不要直接拿 full `q,v` kinodynamics 和 OCS2 centroidal 比速度。先在 Aligator 中构建同样的 centroidal state/input：

```text
x = [h, q_b, q_j]
u = [W_l, W_r, qdot_j]
```

这样才能比较 solver 本身。

3. 减少 hard inequality 数量  
   可把 friction/cone/contact moment 改成 soft cost 或 relaxed barrier，先保留关键 equality。这样可以降低 ProxDDP constrained LQ 的负担。

4. 给 ProxDDP 开并行导数  
   Aligator solver 有 `setNumThreads()`。如果当前 CPU 允许，可对 stage derivative 并行。

5. 检查 wrench frame 约定  
   OpenLoong dynamics 把 wrench 当世界系力/力矩使用，`CentroidalWrenchConeResidual` 直接在 `u` 分量上做 cone；OCS2 的 contact moment XY 会把 wrench 旋到 foot local frame。足底有明显 roll/pitch 时，两边约束语义可能不一致。

6. 如果要 exact Hessian，先评估收益  
   当前 cost Hessian 是 Gauss-Newton，通常对 tracking/residual cost 已经够用。exact dynamics Hessian 会显著增加导数成本，不一定让实时 NMPC 更快。更现实的优化通常是降低模型维度、减少硬约束、并行化和 codegen。

## 10. 下一步：构造一致问题

更合理的下一步不是继续比较 `g1_kinodynamics_nmpc.cpp` 和 OCS2，而是把
`humanoid_centroidal_mpc` 的同一个 OCP 用 Aligator 重新搭出来：

```text
OCS2 humanoid_centroidal_mpc
  vs
Aligator humanoid_centroidal_mpc 等价复刻版
```

这样比较对象一致，才能精确回答：

```text
哪些导数 Aligator 已经有解析形式
哪些导数需要自己写
哪些只能先用 CppAD / finite difference / Gauss-Newton
哪些时间差来自 solver，而不是模型或约束不一致
```

### 10.1 不要直接用 Aligator 自带 `CentroidalFwdDynamics`

Aligator 自带 centroidal dynamics 的状态通常是：

```text
x = [c, h, L]
```

或者 smooth-force 版本：

```text
x = [c, h, L, contact_forces]
u = force_derivatives
```

但 OCS2 `humanoid_centroidal_mpc` 的状态是：

```text
x = [h_normalized, base_pose, q_j]
u = [contact_wrenches, qdot_j]
```

并且它要通过 `CentroidalModelPinocchioMapping` 从：

```text
h_normalized + qdot_j + q
```

反解/恢复 Pinocchio generalized velocity，进而得到：

```text
base_pose_dot
joint_position_dot = qdot_j
normalized_momentum_dot
```

所以直接套 Aligator 的 `CentroidalFwdDynamicsTpl` 只能得到一个“centroidal toy problem”，不能算和
OCS2 `humanoid_centroidal_mpc` 一致。

### 10.2 Aligator 等价复刻版需要的新模块

建议新建一个 Aligator dynamics：

```cpp
Ocs2StyleCentroidalDynamics : aligator::ExplicitDynamicsModelTpl<double>
```

状态/输入维度与 OCS2 完全一致：

```text
nx = 12 + n_j
nu = 12 + n_j
x  = [h_norm(6), base_pose(6), q_j]
u  = [W_l(6), W_r(6), qdot_j]
```

forward 与 OCS2 一致：

```text
q = [base_pose, q_j]
updateCentroidalDynamics(q)
v = CentroidalModelPinocchioMapping::getPinocchioJointVelocity(x, u)

xdot_h = getNormalizedCentroidalMomentumRate(q, u)
xdot_q = v
```

这里 `xdot_q` 里的 base 部分是 OCS2 使用的 base linear velocity 和 ZYX Euler angle derivatives。

### 10.3 哪些可以直接解析，哪些需要补

| 模块 | Aligator 当前是否直接有 | 一致复刻时建议 |
| --- | --- | --- |
| OCS2-style centroidal dynamics `f(x,u)` | 没有现成等价版 | 新写 dynamics，先可复用 OCS2 `PinocchioCentroidalDynamicsAD` 的 codegen Jacobian，之后再手写解析 |
| `df/dx, df/du` | Aligator 有 `[c,h,L]` 版本解析，但不匹配 OCS2 state | 需要实现 OCS2 mapping 的解析 Jacobian，或先用 CppAD/finite diff |
| dynamics Hessian | OCS2 主路径通常不用 exact dynamics Hessian | Aligator 也先不要用 exact dynamics Hessian，保持 GN/一阶动力学一致 |
| state/input quadratic cost | 有 | 直接用 `QuadraticStateCost` / `QuadraticControlCost` 或自定义 target updater |
| terminal quadratic cost | 有 | 直接用 |
| foot task-space kinematics cost | 没有 OCS2 state/input 版本 | 需要新 residual：用 OCS2 centroidal mapping 从 `x,u` 得到 `q,v`，再算 frame pose/velocity |
| foot cost Jacobian | 没有现成等价版 | OCS2 用 CppAD；Aligator 要么包 CppAD/Jacobian，要么手写 Pinocchio + mapping Jacobian |
| ICP cost | 没有现成等价版，但很简单 | 手写 residual/Jacobian，或先只写 value+finite diff |
| external torque cost `J_ee(q)^T W` | 没有现成等价版 | value 容易；Jacobian 对 `q` 需要 frame Jacobian 的导数，建议先 CppAD/GN |
| friction force cone | 有相近形式 | OCS2 是 smooth scalar cone + soft penalty；Aligator 现有 cone 多为 hard residual，需要约束语义也对齐 |
| contact moment XY | 没有完全等价版 | OCS2 会把 wrench 转到 foot local frame；Aligator 需新 residual，Jacobian wrt `q` 不简单 |
| zero wrench | 有，线性 slice 即可 | 直接写 `ControlSliceResidual` |
| stance foot zero velocity | 没有 OCS2 state/input 版本 | 需要 mapped frame velocity residual |
| swing normal velocity | 没有 OCS2 state/input 版本 | 需要 mapped frame velocity residual |
| joint mimic | 可手写线性 | 直接手写 |
| joint limits soft constraint | 可手写 | 先加 barrier/box soft cost，或先跳过做核心对比 |
| foot collision soft constraint | 没有现成等价版 | 后置，先不放进第一版一致问题 |

粗略数一下，要完全复刻 OCS2 `humanoid_centroidal_mpc`，Aligator 当前没有现成解析等价实现的核心模块至少有 7 类：

```text
1. OCS2-style centroidal dynamics + mapping Jacobian
2. foot task-space cost residual/Jacobian
3. ICP cost residual/Jacobian
4. external torque cost residual/Jacobian
5. contact moment XY local-frame residual/Jacobian
6. zero velocity / normal velocity mapped constraints
7. foot collision soft constraint
```

但第一版 benchmark 不应该一次全加。建议分三层：

### 10.4 三层 benchmark

第一层：只比 dynamics + Q/R cost

```text
OCS2 CentroidalDynamicsAD + stateInputQuadraticCost
vs
Aligator Ocs2StyleCentroidalDynamics + QuadraticState/ControlCost
```

目标是测：

```text
df/dx, df/du 生成方式
rollout/integration
LQ/DDP/SQP solver overhead
```

这一层最能分辨“求解器本体”和“dynamics Jacobian”。

第二层：加入接触力约束

```text
friction cone
contact moment XY
zero wrench
```

目标是测：

```text
soft constraint vs hard constraint
barrier/penalty vs AL/projection
constraint Jacobian 开销
```

第三层：加入足端和全任务 cost/constraint

```text
foot task-space cost
zero velocity / normal velocity
ICP
external torque
joint limit
foot collision
```

目标是测 OCS2 完整问题和 Aligator 完整问题的差距。

### 10.5 最关键的判断标准

只要第一层的 dynamics + Q/R cost 里 Aligator 仍明显慢，就说明主要问题在：

```text
Aligator solver / integration / LQ / threading / problem data layout
```

如果第一层接近，而第二层开始慢，说明主要问题在：

```text
hard constraints vs soft constraints
ProxDDP AL/projection
constraint Jacobian 数量和形式
```

如果前两层接近，第三层变慢，说明主要问题在：

```text
foot kinematics/dynamics residual 的 Jacobian
CppAD codegen vs 手写/finite diff
额外 soft constraints
```

### 10.6 当前路线 A 第一层实现

已在根目录新增：

```text
humanoid_cd_nmpc_aligator/
```

当前可执行文件：

```bash
cd /home/songchao/OPTControl_env/OpenLoong-Dyn-Control/humanoid_cd_nmpc_aligator
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/proxddp_centroidal_layer1
```

这一版没有重写 OCS2 dynamics 导数，而是直接用同一个 codegen dynamics 对象：

```text
ocs2::PinocchioCentroidalDynamicsAD::getValue()
ocs2::PinocchioCentroidalDynamicsAD::getLinearApproximation()
```

Aligator 侧只做：

```text
Euler shooting
QuadraticStateCost / QuadraticControlCost，其中 running Q/R 乘 dt
SolverProxDDP backward pass / line search
```

同一程序也构建了 OCS2 `SqpSolver` 的 layer1 问题：

```text
同一个 f, df/dx, df/du
同一个 x_ref / u_ref
同一个 Q/R/Q_final
同一个 Euler 初始 rollout
OCS2 SQP integratorType = EULER
```

默认 G1 centroidal 配置下维度为：

```text
nx = 35
nu = 35
contacts6 = 2
```

cached codegen 后一次参考输出约为：

```text
horizon N=20, dt=0.02
ProxDDP solve ~= 9.2 ms
ProxDDP derivatives ~= 6.5 ms
ProxDDP ddp ~= 2.4 ms
OCS2 SQP total ~= 4.0 ms
OCS2 SQP lq ~= 0.35 ms
OCS2 SQP qp ~= 1.2 ms
```

注意首次运行如果 `/tmp/ocs2_aligator_centroidal` 没有 codegen 动力学库，会先生成 CppADCodeGen 动态库，可能额外花几十秒；之后 `codegen_load` 会降到毫秒以下。

### 10.7 当前严格对比边界

当前 layer1 只能说“数学 OCP 一致”，还不能说“除求解器外实现路径完全一致”。

已经一致的部分：

| 项 | 说明 |
| --- | --- |
| 状态和输入 | `nx=35, nu=35`，OCS2 centroidal state/input |
| robot model | 同一个 `ModelSettings + createCustomPinocchioInterface + CentroidalModelInfo` |
| 连续动力学 | 共用同一个 `PinocchioCentroidalDynamicsAD` codegen object |
| 连续动力学导数 | 共用 `getLinearApproximation()` |
| 离散化数学形式 | Euler |
| running cost | 同一个 Q/R；Aligator 侧乘 `dt`，对齐 OCS2 连续 running cost 积分 |
| terminal cost | 同一个 `Q_final * terminalCostScaling` |
| reference | `x_ref=x0`, `u_ref=mg/2` 双脚分配 |
| 初始轨迹 | 同一条 Euler rollout |
| 物理约束 | layer1 两边都没有加入 |

仍然不同的部分：

| 差异 | 这是不是求解器差异 | 影响 |
| --- | --- | --- |
| Aligator `StageModel + CostStack + QuadraticResidualCost` | 不完全是 | 多了 generic residual/Jacobian/Hessian 组装 |
| OCS2 `OptimalControlProblem + setupQuadraticSubproblem` | 不完全是 | 更贴近 SQP/HPIPM 数据结构 |
| Aligator `ODEAbstract + IntegratorEuler` 包装 | 不完全是 | 数学等价，但有 generic integration/transport 开销 |
| OCS2 `SensitivityIntegratorType::EULER` | 不完全是 | 数学等价，但直接进 OCS2 SQP 管线 |
| convergence / merit / line search | 是 | ProxDDP 和 SQP 的停止准则、merit/filter 不同 |
| timer 范围 | 不是数学问题差异 | `proxddp_derivatives` 不是裸 codegen，而是 Aligator 全 stage derivative 管线累计 |

因此现在 `raw_codegen_linearization ~= 0.24 ms`，而 `proxddp_derivatives ~= 5.9 ms`，说明主要不是 OCS2 codegen 导数本身慢，而是 Aligator frontend/装配路径和 ProxDDP 每次迭代重新线性化的组合开销。

如果要进一步做到“除求解器外实现路径也一致”，下一步不是继续包 `StageModel`，而是做一个更底层 adapter：

```text
OCS2 setupQuadraticSubproblem 产出离散 LQ/QP 数据
        |
        +--> OCS2 SQP / HPIPM
        |
        +--> Aligator/ProxDDP Riccati data
```

这样才能把 frontend、cost/residual 组装、离散化包装全部排除，只比较 SQP/HPIPM 和 ProxDDP/Riccati 的求解器路径。

## 11. 一句话判断

OCS2 centroidal 快，主要是模型更轻；OCS2 whole-body 也快，主要是 codegen 导数、soft constraint、成熟 SQP/MPC 复用和求解器工程化。OpenLoong 当前 kino-dyn NMPC 已经有不少解析一阶导数，但这些导数是在在线阶段跑 Pinocchio runtime derivative；再叠加 hard constraints 和 serial constrained LQ，就很容易比 OCS2 WB 慢 5 倍。

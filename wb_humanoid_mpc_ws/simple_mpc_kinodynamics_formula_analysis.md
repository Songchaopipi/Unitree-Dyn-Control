# simple-mpc kinodynamics NMPC 公式解析

本文基于 `/home/songchao/Hector_Simulation/simple-mpc` 和
`/home/songchao/Hector_Simulation/aligator` 源码整理，重点覆盖：

- kinodynamics NMPC 的状态/输入定义
- aligator `KinodynamicsFwdDynamics` 的真实动力学公式
- simple-mpc `KinodynamicsOCP` 里实际启用的 cost
- simple-mpc `KinodynamicsOCP` 里实际启用的约束
- MPC horizon / swing reference 如何更新
- MPC 输出如何被 example 中的 TSID inverse dynamics 使用

主要源码入口：

- `simple-mpc/include/simple-mpc/kinodynamics.hpp`
- `simple-mpc/src/kinodynamics.cpp`
- `simple-mpc/src/ocp-handler.cpp`
- `simple-mpc/src/mpc.cpp`
- `simple-mpc/src/foot-trajectory.cpp`
- `aligator/include/aligator/modelling/dynamics/kinodynamics-fwd.hpp`
- `aligator/include/aligator/modelling/dynamics/kinodynamics-fwd.hxx`
- `aligator/include/aligator/modelling/multibody/centroidal-momentum.hxx`
- `aligator/include/aligator/modelling/multibody/centroidal-momentum-derivative.hxx`
- `aligator/include/aligator/modelling/centroidal/centroidal-friction-cone.hxx`
- `aligator/include/aligator/modelling/centroidal/centroidal-wrench-cone.hxx`
- `aligator/include/aligator/modelling/multibody/frame-placement.hxx`
- `aligator/include/aligator/modelling/multibody/frame-translation.hxx`
- `aligator/include/aligator/modelling/multibody/frame-velocity.hxx`

说明：

- 这里写的是 simple-mpc 的 `KinodynamicsOCP`，不是 `CentroidalOCP`，也不是 `FullDynamicsOCP`。
- 这个 formulation 是“全身运动学 + 质心动力学”，不是纯 centroidal，也不是完整刚体逆动力学。
- simple-mpc 本身没有像当前 OCS2 工程那样的 task file；权重通常由 Python example 的 dict 传入。
- aligator 的 `NegativeOrthant` 表示 residual `h(x,u) <= 0`，所以 cone 约束的符号要按源码 residual 写。

## 1. 模型状态与输入

### 1.1 状态

`KinodynamicsOCP` 使用 aligator 的 `MultibodyPhaseSpace(model)`。

状态是完整 Pinocchio floating-base 相空间：

```math
x = [q, v]
```

其中：

- `q` 是 Pinocchio generalized configuration，维度 `nq`
- `v` 是 Pinocchio generalized velocity，维度 `nv`
- floating base 使用 Pinocchio 模型本身的自由基表达，通常 `q` 里 base 是 7 维，`v` 里 base 是 6 维

因此：

```math
nx = nq + nv
```

aligator 的 tangent dimension 是：

```math
ndx = 2 nv
```

simple-mpc 的 `OCPHandler` 构造里也设置：

```text
nq_ = model.nq
nv_ = model.nv
ndx_ = 2 * model.nv
```

`RobotModelHandler::reference_state_` 初始化为：

```math
x_{ref,0} = [q_{reference}, 0_{nv}]
```

### 1.2 输入

`KinodynamicsOCP` 的输入是：

```math
u = [\lambda_0, \lambda_1, \ldots, \lambda_{n_c-1}, a_j]
```

其中：

- `\lambda_i` 是第 `i` 个足端的 contact force 或 contact wrench
- `a_j` 是 actuated joint acceleration，维度 `nv - 6`

源码里：

```text
nu_ = nv_ - 6 + force_size * feet_nb
```

即：

```math
nu = n_c \cdot n_f + (nv - 6)
```

其中：

- `n_c = feet_nb`
- `n_f = force_size`
- `force_size = 3` 表示 point foot，只优化 `[F_x,F_y,F_z]`
- `force_size = 6` 表示 6D foot，优化 `[F_x,F_y,F_z,M_x,M_y,M_z]`

输入排列顺序由 `RobotModelHandler::getFeetFrameNames()` 决定。

第 `i` 个接触输入在：

```math
u[i n_f : (i+1)n_f]
```

关节加速度在：

```math
u[n_c n_f : n_c n_f + nv - 6]
```

### 1.3 Talos example 里的维度

`examples/talos_kinodynamics.py` 里：

- 双足
- 每只脚是 quad foot
- `force_size = 6`

因此：

```math
u =
[W_L, W_R, a_j]
```

```math
nu = 12 + nv - 6
```

其中：

```math
W_i = [F_x,F_y,F_z,M_x,M_y,M_z]
```

### 1.4 Go2 example 里的维度

`examples/go2_kinodynamics.py` 里：

- 四足
- 每只脚是 point foot
- `force_size = 3`

因此：

```math
u =
[f_{FL}, f_{FR}, f_{RL}, f_{RR}, a_j]
```

```math
nu = 12 + nv - 6
```

这里虽然总维度也可能是 `12 + nv - 6`，但前 12 维是 4 个 3D force，不是 2 个 6D wrench。

## 2. 动力学

### 2.1 aligator 的 kinodynamics 定义

aligator 源码注释写得很清楚：

```text
Nonlinear centroidal and full kinematics forward dynamics.
```

状态空间是：

```math
\mathcal{X} = T\mathcal{Q}, \qquad x=(q,v)
```

动力学形式是：

```math
\dot q = v
```

```math
\dot v =
\begin{bmatrix}
a_b \\
a_j
\end{bmatrix}
```

其中：

- `a_j` 是 MPC 控制输入里的 actuated joint acceleration
- `a_b` 不是控制输入，而是由 centroidal Newton-Euler 方程解出来

### 2.2 centroidal momentum 方程

Pinocchio centroidal momentum matrix：

```math
H = A_g(q) v
```

对时间求导：

```math
\dot H =
\dot A_g(q,v) v + A_g(q) \dot v
```

把 `A_g` 按 base / actuated joints 分块：

```math
A_g =
\begin{bmatrix}
A_b & A_j
\end{bmatrix}
```

其中：

- `A_b = A_g.leftCols(6)`
- `A_j = A_g.rightCols(nv-6)`

又因为：

```math
\dot v =
\begin{bmatrix}
a_b \\
a_j
\end{bmatrix}
```

所以：

```math
\dot H =
\dot A_g v + A_b a_b + A_j a_j
```

另一方面，外力产生的 centroidal wrench 为：

```math
w_c =
\begin{bmatrix}
\sum_i f_i + m g \\
\sum_i (p_i - c) \times f_i + \sum_i \tau_i
\end{bmatrix}
```

其中：

- `m` 是总质量
- `g` 是 `settings.gravity`
- `c` 是 CoM 位置
- `p_i` 是第 `i` 个 contact frame 位置
- `f_i` 是 contact force
- `tau_i` 是 6D contact 时的 contact moment；`force_size = 3` 时没有这一项

centroidal Newton-Euler law：

```math
\dot H = w_c
```

于是 base acceleration 由下面的线性方程解出：

```math
A_b a_b =
w_c - \dot A_g v - A_j a_j
```

即：

```math
a_b =
A_b^{-1}
\left(
w_c - \dot A_g v - A_j a_j
\right)
```

这就是 `aligator/modelling/dynamics/kinodynamics-fwd.hxx` 里：

```text
d.Agu_inv_ = inverse(pdata.Ag.leftCols(6))
xdot.segment(nv, 6) =
  Agu_inv * (cforces - pdata.dAg * v - pdata.Ag.rightCols(nv - 6) * a)
```

### 2.3 连续时间 xdot 排列

`KinodynamicsFwdDynamics::forward()` 最终填：

```math
\dot x =
\begin{bmatrix}
\dot q \\
\dot v
\end{bmatrix}
=
\begin{bmatrix}
v \\
a_b \\
a_j
\end{bmatrix}
```

源码对应：

```text
xdot.head(nv) = v
xdot.segment(nv, 6) = a_b
xdot.tail(nv - 6) = a_j
```

注意这里 `xdot.head(nv)` 是 tangent 里的 configuration velocity，不是 `q` 的欧氏差分。

### 2.4 离散积分

`KinodynamicsOCP` 不直接用连续 ODE 作为 stage dynamics，而是包了一层：

```text
IntegratorSemiImplEuler(ode, settings.timestep)
```

半隐式欧拉逻辑是：

1. 先用 ODE 算 `xdot`
2. 先更新 velocity 部分
3. 再用更新后的 velocity 更新 configuration 部分

概念上：

```math
v_{k+1} = v_k + dt \, \dot v_k
```

```math
q_{k+1} = q_k \oplus dt \, v_{k+1}
```

其中 `\oplus` 是 Pinocchio/aligator manifold integrate。

### 2.5 与当前 OCS2 centroidal formulation 的关键区别

两者都用了全身运动学和质心动力学，但决策变量层级不同：

```text
当前 OCS2 centroidal_mpc:
  x = [normalized centroidal momentum h, base pose, joint positions]
  u = [contact wrench, joint velocities]
  base velocity 由 h 和 joint velocity 通过 centroidal momentum matrix 代数恢复

simple-mpc KinodynamicsOCP:
  x = [q, v]
  u = [contact force/wrench, joint accelerations]
  base acceleration 由 centroidal Newton-Euler 方程代数求解
```

所以 simple-mpc 是 acceleration-level kinodynamics，当前 OCS2 是 momentum/velocity-level centroidal kinodynamics。

## 3. OCP stage 构建

每个 shooting node 都由：

```text
KinodynamicsOCP::createStage(contact_phase, contact_pose, contact_force, land_constraint)
```

创建。

输入参数含义：

- `contact_phase[name]`：该足当前 node 是否接触
- `contact_pose[name]`：该足的参考 placement / translation
- `contact_force[name]`：该足的 nominal force / wrench
- `land_constraint[name]`：该 node 是否是该足从 swing 切到 stance 的 landing node

在 `createStage()` 内部：

1. 创建 `MultibodyPhaseSpace(model)`
2. 创建 `CostStack`
3. 根据 `contact_phase` 生成 `contact_states`
4. 把 `contact_force` 写入 `control_ref_` 的前 `feet_nb * force_size` 维
5. 添加 cost
6. 创建 `KinodynamicsFwdDynamics`
7. 包装成 `IntegratorSemiImplEuler`
8. 添加约束

## 4. Cost

所有 stage cost 都加到同一个 `CostStack`。

aligator 的 `QuadraticResidualCost` 形式是：

```math
L = \frac12 r(x,u)^T W r(x,u)
```

`CostStack` 再把各项加起来。

### 4.1 state cost

```text
rcost.addCost("state_cost", QuadraticStateCost(space, nu, reference_state, w_x))
```

residual 是 manifold state difference：

```math
r_x = x \ominus x_{ref}
```

cost：

```math
L_x = \frac12 r_x^T W_x r_x
```

初始 reference 是：

```math
x_{ref} = model_handler.getReferenceState()
```

运行中可以通过：

```text
setReferenceState(t, x_ref)
setPoseBase(t, pose_base)
setVelocityBase(t, velocity_base)
```

修改某个 node 的 state reference。

`setPoseBase()` 修改 target 的 `head<7>()`，也就是 base configuration。

`setVelocityBase()` 修改 target 的 `segment(nq, 6)`，也就是 base velocity。

### 4.2 control cost

```text
rcost.addCost("control_cost", QuadraticControlCost(space, control_ref_, w_u))
```

residual：

```math
r_u = u - u_{ref}
```

cost：

```math
L_u = \frac12 (u-u_{ref})^T W_u (u-u_{ref})
```

`u_ref` 的前 `feet_nb * force_size` 维由 `contact_force` 填入。

joint acceleration reference 默认是 0，因为 `control_ref_` 初始化后只写 contact force/wrench 段。

### 4.3 centroidal momentum cost

```text
CentroidalMomentumResidual(space.ndx(), nu, model, zero_6)
```

aligator residual：

```math
r_H = A_g(q) v - H_{ref}
```

在 simple-mpc 中：

```math
H_{ref} = 0
```

所以：

```math
L_H = \frac12 H^T W_H H
```

它惩罚的是完整 centroidal momentum，不是 normalized momentum。

Talos example 中：

```text
w_cent_lin = [0.0, 0.0, 1]
w_cent_ang = [0.1, 0.1, 10]
```

即更重地压制 vertical linear momentum 和 yaw angular momentum。

### 4.4 centroidal momentum derivative cost

```text
CentroidalMomentumDerivativeResidual(
  ndx, model, gravity, contact_states, feet_frame_ids, force_size)
```

aligator residual 只计算由外力产生的 centroidal wrench：

```math
r_{\dot H} =
\begin{bmatrix}
\sum_i f_i + m g \\
\sum_i (p_i - c) \times f_i + \sum_i \tau_i
\end{bmatrix}
```

其中只有 `contact_states[i] == true` 的足会贡献。

cost：

```math
L_{\dot H} =
\frac12 r_{\dot H}^T W_{\dot H} r_{\dot H}
```

注意：

- 这项不是约束
- 它没有减去一个非零 `\dot H_ref`
- 它倾向于让合外力矩接近 0，尤其是根据权重抑制角动量变化

Talos / Go2 examples 中：

```text
w_centder_lin = [0, 0, 0]
w_centder_ang = [0.1, 0.1, 0.1]
```

所以 linear centroidal wrench 不惩罚，angular centroidal wrench 会被惩罚。

### 4.5 foot pose / translation cost

每个足端都会加一项：

```text
name + "_pose_cost"
```

如果：

```text
force_size == 6
```

使用 6D placement residual：

```math
r_{foot} =
\log\left( T_{ref}^{-1} T_{foot}(q) \right)
```

cost：

```math
L_{foot} =
\frac12 r_{foot}^T W_{frame} r_{foot}
```

如果：

```text
force_size == 3
```

只使用 translation residual：

```math
r_{foot} =
p_{foot}(q) - p_{ref}
```

cost：

```math
L_{foot} =
\frac12 r_{foot}^T W_{frame} r_{foot}
```

所以：

- 6D foot 会跟踪足端位置和姿态
- 3D point foot 只跟踪足端位置

### 4.6 terminal cost

terminal cost 在 `createTerminalCost()` 中创建。

它包含：

```text
state_cost
centroidal_cost
```

没有：

- control cost
- centroidal derivative cost
- foot pose cost

公式：

```math
L_f =
\frac12 (x_N \ominus x_{ref})^T W_x (x_N \ominus x_{ref})
+ \frac12 H_N^T (10 W_H) H_N
```

源码里 terminal centroidal momentum cost 权重是：

```text
settings.w_cent * 10
```

## 5. 约束

### 5.1 kinematics joint position limits

启用条件：

```text
settings.kinematics_limits == true
```

实现：

```text
StateErrorResidual(space, nu, space.neutral())
state_id = [6, 7, ..., nv-1]
FunctionSliceXpr(state_fn, state_id)
BoxConstraint(qmin, qmax)
```

这里 slice 的是 state error 的 configuration tangent 部分，跳过 floating base 的 6 维。

约束形式：

```math
q_{min} \le q_j \le q_{max}
```

其中：

```text
qmin / qmax
```

由 example 从 Pinocchio model position limits 取出。

Talos / Go2 examples 都用：

```text
model.lowerPositionLimit[7:]
model.upperPositionLimit[7:]
```

### 5.2 stance foot velocity equality

启用条件：

```text
contact_phase[name] == true
```

先构造：

```text
FrameVelocityResidual(..., v_ref = 0, frame_id, pinocchio::LOCAL)
```

如果：

```text
force_size == 6
```

直接约束完整 6D frame velocity：

```math
{}^{local}v_{foot} = 0
```

也就是：

```math
\begin{bmatrix}
v_{linear} \\
\omega
\end{bmatrix}
= 0
```

如果：

```text
force_size == 3
```

只取前 3 维线速度：

```math
{}^{local}v_{foot,linear} = 0
```

代码里：

```text
vel_id = {0,1,2}
FunctionSliceXpr(frame_vel, vel_id)
```

因此 point foot 不约束 foot angular velocity。

### 5.3 force cone / wrench cone

启用条件：

```text
contact_phase[name] == true
settings.force_cone == true
```

simple-mpc examples 里 Talos 和 Go2 默认：

```text
force_cone = False
```

所以 example 默认不启用 OCP 内部 contact cone constraint。

如果启用，约束方向是：

```math
h(x,u) \le 0
```

因为用的是 `NegativeOrthant()`。

#### 5.3.1 force_size == 3: friction cone residual

aligator residual 是 2 维：

```math
h_0 = -F_z + \epsilon
```

```math
h_1 = -\mu^2 F_z^2 + F_x^2 + F_y^2
```

加 `NegativeOrthant` 后：

```math
h_0 \le 0 \Rightarrow F_z \ge \epsilon
```

```math
h_1 \le 0 \Rightarrow F_x^2 + F_y^2 \le \mu^2 F_z^2
```

simple-mpc 创建时传：

```text
epsilon = 1e-4
```

注意这里的 residual 索引用的是 `k * 3`，所以它假设 force contact blocks 是连续 3 维。

#### 5.3.2 force_size == 6: wrench cone residual

aligator residual 是 17 维：

1. unilateral contact
2. linearized Coulomb friction, x/y 四个面
3. local CoP x/y bounds
4. z torque bounds

源码 residual 前几项：

```math
h_0 = -F_z
```

```math
h_1 = -F_x - \mu F_z
```

```math
h_2 =  F_x - \mu F_z
```

```math
h_3 = -F_y - \mu F_z
```

```math
h_4 =  F_y - \mu F_z
```

加 `NegativeOrthant` 后等价于：

```math
F_z \ge 0
```

```math
|F_x| \le \mu F_z
```

```math
|F_y| \le \mu F_z
```

CoP 相关项使用：

```math
hW = Wfoot
```

```math
hL = Lfoot
```

其中：

- `M_x, M_y, M_z` 是 contact wrench 后 3 维
- aligator 构造函数参数名是 `half_length` / `half_width`
- simple-mpc 直接把 `settings.Lfoot` / `settings.Wfoot` 传给它，所以这两个配置值按 half-length / half-width 使用

### 5.4 landing z-position constraint

启用条件全部满足时才启用：

```text
force_size != 6
settings.land_cstr == true
land_constraint[name] == true
contact_phase[name] == true
```

也就是说这条只在 3D point foot 模式下使用。

约束形式：

```math
p_{foot,z}(q) - p_{ref,z} = 0
```

代码通过：

```text
FrameTranslationResidual(...)
frame_id = {2}
FunctionSliceXpr(frame_residual, frame_id)
EqualityConstraint()
```

实现。

simple-mpc examples 默认：

```text
land_cstr = False
```

因此默认不启用。

### 5.5 terminal DCM constraint

`createProblem(..., terminal_constraint=true)` 时启用。

`KinodynamicsOCP::createTerminalConstraint()` 创建：

```text
DCMPositionResidual(ndx, nu, model, com_ref, tau)
```

其中：

```math
\tau = \sqrt{\frac{z_{ref}}{9.81}}
```

aligator residual：

```math
r_{dcm} = c(q) + \tau \dot c(q,v) - c_{ref}
```

约束：

```math
r_{dcm} = 0
```

在 Talos / Go2 examples 里 `createProblem(..., False)`，所以默认没有 terminal DCM constraint。

如果启用，MPC 更新中会调用：

```text
updateTerminalConstraint(com_ref)
```

其中 `com_ref` 是 horizon 末端左右/多足参考点平均值再加初始 CoM 高度。

注意 `updateTerminalConstraint()` 只更新 DCM reference，没有重新计算 `tau`；`tau` 是创建 terminal constraint 时由当时的 `com_ref[2]` 固定下来的。

## 6. Contact schedule 与 horizon 更新

### 6.1 createProblem 初始 horizon

`OCPHandler::createProblem()` 会先创建全接触 standing horizon。

每个足端：

```text
contact_phase = true
contact_pose = Identity
contact_force[2] = -mass * gravity / feet_nb
```

注意 simple-mpc examples 中：

```text
gravity = [0,0,-9.81]
```

传入 `createProblem()` 的第四个参数是：

```text
gravity[2] = -9.81
```

所以：

```math
F_z = -m(-9.81)/n_c = \frac{mg}{n_c}
```

### 6.2 generateCycleHorizon

`MPC::generateCycleHorizon(contact_states)` 会把用户给的一段 contact sequence 复制到至少覆盖 problem horizon，然后为每个 phase 创建 stage。

对每个 stage：

```math
F_{z,ref} =
\frac{support\_force}{N_{active}}
```

其中：

- `support_force` 在 examples 里是 `-mass * gravity[2]`
- `N_active` 是该 node 的接触脚数量

如果某足是 swing：

```math
force_ref = 0
```

并且该 stage 的 `contact_state` 中该足为 false。

### 6.3 recedeWithCycle

每次 `MPC::iterate(x)`：

1. 用当前测量状态更新 `RobotDataHandler`
2. horizon 往前滚动
3. 用 cycle horizon 或 standing horizon 替换末端 stage
4. 更新足端 landing / takeoff 计数
5. 更新足端参考和 terminal reference
6. 平移上一轮解 `xs_ / us_`
7. 调用 `SolverProxDDP`

如果当前处于 walking，或者 horizon 末端还不是全支撑，就使用 cycle horizon。

否则切到 standing horizon。

### 6.4 swing foot reference

`FootTrajectory` 使用 Bezier 曲线生成 swing foot translation reference。

给定初末位置：

```math
p_0,\quad p_f
```

控制点中间有一个抬脚 apex：

```math
p_{mid} =
\frac34 p_0 + \frac14 p_f + [0,0,swing\_apex]
```

轨迹用于更新每个 node 的 foot pose cost reference。

在 `MPC::updateStepTrackerReferences()` 中，下一落脚点使用一个 Raibert-like heuristic：

```math
p_{next,xy} =
p_{footref,xy}
+
\left(
v_{base,xy}
+ \omega_{base,z}
\begin{bmatrix}
-(p_{footref,y}-p_{base,y}) \\
 p_{footref,x}-p_{base,x}
\end{bmatrix}
\right)
(T_{fly}+T_{contact}) dt
```

`p_next,z` 取当前足端高度。

然后对 horizon 内每个 time：

```text
setReferencePose(time, foot_name, pose)
```

更新 `name + "_pose_cost"` 的 reference。

## 7. Solver

`MPC` 使用：

```text
SolverProxDDP(settings.TOL, settings.mu_init, maxiters=100)
```

初始化阶段先：

```text
solver.setup(problem)
solver.run(problem, xs, us)
```

然后将：

```text
solver.max_iters = settings.max_iters
```

examples 中通常：

```text
max_iters = 1
```

也就是 receding horizon 每次只做少量 DDP 迭代，依赖上一轮解 warm start。

如果：

```text
num_threads > 1
```

使用 parallel LQ solver。

## 8. MPC 输出

求解后：

```text
xs_ = solver.results.xs
us_ = solver.results.us
Ks_ = solver.results.getCtrlFeedbacks()
```

其中：

```math
x_k = [q_k, v_k]
```

```math
u_k = [\lambda_k, a_{j,k}]
```

### 8.1 获取完整 acceleration

examples 中不是直接把 `u` 当完整 `ddq`。

它先从 dynamics data 取：

```text
a0 = mpc.getStateDerivative(0)[nv:]
```

这段是完整：

```math
\dot v =
\begin{bmatrix}
a_b \\
a_j
\end{bmatrix}
```

然后又显式覆盖 actuated 部分：

```text
a0[6:] = mpc.us[0][nk * force_size:]
```

这和动力学定义一致：

- base acceleration 由 centroidal dynamics 算出
- joint acceleration 来自 control input

### 8.2 contact force / wrench 输出

第一个 node 的接触输出：

```math
\lambda_0 = u_0[0 : n_c n_f]
```

examples 中：

```text
forces0 = mpc.us[0][: nk * force_size]
```

然后 reshape 成每个足端一段 force/wrench。

## 9. inverse dynamics / TSID 使用

simple-mpc 的 examples 并不是把 MPC 输出的力矩直接发送给机器人。

流程是：

1. MPC 输出 `q_ref, v_ref, ddq_ref, contact forces`
2. `Interpolator` 在 MPC node 之间插值
3. `KinodynamicsID` 用 TSID 解 inverse dynamics QP
4. 输出 actuator torque

### 9.1 KinodynamicsID target

examples 中每个 simulation substep 调：

```text
kino_ID.setTarget(q_interp, v_interp, acc_interp, contact_states, force_interp)
tau_cmd = kino_ID.solve(t, q_meas, v_meas)
```

`setTarget()` 设置：

- joint posture task：目标 `q_j, v_j, a_j`
- base SE3 task：目标 base pose，base velocity，base acceleration
- contact tasks：根据 contact state 增删 rigid contact
- contact force reference：MPC 输出的 force/wrench

### 9.2 contact handling in TSID

如果对应足端 contact 为 true：

- point foot 使用 `tsid::contacts::ContactPoint`
- quad foot 使用 `tsid::contacts::Contact6d`
- 设置 `setForceReference(f_target)`

如果 contact 为 false：

- 从 TSID formulation 里移除该 rigid contact

### 9.3 hard constraints in TSID

`KinodynamicsID` 里还会加：

- joint position / velocity bounds
- actuation torque bounds

这些属于 inverse dynamics 层，不属于 kinodynamics NMPC OCP 本体。

### 9.4 一个源码细节

`KinodynamicsID::solve()` 里：

```text
sampleBase_.setDerivative(v_world_aligned.toVector());
sampleBase_.setDerivative(a_world_aligned.toVector());
```

第二行看起来应该是设置 acceleration，但源码里仍然调用了 `setDerivative()`。

如果 TSID 的 `TrajectorySample` API 没有特殊重载，这可能导致 base acceleration reference 没有按预期设置。

这是 ID 层的问题，不影响上面 NMPC OCP 的 formulation。

## 10. Talos example 的主要权重

`examples/talos_kinodynamics.py` 中：

```text
force_size = 6
dt_mpc = 0.01
mu = 0.8
Lfoot = 0.1
Wfoot = 0.075
kinematics_limits = True
force_cone = False
land_cstr = False
```

state weight：

```text
w_basepos = [0, 0, 1000, 1000, 1000, 1000]
w_legpos = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1] * 2
w_torsopos = [1, 1000]
w_armpos = [1, 1, 10, 10] * 2
w_basevel = [10, 10, 10, 10, 10, 10]
w_legvel = [1, 1, 1, 1, 1, 1] * 2
w_torsovel = [0.1, 100]
w_armvel = [10, 10, 10, 10] * 2
w_x = diag(all_above) * 10
```

control weight：

```text
w_linforce = [0.001, 0.001, 0.01]
w_angforce = [0.1, 0.1, 0.1]
w_joint_acc = 1e-4
```

即：

```text
w_u = diag([
  w_linforce, w_angforce,
  w_linforce, w_angforce,
  1e-4 repeated nv-6
])
```

foot pose weight：

```text
w_frame = 100000 * I_6
```

centroidal momentum weight：

```text
w_cent = diag([0, 0, 1, 0.1, 0.1, 10])
```

centroidal derivative weight：

```text
w_centder = diag([0, 0, 0, 0.1, 0.1, 0.1])
```

## 11. Go2 example 的主要权重

`examples/go2_kinodynamics.py` 中：

```text
force_size = 3
dt_mpc = 0.01
mu = 0.8
Lfoot = 0.01
Wfoot = 0.01
kinematics_limits = True
force_cone = False
land_cstr = False
```

state weight：

```text
w_basepos = [0, 0, 100, 10, 10, 0]
w_legpos = [1, 1, 1] * 4
w_basevel = [10, 10, 10, 10, 10, 10]
w_legvel = [0.1, 0.1, 0.1] * 4
w_x = diag(all_above)
```

control weight：

```text
w_linforce = [0.01, 0.01, 0.01] * 4
w_joint_acc = 1e-5
```

foot translation weight：

```text
w_frame = 2000 * I_3
```

centroidal momentum weight：

```text
w_cent = diag([0, 0, 1, 0.1, 0.1, 10])
```

centroidal derivative weight：

```text
w_centder = diag([0, 0, 0, 0.1, 0.1, 0.1])
```

## 12. 哪些东西不在 KinodynamicsOCP 本体里

下面这些容易混淆，但不是 `KinodynamicsOCP` OCP 本体：

- torque bounds：在 `KinodynamicsID` TSID 层
- full rigid-body equation `M(q)\ddot q + h = S^T\tau + J^Tf`：不作为 NMPC 约束出现
- actuator torque `tau`：不是 NMPC 决策变量
- swing foot 速度/加速度 tracking：OCP 里没有专门的 swing velocity cost，主要靠 foot pose/translation cost 随 horizon 更新
- foot collision constraint：simple-mpc kinodynamics OCP 没有
- zero wrench for swing foot：没有显式 equality，但 swing foot `contact_state=false` 后不会进入 dynamics 外力项；如果 force reference 是 0，再加 control cost 会把它压向 0
- cone constraints：代码支持，但 Talos/Go2 examples 默认 `force_cone=False`
- terminal DCM constraint：代码支持，但 Talos/Go2 examples 默认 `terminal_constraint=False`

## 13. 一句话总结

simple-mpc 的 kinodynamics NMPC 核心是：

- 状态直接是全身 `x=[q,v]`
- 输入是 contact force/wrench 加 actuated joint acceleration
- contact force/wrench 通过 centroidal Newton-Euler 方程决定 floating-base acceleration
- actuated joint acceleration 由 MPC 直接优化
- 全身运动学通过 `qdot=v` 和 foot frame residual / velocity equality 进入 OCP
- centroidal momentum 和 centroidal momentum derivative 作为 cost 约束整体动量行为
- 最终 torque 不由 NMPC 直接输出，而是在 example 里交给 TSID inverse dynamics 根据 MPC 的 `q,v,ddq,force` 再求

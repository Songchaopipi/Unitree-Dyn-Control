# humanoid_centroidal_mpc 公式解析

本文基于当前工作区中的 `g1_centroidal_mpc` 源码整理，重点覆盖：

- 状态/输入定义
- 全部实际启用的约束
- 全部实际启用的 cost 和权重
- MPC 输出如何映射到关节力矩

说明：

- 这里写的是当前 centroidal 版本真实生效的逻辑。
- 任务文件里有些字段被读取了，但在 centroidal 这条链路里没有被用到，我会单独标出来。
- `Q` / `R` 这类矩阵都通过 `loadEigenMatrix()` 读取，`scaling` 会乘到每个元素上。

## 1. 模型状态与输入

当前 G1 centroidal 配置：

- `centroidalModelType = 0`，即 `FullCentroidalDynamics`
- active joints = 23
- contact wrenches = 2 个 6D 接触

因此：

- `state_dim = 12 + 23 = 35`
- `input_dim = 6 * 2 + 23 = 35`

### 状态

`x = [h, q_b, q_j]`

- `h = [h_x/m, h_y/m, h_z/m, L_x/m, L_y/m, L_z/m]`
- `q_b = [p_x, p_y, p_z, yaw, pitch, roll]`
- `q_j` 为 23 个 active joint

### 输入

`u = [W_l, W_r, qdot_j]`

- `W_l = [Fx, Fy, Fz, Mx, My, Mz]`
- `W_r = [Fx, Fy, Fz, Mx, My, Mz]`
- `qdot_j` 为 23 个关节速度

### active joint 顺序

`[left_hip_pitch, left_hip_roll, left_hip_yaw, left_knee, left_ankle_pitch, left_ankle_roll,`
` right_hip_pitch, right_hip_roll, right_hip_yaw, right_knee, right_ankle_pitch, right_ankle_roll,`
` waist_yaw, waist_roll, waist_pitch,`
` left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow,`
` right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow]`

6 个 wrist joint 被固定，不进入 MPC。

## 2. 动力学

### 2.1 centroidal 动力学

当前是 full centroidal dynamics。接触输入在世界系里表达，6D contact wrench 直接进入 centroidal momentum rate：

```math
\dot{\bar h} =
\frac{1}{m}
\left[
\sum_i f_i + m g,\;
\sum_i (r_i - c) \times f_i + \tau_i
\right]
```

其中：

- `m` 是机器人质量
- `c` 是质心位置
- `r_i` 是第 `i` 个 contact frame 在世界系的位置
- `f_i, tau_i` 是该 contact wrench 的力和力矩

因为当前是 6D contact，所以每个足端都贡献 `f_i` 和 `tau_i`。

### 2.2 Pinocchio generalized velocity 映射

`CentroidalModelPinocchioMapping` 把 centroidal state/input 映射到 Pinocchio 的 generalized coordinates / velocities。

对 full centroidal 模型：

```math
v_j = qdot_j
```

```math
v_b = A_b^{-1}\left(m\,h - A_j qdot_j\right)
```

这里：

- `A = [A_b, A_j]` 是 centroidal momentum matrix
- `A_b` 是前 6 列
- `qdot_j` 就是 MPC input 里的 joint velocity

于是：

```math
v_{pin} = [v_b, qdot_j]
```

### 2.3 运行时实现

`PinocchioCentroidalDynamicsAD` 最终做的是：

```math
\dot x = [\dot h,\; \dot q_b,\; \dot q_j]
```

其中：

- `\dot h` 来自上面的 centroidal momentum rate
- `[\dot q_b, \dot q_j]` 来自 `getPinocchioJointVelocity()`

## 3. 参考与 nominal

### 3.1 state nominal

`StateInputQuadraticCost` 里的 `x_nominal` 来自 `referenceManager->getDesiredState()`。

它不是纯粹的 target trajectory 原样输出，还会叠加 arm swing reference：

- shoulder / elbow 会按 gait phase 做小幅修正

### 3.2 input nominal

`u_nominal = weightCompensatingInput()`

公式是：

```math
u_{nom} = [0, 0, 0, 0, 0, 0,\;
0, 0, 0, 0, 0, 0,\;
0_{23}]
```

但如果当前有 `N` 个 stance feet，则每个 stance foot 的法向力为：

```math
F_z = \frac{m g}{N}
```

其余分量全为 0。

### 3.3 target trajectory

命令轨迹由 `CentroidalMpcTargetTrajectoriesCalculator` 生成：

- 位移命令会生成目标 base pose
- 速度命令会先滤波，再积分成目标 base pose / momentum

当前默认：

- `defaultBaseHeight = 0.7925`
- `defaultJointState` 见 `reference.info`

## 4. Cost

### 4.1 基础二次 cost

`StateInputQuadraticCost` 的形式是：

```math
L = \frac12 (x-x_{nom})^T Q (x-x_{nom})
  + \frac12 (u-u_{nom})^T R (u-u_{nom})
```

这里的 `Q` / `R` 都是 task file 里的矩阵，`scaling` 已经乘入元素。

### 4.2 Q

`Q` 的 effective diagonal：

```text
momentum: [8, 8, 15, 15, 15, 4]
base:     [0, 0, 15, 0, 5, 5]
joints:
[0.02, 0.06, 4, 0.02, 0.01, 0.01,
 0.02, 0.06, 4, 0.02, 0.01, 0.01,
 2, 0.5, 0.5,
 10, 20, 2, 2,
 10, 20, 2, 2]
```

### 4.3 R

`R` 的 task file 里 `scaling = 1e-3`，所以 effective 值是 raw 值乘 `1e-3`：

```text
left wrench:
[5e-5, 5e-5, 1e-5, 5e-5, 5e-5, 2e-4]

right wrench:
[5e-5, 5e-5, 1e-5, 5e-5, 5e-5, 2e-4]

joint velocity:
[0.02, 0.02, 0.2, 0.02, 0.02, 0.02,
 0.02, 0.02, 0.2, 0.02, 0.02, 0.02,
 2, 0.8, 2,
 0.2, 0.1, 0.1, 0.2,
 0.2, 0.1, 0.1, 0.2]
```

### 4.4 terminal cost

终端项是：

```math
L_f = \frac12 (x-x_f)^T (3 Q_{final}) (x-x_f)
```

`terminalCostScaling = 3.0`。

`Q_final` 的 effective diagonal：

```text
momentum: [75, 75, 75, 75, 75, 75]
base:     [0, 0, 60, 0, 15, 15]
joints:
[0.06, 0.18, 24, 0.06, 0.03, 0.03,
 0.06, 0.18, 24, 0.06, 0.03, 0.03,
 6, 1.5, 1.5,
 30, 60, 6, 6,
 30, 60, 6, 6]
```

### 4.5 task-space foot cost

每个 foot 的 residual 是 12 维：

```math
e_{foot} =
\begin{bmatrix}
p - p_{ref} \\
\Delta q_{plane}(R, n) \\
s_{imp}(t) \, (v - v_{ref}) \\
\omega - \omega_{ref}
\end{bmatrix}
```

其中：

- `\Delta q_{plane}(R, n)` 是 foot z-axis 与 plane normal 的最短旋转误差
- `s_imp(t)` 是 impact proximity factor

当前 `CentroidalMpcEndEffectorFootCost` 里 reference 被硬编码成：

- `p_ref = 0`
- `n = [0,0,1]`
- `v_ref = 0`
- `\omega_ref = 0`

所以它不是“跟踪某个脚步轨迹”，而是“保持 foot 姿态对平面、并在接近落脚时抑制速度”。

effective weights（只用 12 维，acceleration 字段在 centroidal 里没用）：

```text
foot:
[0, 0, 0,
 1000, 1000, 0,
 10, 10, 0,
 1, 1, 0.005]
```

注意：

- `pos_* = 0`，所以 foot cost 不直接惩罚位置
- `orientation_z = 0`，所以不惩罚 yaw
- `lin_velocity_z = 0`
- `ang_velocity_z = 0.005`
- task file 里额外的 `lin_acceleration_*` / `ang_acceleration_*` 字段在这个 centroidal 版本里未被读取

### 4.6 torso task-space cost

`task_space_costs.torso` 作用在 `mid360_link` 上，12 维 residual 是：

```math
e_{torso} =
\begin{bmatrix}
p - p_{ref} \\
\Delta q(q, q_{ref}) \\
v - v_{ref} \\
\omega - \omega_{ref}
\end{bmatrix}
```

`q_ref` / `v_ref` 来自 target trajectory 在该时刻的参考状态。

effective weights：

```text
torso:
[0, 0, 0,
 100, 100, 0,
 0.1, 0.1, 0.005,
 5, 5, 2]
```

### 4.7 ICP cost

代码名叫 `ICPCost`，但当前实现并没有用真正的 ICP 公式，而是：

```math
e_{icp} = p_{com,xy}^{des} - p_{com,xy}
```

其中：

- `p_{com,xy}^{des}` = 两个 contact points 的水平中点
- `p_{com,xy}` = 当前 CoM 水平位置

真正的 `com + v/omega` 形式在源码里被注释掉了。

当前：

- `icpErrorWeight = 0.0`
- 所以这个 cost 实际上关闭

如果未来打开，它是：

```math
L_{icp} = \frac12 w \|e_{icp}\|^2
```

左右两个轴权重相同。

### 4.8 external torque cost

每条腿一个 `ExternalTorqueQuadraticCostAD`，residual 是：

```math
r_{\tau} = (1 - s_{imp,opp}(t)) \; \sqrt{W_\tau} \; \tau_{ext,active}
```

其中：

- `\tau_{ext} = J_{ee}^T W`
- `\tau_{ext,active}` 只取 active joints 的分量
- `s_{imp,opp}(t)` 用的是对侧 swing foot 的 impact factor

因此 cost 实际上是：

```math
L_\tau = \frac12 (1 - s_{imp,opp}(t))^2 \; \tau_{ext,active}^T W_\tau \tau_{ext,active}
```

左右腿的 effective weights（task file raw 值乘 `1e-4`）：

```text
[2e-4, 2e-4, 1e-4, 8e-4, 2e-5, 2e-5]
```

对应关节：

- hip pitch
- hip roll
- hip yaw
- knee
- ankle pitch
- ankle roll

## 5. 约束

### 5.1 stance foot zero velocity constraint

这是 `ZeroVelocityConstraintCppAd` + `EndEffectorKinematicsTwistConstraint`。

启用条件：

- 当前 foot 在 contact 中

形式：

```math
g = A_x \, \xi + A_v \, \upsilon + b = 0
```

其中 `\xi` 是 foot pose 6 维：

```math
\xi = [p_x, p_y, p_z, e_{ori,x}, e_{ori,y}, e_{ori,z}]
```

当前 config：

- `A_v = I_6`
- `A_x(2,2) = 5.0`
- `A_x(3:5,3:5) = 20 I_3`
- 其余位置为 0

因此实际 residual 是：

```math
\begin{bmatrix}
v_x \\
v_y \\
v_z + 5(z - z_{ref}) \\
\omega_x + 20 e_x \\
\omega_y + 20 e_y \\
\omega_z + 20 e_z
\end{bmatrix}
= 0
```

这里：

- `z_ref` 来自 swing planner 的 foot height reference
- 只有 `z` 和 orientation 被额外拉回
- `x/y` position 没有直接约束

### 5.2 swing foot normal velocity constraint

启用条件：

- 当前 foot 不在 contact

形式：

```math
g = v_z - \dot z_{ref} + k_z (z - z_{ref}) = 0
```

其中：

- `k_z = positionErrorGain_z = 5.0`
- `z_ref` 和 `\dot z_ref` 来自 swing trajectory planner

这条约束只约束法向速度，不约束平面内速度。

### 5.3 zero wrench constraint

启用条件：

- 当前 foot 不在 contact

形式：

```math
W_i = 0
```

即 6 个分量都为 0。

### 5.4 friction cone soft constraint

启用条件：

- 当前 foot 在 contact

当前实现里 `setSurfaceNormalInWorld()` 没被用上，`t_R_w = I`，所以这个 cone 基本就是世界系 z 轴方向的 cone。

约束：

```math
g_f = \mu (F_z + g_r) - \sqrt{F_x^2 + F_y^2 + \epsilon} \ge 0
```

当前参数：

- `frictionCoefficient = 0.4`
- `gripperForce = 0`
- `regularization = 25`
- `hessianDiagonalShift = 1e-6`

对应的软罚函数是 `RelaxedBarrierPenalty(mu=0.2, delta=5.0)`。

因为它是 soft constraint，实际 cost 是：

```math
L_f = p(g_f)
```

其中 `p()` 是 relaxed barrier。

### 5.5 contact moment XY soft constraint

启用条件：

- 当前 foot 在 contact

这是一个 CoP / contact moment 的矩形边界。设 contact frame local wrench 为：

```math
[F_x, F_y, F_z, M_x, M_y, M_z]
```

则：

```math
x_{cop} = -\frac{M_y}{F_z}, \qquad
y_{cop} = \frac{M_x}{F_z}
```

接触矩形：

```text
x in [-0.09, 0.09]
y in [-0.03, 0.03]
```

等价的 4 个线性不等式是：

```math
M_x - y_{min} F_z \ge 0
-M_x + y_{max} F_z \ge 0
-M_y - x_{min} F_z \ge 0
 M_y + x_{max} F_z \ge 0
```

当前 soft penalty：

- `RelaxedBarrierPenalty(mu=0.6, delta=0.03)`

### 5.6 joint limits soft constraint

启用条件：

- 始终启用

实际 joint bounds 来自 URDF / Pinocchio，不写死在 task 文件里。

对每个 joint `q_i`：

```math
L_{jl} = \sum_i p(q_i^{max} - q_i) + p(q_i - q_i^{min})
```

其中 `p()` 是 `PieceWisePolynomialBarrierPenalty`：

```math
p(h)=
\begin{cases}
\mu \left(\frac12 h^2 - \frac{\delta}{2}h + \frac{\delta^2}{6}\right), & h \le 0 \\
\mu \left(-\frac{h^3}{6\delta} + \frac12 h^2 - \frac{\delta}{2}h + \frac{\delta^2}{6}\right), & 0 < h < \delta \\
0, & h \ge \delta
\end{cases}
```

当前参数：

- `mu = 1200`
- `delta = 0.1`

### 5.7 foot collision soft constraint

启用条件：

- 如果两只脚同时在 contact，这条约束会被关闭，避免和 stance foot 约束打架
- 其他情况下启用

它检查 16 个球心距离，形式统一为：

```math
d_{ij}(q) = \|p_i(q) - p_j(q)\| - 2 r \ge 0
```

当前参数：

- foot sphere radius = `0.065`，所以最小距离 `0.13`
- knee sphere radius = `0.07`，所以最小距离 `0.14`
- soft penalty = `PieceWisePolynomialBarrierPenalty(mu=30000, delta=0.05)`

16 个具体约束：

1. `foot_l_contact_collision_p_1` vs `foot_r_contact_collision_p_1`
2. `foot_l_contact_collision_p_1` vs `foot_r_contact_collision_p_2`
3. `foot_l_contact_collision_p_2` vs `foot_r_contact_collision_p_1`
4. `foot_l_contact_collision_p_2` vs `foot_r_contact_collision_p_2`
5. `foot_l_contact` vs `foot_r_contact_collision_p_1`
6. `foot_l_contact` vs `foot_r_contact_collision_p_2`
7. `foot_r_contact` vs `foot_l_contact_collision_p_1`
8. `foot_r_contact` vs `foot_l_contact_collision_p_2`
9. `foot_l_contact` vs `foot_r_contact`
10. `left_knee_joint` vs `right_knee_joint`
11. `foot_l_contact` vs `right_ankle_roll_joint`
12. `foot_l_contact_collision_p_1` vs `right_ankle_roll_joint`
13. `foot_l_contact_collision_p_2` vs `right_ankle_roll_joint`
14. `foot_r_contact` vs `left_ankle_roll_joint`
15. `foot_r_contact_collision_p_1` vs `left_ankle_roll_joint`
16. `foot_r_contact_collision_p_2` vs `left_ankle_roll_joint`

### 5.8 joint mimic constraint

代码里有，但当前 G1 centroidal task file 没有 `mimicJoints` 段，所以不会被加到 OCP。

如果未来启用，其公式是：

```math
g = k_p(m q_{parent} - q_{child}) + (m \dot q_{parent} - \dot q_{child}) = 0
```

其中：

- `m` 是 multiplier
- `k_p` 是 positionGain

## 6. 关节力矩映射

MPC 的输入不是关节力矩，而是：

```math
u = [W_l, W_r, qdot_j]
```

最终送到执行器的 `feed_forward_effort` 是通过逆动力学算出来的。

### 6.1 contact wrench -> generalized force

对每个 foot：

```math
\tau_{ext} = J_l^T W_l + J_r^T W_r
```

这里的 Jacobian 是 `LOCAL_WORLD_ALIGNED`。

### 6.2 逆动力学

`computeJointTorques()` 先算：

- `M(q)` mass matrix
- `nle(q, qd)` 非线性项

再把 contact wrench 投影到 joint space。

浮动基座加速度按 block 形式解：

```math
\ddot q_b = M_{bb}^{-1} \left(-nle_b - M_{bj}\ddot q_j + \tau_{ext,b}\right)
```

然后 actuated joint torque：

```math
\tau_j = M_{jb}\ddot q_b + M_{jj}\ddot q_j + nle_j - \tau_{ext,j}
```

### 6.3 当前代码里的一个重要事实

`CentroidalMpcMrtJointController` 里：

- `inverse_dynamics_kp_` 全部被置 0
- `inverse_dynamics_kd_` 全部被置 0

所以：

```math
\ddot q_j^{des} = 0
```

也就是说当前 feed-forward torque 本质上是：

```math
\tau_{ff} = ID(q, \dot q, \ddot q_j = 0, W_l, W_r)
```

不是一个带 PD acceleration tracking 的版本。

### 6.4 最终给执行器的量

对 MPC joints：

- `q_des = mpc_q_j_des`
- `qd_des = mpc_qd_j_des`
- `kp = 1200`
- `kd = 10`
- `feed_forward_effort = tau_ff`

固定 wrist joints：

- `q_des = 0`
- `qd_des = 0`
- `kp = 100`
- `kd = 1`
- `feed_forward_effort = 0`

## 7. 哪些配置字段在 centroidal 里没有生效

这个部分很重要，避免把“读进来了”误当成“真的用了”。

- `foot_constraint.linearVelocityErrorGain_xy`
- `foot_constraint.linearVelocityErrorGain_z`
- `foot_constraint.angularVelocityErrorGain`
- `foot_constraint.linearAccelerationErrorGain_z`
- `foot_constraint.linearAccelerationErrorGain_xy`
- `foot_constraint.angularAccelerationErrorGain`

这些在当前 centroidal 代码里没有被使用。

另外：

- `task_space_foot_cost_weights.lin_acceleration_*`
- `task_space_foot_cost_weights.ang_acceleration_*`

也没有被当前 centroidal foot cost 读取。

再加上：

- `mimicJoints` 在当前 task file 里不存在，所以 knee mimic 约束不会启用
- `ICPCost` 当前权重为 0，所以这个 cost 关闭
- friction cone 的 `setSurfaceNormalInWorld()` 没有实际启用，cone 仍按 identity 处理

## 8. 一句话总结

这套 centroidal MPC 的核心就是：

- 状态是 normalized centroidal momentum + base pose + 23 个关节角
- 输入是两只脚的 6D wrench + 23 个关节速度
- 运行时用 `Q/R` 做 state/input 二次型
- 用 stance/swing 约束把足端速度、法向速度、零 wrench、摩擦锥、CoP、碰撞、关节限位都压住
- 最终力矩不是 MPC 直接输出，而是把 MPC 的 wrench 通过逆动力学投影成关节 `feed_forward_effort`


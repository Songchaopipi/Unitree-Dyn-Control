# g1_srbd_ik_walk 与 g1_srbd_ik_walk_rp 的 MPC 公式和控制 Pipeline 对比

本文对比两条 demo/control pipeline：

- `demo/g1_srbd_ik_walk.cpp` + `algorithm/g1_srbd_mpc.cpp`
- `demo/g1_srbd_ik_walk_rp.cpp` + `algorithm/g1_srbd_mpc_rp.cpp`

一句话概括：`g1_srbd_ik_walk` 是纯 foot contact wrench 的 SRBD-MPC；`g1_srbd_ik_walk_rp` 在同一个 SRBD-MPC 框架上额外引入 torso roll/pitch reaction torque，并把该力矩前馈到腰部 roll/pitch 电机。

## 1. 总体差异

| 项目 | `g1_srbd_ik_walk` | `g1_srbd_ik_walk_rp` |
| --- | --- | --- |
| MPC class | `G1SrbdMpc` | `G1SrbdMpcRp` |
| 状态维度 | 13 | 17 |
| 输入维度 | 12 | 14 |
| 优化输入 | 左右脚 plane-force 参数 | 左右脚 plane-force 参数 + torso roll/pitch torque |
| 对 base 姿态的作用通道 | 只能通过接触 wrench 产生角动量 | 接触 wrench + 腰部 RP 反作用力矩 |
| QP 输出 | `firstPlaneForces`, `firstWrenches` | 额外输出 `firstTorsoTorqueRp` |
| 电机前馈 | 腿部 `-J^T f` | 腿部 `-J^T f` + 腰部 RP torque |
| Swing foot planner | 共用 `FootPlacement` | 共用 `FootPlacement` |
| IK | 腿 IK + 上身 locked pose | 腿 IK + 上身 locked pose |

## 2. MPC 公式差异

### 2.1 标准版 SRBD-MPC

标准版状态是 13 维：

```text
x = [
  base_rpy(3),
  com_pos_W(3),
  base_omega_W(3),
  com_vel_W(3),
  g
]
```

输入是左右脚各 6 维 plane-force 参数：

```text
u = [
  u_left_foot_plane(6),
  u_right_foot_plane(6)
]
```

每只脚先由 `planeForceToWrenchMap()` 映射到足端 wrench：

```text
w_i = H_contact * u_i
w_i = [F_i; tau_i]
```

再由足端相对 CoM 的力臂 `r_i = p_foot_i - p_com` 映射到质心动力学：

```text
centroidal_wrench_i = [
  F_i
  r_i x F_i + tau_i
]
```

连续动力学可概括为：

```text
rpy_dot   = Rz(yaw) * omega
p_dot     = v
omega_dot = I_W^-1 * sum_i (r_i x F_i + tau_i)
v_dot     = (1 / m) * sum_i F_i - [0, 0, g]^T
g_dot     = 0
```

代码中使用一阶离散化：

```text
A_d = I + dt * A_c
B_d = dt * B_c
```

然后堆叠 horizon 得到 condensed form：

```text
X = A_qp * x0 + B_qp * U
```

QP 的基本目标函数为：

```text
min_U ||A_qp x0 + B_qp U - X_ref||_Q^2
    + ||U||_R^2
    + nominal_force_weight * ||U - U_nom||^2
```

其中标准版状态权重为：

```text
rpy:   80, 80, 120
pos:   120, 120, 220
omega: 1, 1, 4
vel:   40, 40, 20
g:     0
```

约束包括每只脚的 wrench/friction cone、每只脚 6 维输入上下界、非接触脚输入收紧为 0。

### 2.2 RP 版 SRBD-MPC

RP 版在标准状态后面增加 torso roll/pitch 的角度和速度：

```text
x_rp = [
  base_rpy(3),
  com_pos_W(3),
  base_omega_W(3),
  com_vel_W(3),
  g,
  torso_roll,
  torso_pitch,
  torso_roll_rate,
  torso_pitch_rate
]
```

输入也增加 2 维腰部 roll/pitch torque：

```text
u_rp = [
  u_left_foot_plane(6),
  u_right_foot_plane(6),
  tau_torso_roll,
  tau_torso_pitch
]
```

接触 wrench 对 COM 和 base 的作用与标准版完全一致。新增部分是 torso torque 对 base 角速度的反作用力矩：

```text
omega_dot += -I_W^-1 * R_WB * S_rp * tau_torso_rp

S_rp = [
  1 0
  0 1
  0 0
]
```

负号表示优化变量是施加在 torso roll/pitch 轴上的驱动力矩，而 base/lower body 在 SRBD 方程里受到相反方向的 reaction torque。

同时 RP 版给 torso roll/pitch 自身加一个二阶近似模型：

```text
torso_rp_dot      = torso_rp_rate
torso_rp_rate_dot = I_torso^-1 * tau_torso_rp
```

所以 QP 尺寸变为：

```text
nx = 17
nu = 14
nVar = 14 * horizon
nState = 17 * horizon
```

RP 版 cost 在标准版基础上增加：

```text
torso_roll/pitch weight      = 10, 10
torso_roll/pitch_rate weight = 1, 1
torso torque input weight    = 1e-3, 1e-3
```

另外，RP 版还增加 torso torque 平滑项：

```text
sum_k ||tau_torso_rp[k] - tau_torso_rp[k-1]||^2 * torsoTorqueRateWeight
```

默认参数：

```text
torsoTorqueMaxRoll  = 30 Nm
torsoTorqueMaxPitch = 50 Nm
torsoTorqueRateWeight = 1e-3
torsoInertiaRoll/Pitch = 1.0
```

约束方面，RP 版保留所有 foot contact 约束，并额外加入每个 knot 的 torso torque box constraint：

```text
-torsoTorqueMaxRoll  <= tau_roll  <= torsoTorqueMaxRoll
-torsoTorqueMaxPitch <= tau_pitch <= torsoTorqueMaxPitch
```

一个实现细节：标准版的 nominal force cost 加在全部 12 维输入上；RP 版只把 nominal force cost 加到前 12 维 contact input，不把 torso torque 拉向 nominal foot load。

## 3. Control Pipeline 对比

两条 demo 的主循环结构基本一致：

```text
MuJoCo sensor
  -> MJ_Interface 写 DataBus
  -> Pin_KinDyn 更新 kinematics/dynamics/Jacobian
  -> JoyStickInterpreter 生成速度/位置命令
  -> GaitScheduler 更新接触相位
  -> FootPlacement 生成 swing foot 当前目标和最终落脚点
  -> previewContactTable / previewContactPositions
  -> buildWalkingMpcInput
  -> solve MPC
  -> leg IK
  -> contact torque feedforward
  -> PVT joint control
  -> MuJoCo motor torque
```

### 3.1 Gait 和 swing foot

两版都使用同一套 gait/swing pipeline：

- `GaitScheduler` 更新 `walk_left_contact`、`walk_right_contact`、`walk_stance_leg`、`phi`、`tSwing` 等。
- `FootPlacement` 生成 `swingDesPosCur_W` 和 `swingDesPosFinal_W`。
- 当前 swing foot 位置给 IK 使用。
- 最终落脚点给 MPC horizon 里的 future contact position 使用。

因此，两版的落脚点规划、Bezier 摆腿轨迹、摆腿高度、接触预览逻辑不是核心差异。

### 3.2 MPC input 构造

标准版 `buildWalkingMpcInput()` 写入 13 维状态：

```text
base_rpy, pCoM_W, base_omega_W, dq.head(3), g
```

RP 版写入 17 维状态，额外加入当前腰部 roll/pitch 电机状态：

```text
motors_pos_cur[13], motors_pos_cur[14],
motors_vel_cur[13], motors_vel_cur[14]
```

其中：

```text
13 = waist_roll
14 = waist_pitch
```

两版的 reference 都追踪：

```text
rpy_ref = [0, 0, yaw_ref]
com_ref = comDes + k * dt * velDesWorld
omega_ref = [0, 0, yaw_rate_ref]
vel_ref = velDesWorld
g_ref = g
```

RP 版额外把 torso roll/pitch 和 rate 的 reference 设为 0：

```text
torso_roll_ref = 0
torso_pitch_ref = 0
torso_roll_rate_ref = 0
torso_pitch_rate_ref = 0
```

### 3.3 MPC solve 和输出接线

标准版求解后保存：

```text
lastMpcPlaneForces = srbdMpc.firstPlaneForces()
lastMpcWrench      = srbdMpc.firstWrenches()
robotState.Fr_ff   = lastMpcWrench
```

RP 版多保存：

```text
lastMpcTorsoTorqueRp = srbdMpc.firstTorsoTorqueRp()
```

注意：两个 MPC class 的 `dataBusWrite()` 都只写 `Fr_ff` 和 `qpStatus_MPC`。RP torque 没有进入 `DataBus` 字段，而是在 `g1_srbd_ik_walk_rp.cpp` 中由 demo 直接读取并传给 feedforward torque 函数。

### 3.4 电机 torque feedforward

标准版只把左右脚 plane-force 通过足端 Jacobian 映射成腿部前馈力矩：

```text
tau_leg_ff = -J_foot^T * u_foot_plane
```

代码结构是：

```text
left_leg_tau  -= J_l_foot^T * planeForces_left
right_leg_tau -= J_r_foot^T * planeForces_right
```

RP 版先做同样的腿部 `-J^T f`，再把 MPC 输出的 torso torque 加到腰部电机：

```text
tau_waist_roll_ff  += tau_torso_roll
tau_waist_pitch_ff += tau_torso_pitch
```

最后两版都会经过：

```text
feedforward ramp
motor torque limit
PVT_Ctr::calMotorsPVT()
mjInterface.setMotorsTorque()
```

### 3.5 COM reference 差异

这不是 MPC class 内部差异，而是两个 demo 的 walking pipeline 差异。

标准版 walking 时的水平 COM 目标更像“从初始 COM 出发，跟随 joystick 位置命令的限幅偏移”：

```text
com_des_xy = initialCom_xy
           + clamp(js_pos_des_xy - initialBasePos_xy, walk_com_clamp)
```

RP 版改成“从当前实测 COM 出发，朝目标 COM 小步纠偏”：

```text
target_xy = initialCom_xy
          + clamp(js_pos_des_xy - initialBasePos_xy, walk_com_clamp)

com_des_xy = current_com_xy
           + clamp(target_xy - current_com_xy, kWalkComCorrectionClamp)
```

其中：

```text
kWalkComCorrectionClamp = 0.035
```

这会让 RP demo 的 COM reference 更贴近当前状态，避免一次性给出过大的水平 COM 跳变。

## 4. 数据流总结

标准版数据流：

```text
DataBus
  -> G1SrbdMpc::Input
  -> solve contact-wrench QP
  -> firstPlaneForces / firstWrenches
  -> robotState.Fr_ff
  -> tau_ff = -J_foot^T * planeForces
  -> motors_tor_des
  -> PVT output torque
```

RP 版数据流：

```text
DataBus
  -> G1SrbdMpcRp::Input
     includes waist roll/pitch pos/vel
  -> solve contact-wrench + torso-torque QP
  -> firstPlaneForces / firstWrenches / firstTorsoTorqueRp
  -> robotState.Fr_ff
  -> tau_ff = -J_foot^T * planeForces + waist_torso_tau
  -> motors_tor_des
  -> PVT output torque
```

## 5. 实际调试时看哪些量

如果两版步态表现不同，优先看这些量：

| 现象 | 优先检查 |
| --- | --- |
| base roll/pitch 抖动 | RP torque 是否饱和、`torsoTorqueRateWeight` 是否太小、腰部 PD 是否过硬 |
| 接触力异常 | `Fr_ff`、`firstPlaneForces`、contact table、非接触脚是否被置零 |
| COM 轨迹不同 | 标准版和 RP 版的 `comDes` 生成逻辑不同 |
| 腰部动作异常 | `lastMpcTorsoTorqueRp`、waist roll/pitch motor index 13/14、motor torque limit |
| 摆腿位置不同 | 通常不是 MPC 公式差异，检查 `FootPlacement`、`swingDesPosCur_W`、`swingDesPosFinal_W` |

## 6. 结论

`g1_srbd_ik_walk.cpp` 的 MPC 只优化脚底接触 wrench，本质是 contact-only SRBD-MPC。它通过调整左右脚接触力和力矩来控制 COM、base yaw/roll/pitch 和速度。

`g1_srbd_ik_walk_rp.cpp` 的 MPC 在 contact-only SRBD-MPC 上增加了 torso roll/pitch 的状态和输入，使 QP 可以直接优化腰部 roll/pitch torque，并通过反作用力矩影响 base roll/pitch dynamics。控制 pipeline 上，它把 `firstTorsoTorqueRp()` 直接加到 waist roll/pitch 电机前馈中，所以公式里的新增 torque 输入确实接到了执行端。

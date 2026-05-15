# G1 SRBD-MPC / Full-SRB 控制与仿真框架总结

对应常用运行方式：

```bash
python theory_anylisis/run_g1_srbd_full_srb_analysis.py
```

参数在 `run_g1_srbd_full_srb_analysis.py` 顶部的 `CONFIG = RunConfig(...)`
里直接修改。

这个目录是一个 Python 版研究沙盒，用来验证：用较简化的 SRBD-MPC 控制器，是否能够稳定一个更完整的单刚体 Full-SRB 动力学仿真对象。它不追求复现 MuJoCo 全身机器人，而是隔离“控制模型”和“仿真模型”的差异。

## 1. 总体闭环

主循环在 `run_g1_srbd_full_srb_analysis.py` 中完成：

```text
初始 G1 torso / 足端参数
  -> HLIP/LQR 落脚点规划
  -> 接触时序与未来足端位置 preview
  -> OSQP 求解 SRBD-MPC
  -> 取第一拍足端 wrench
  -> Full-SRB 非线性刚体积分
  -> 记录 log / 生成 3D 动画帧
```

站立阶段默认先用 `StableStandMpc` 稳定双支撑；进入行走后切换到 `OsqpSrbdMpc`。控制器输出的是左右脚的 6 维 wrench：

```text
[Fx, Fy, Fz, Mx, My, Mz]left + [Fx, Fy, Fz, Mx, My, Mz]right
```

动画中：橙色脚表示当前接触，灰色脚表示非接触，红色箭头表示地面反力，绿色叉表示下一落脚目标。

## 2. 关键模块

- `run_g1_srbd_full_srb_analysis.py`：实验入口、顶部参数配置、主闭环、日志和动画调用。
- `models.py`：机器人、MPC、步态、仿真的参数 dataclass。
- `alip_footstep.py`：HLIP/LQR 风格落脚点规划和接触 preview。
- `osqp_srbd_mpc.py`：行走阶段 condensed SRBD-MPC QP。
- `stable_stand_mpc.py`：站立阶段的一步 centroidal wrench 分配 QP。
- `full_srb_sim.py`：Full-SRB plant，使用欧拉 / SO(3) 积分。
- `math_utils.py`：旋转、反对称矩阵、足端 wrench 约束和名义竖直 wrench 工具。
- `visualize_srb.py`：Matplotlib 3D 回放。
- `osqp_compat.py`：兼容不同 OSQP 版本的 `polishing` / `polish` 参数名。

## 3. 仿真对象：Full-SRB plant

仿真状态为：

```text
R: 姿态旋转矩阵
p: 质心位置
omega: 世界系角速度
v: 质心速度
```

`full_srb_sim.py` 使用更完整的单刚体转动方程：

```text
I_W * omega_dot + omega x (I_W * omega) = sum_i ((p_foot_i - p_com) x f_i + tau_i)
v_dot = sum_i f_i / m - g
R_next = exp(dt * omega) * R
```

也就是说，plant 中保留了陀螺项 `omega x I omega`，而 MPC 内部模型更简化。这正是该实验想测试的模型失配。

## 4. 控制器设计

### 4.1 站立控制 `StableStandMpc`

站立时默认启用：

```bash
--stable-stand
```

它先根据 COM / RPY 误差构造期望 centroidal wrench：

```text
force_des = m * (kp_pos * pos_err + kd_pos * vel_err + gravity)
moment_des = I_W * alpha_des + omega x I_W omega
```

默认增益：

```text
kp_pos = [18, 18, 70]
kd_pos = [9, 9, 18]
kp_rpy = [70, 70, 35]
kd_omega = [10, 10, 6]
```

然后通过 QP 把期望 6 维 centroidal wrench 直接分配到左右脚 6 维 foot wrench，并满足摩擦锥、法向力和 wrench 上下限。

### 4.2 行走控制 `OsqpSrbdMpc`

行走阶段使用 condensed SRBD-MPC。状态维度 `nx=13`：

```text
[roll, pitch, yaw,
 com_x, com_y, com_z,
 omega_x, omega_y, omega_z,
 vel_x, vel_y, vel_z,
 gravity]
```

控制维度 `nu=12`，为左右脚各 6 维 foot wrench：

```text
[Fx, Fy, Fz, Mx, My, Mz]left
+ [Fx, Fy, Fz, Mx, My, Mz]right
```

MPC 内部离散模型近似为：

```text
rpy_dot = Rz(yaw) * omega
p_dot = v
omega_dot = I_W^-1 * centroidal_moment
v_dot = force / m - g
```

代价函数主要包含：

```text
状态跟踪误差
输入正则
名义竖直支撑力正则
```

接触切换时会强制重新求解 MPC，避免新支撑脚在切换瞬间继续沿用旧 wrench。

## 5. 足步与接触规划

`AlipFootstepPlanner` 模拟 C++ 中 `GaitScheduler + FootPlacement` 的核心逻辑：

- 双支撑时左右脚都接触。
- 单支撑时只有 stance leg 接触。
- swing leg 的下一落脚点由 HLIP/LQR 规则计算。
- MPC horizon 内生成 `contact_table` 和 `contact_positions` preview。

横向落脚步长有硬限制：

```text
lateral_step_min = 0.10 m
lateral_step_max = 0.60 m
```

默认初始支撑脚：

```text
initial_stance = left
```

## 6. 主要参数设计

### 6.1 G1 torso / 足底参数

来自 `models.py`：

```text
torso mass = 7.818 kg
whole body mass = 33.341142 kg
nominal COM height = 0.6993168 m
foot front = 0.12 m
foot back = -0.05 m
foot half width = 0.025 m
mu = 1.0
yaw friction = 0.046
f_max = 1400 N
fz_low = 20 * torso_mass / whole_body_mass
```

注意：这里用的是 torso 近似质量和惯量，不是完整 G1 全身动力学。

### 6.2 MPC 默认参数

```text
horizon = 10
mpc_dt = 0.04 s
solve_every = 1
max_iter = 4000
eps_abs / eps_rel = 1e-5 行走，1e-6 站立
```

状态权重：

```text
[8000, 8000, 2000,
 120, 120, 5000,
 1200, 1200, 50,
 40, 40, 1000,
 0]
```

含义上更重视 roll / pitch、COM 高度、角速度和竖直速度稳定。因为这里 torso 惯量较小，角向稳定需要更强权重。

输入正则直接作用在每只脚 6 维 wrench 上。行走 MPC 使用两个 6x6 矩阵：

```text
input_wrench_metric:
  惩罚 wrench effort

nominal_wrench_metric:
  惩罚 wrench 偏离名义竖直支撑力
```

这两个矩阵都写在 wrench 坐标 `[Fx,Fy,Fz,Mx,My,Mz]` 下，不再通过 plane-force 坐标映射。

### 6.3 步态默认参数

```text
t_swing = 0.30 s
t_ds = 0.0 s
step_width = 2 * initial_foot_y
vx = 0.50 m/s
vy = 0.0 m/s
yaw_rate = 0.0 rad/s
velocity_ramp = 2.0 s
stand_time = 2.0 s
prewalk_shift_y = 0.035 m
prewalk_shift_duration = 1.0 s
sim_dt = 0.002 s
duration = 8.0 s
```

启动行走前会做一个横向 COM 预偏移，降低刚进入单支撑时的横滚冲击。

## 7. QP 约束设计

每只脚的约束包含两类：

1. 直接作用在 foot wrench 上的线性接触约束：

```text
Fz >= fz_low
|Fx| <= mu / sqrt(2) * Fz
|Fy| <= mu / sqrt(2) * Fz
|Mx| <= foot_half_width * Fz
foot_back * Fz <= My <= foot_front * Fz
|Mz| <= yaw_friction * Fz
```

2. 每只脚 6 维变量上下界：

```text
接触脚: [-f_max, f_max]
非接触脚: 固定为 0
```

法向力下限 `fz_low` 只在接触时启用。

## 8. 顶部 CONFIG 调参入口

常用调参项：

```python
vx / vy / yaw_rate
t_swing / t_ds
step_width
lateral_step_min / lateral_step_max
mpc_dt / horizon / mpc_solve_every
mass / ixx / iyy / izz
com_x / com_y / com_z
prewalk_shift_y / prewalk_shift_duration
mu / fz_low
roll0 / pitch0 / yaw0
```

诊断输出：

```python
csv=Path("theory_anylisis/out.csv")
plot=True
print_wrench_summary=True
```

可视化：

```python
animate=True
vis_stride=10
realtime=True
```

## 9. 当前框架适合回答的问题

- SRBD-MPC 在 Full-SRB 非线性转动方程下是否还能稳定。
- 接触切换时 wrench 是否连续、是否及时重求解。
- HLIP/LQR 落脚点对 COM 偏移、横滚稳定性的影响。
- MPC horizon、状态权重、步态周期和横向步长限制如何影响可行性。
- 单支撑时足底 roll moment margin 是否足够。

它不适合直接回答：

- 关节级力矩是否可行。
- 腿长、关节限位、碰撞和摆腿轨迹是否合理。
- MuJoCo 接触模型下是否完全一致。
- 全 G1 质量分布和完整浮基动力学下是否稳定。

## 10. 快速结论

这个框架的核心价值是把复杂全身行走问题压缩成“足步规划 + 接触预览 + SRBD-MPC + Full-SRB plant”的最小可研究闭环。参数设计上，它用较强的姿态和竖直方向权重、站立预稳定、行走前横向 COM 预偏移、接触切换强制重求解，来提升简化 SRBD 控制器面对 Full-SRB 非线性 plant 时的稳定裕度。

# CAFE-MPC 框架解析与人形机器人 NMPC 集成方案

本文面向 `Unitree-Dyn-Control` 当前代码库，解释 CAFE-MPC 的整体架构、multi-phase DDP / MS-iLQR 的核心公式，并给出将该框架用于 Unitree G1 / 双足人形机器人 NMPC 的实现路线。

> 参考来源：  
> - CAFE-MPC 论文：Li & Wensing, *Cafe-Mpc: A Cascaded-Fidelity Model Predictive Control Framework with Tuning-Free Whole-Body Control*, arXiv:2403.03995。  
> - CAFE-MPC 开源仓库：`ROAM-Lab-ND/CAFE-MPC`。  
> - 本仓库当前 `main` 分支中的 `CMakeLists.txt`、`algorithm/g1_centroidal_nmpc.*`、`algorithm/g1_kinodynamics_nmpc.*`、`algorithm/gait_scheduler.*`、`algorithm/wbc_priority.*`、`common/data_bus.h`。  
> - 公式按论文记号重新整理；代码落地部分按本仓库已有结构设计。

---

## 1. CAFE-MPC 的核心思想

CAFE-MPC 是 Cascaded-Fidelity MPC：在一个预测时域内，不强行使用同一种模型、同一种时间步长、同一组约束，而是沿 horizon 逐步放松问题：

```text
近端 horizon                  远端 horizon
高保真模型                    低保真模型
小时间步 dt_w                 大时间步 dt_s
完整物理约束                  放松或去掉部分约束
强执行意义                    提供长期价值近似
```

在原论文和代码中，近端使用带硬接触的 whole-body dynamics，远端使用 SRB（single rigid body）模型。对人形机器人来说，最自然的迁移是：

```text
G1 / humanoid CAFE-MPC:
  near-term:  full-body / kinodynamics / hard-contact model
  tail-term:  centroidal / LIP / SRB model
  low-level:  value-based WBC or existing WBC-QP
```

这样做的原因是：高保真 whole-body NMPC 能保证当前若干步的接触、关节、力矩约束；低保真 tail model 以较小计算量提供更长 horizon 的 viability / long-term cost 信息，避免纯短 horizon MPC 只看眼前而造成不可恢复状态。

---

## 2. CAFE-MPC 仓库框架

CAFE-MPC 顶层 CMake 将系统拆成三个主要模块：

```text
CAFE-MPC/
  HSDDPSolver/   # hybrid-system / multi-phase DDP solver
  MHPC/          # CAFE / model-hierarchy MPC 应用
  HKDMPC/        # hybrid kinodynamic MPC 应用
  Reference/     # reference trajectory
  common/
  lcmtypes/
  scripts/
  urdf/
```

### 2.1 HSDDPSolver

`HSDDPSolver` 被构建为 `hsddp` shared library。它承担以下职责：

- 统一处理多相系统的 dynamics、cost、constraints；
- 对 hybrid system 做 backward sweep / forward sweep；
- 在 phase transition 处通过 reset map 传播 value function；
- 使用 Augmented Lagrangian 处理 switching / terminal equality；
- 使用 relaxed barrier 处理 inequality constraints；
- 面向 real-time 实现时，主要采用 MS-iLQR，即省略二阶 dynamics tensor 项。

### 2.2 MHPC / CAFE-MPC 应用层

`MHPC` 是 locomotion MPC 应用入口。它使用 `MHPC-Trajopt` 构造具体机器人问题，将 contact schedule、whole-body model、SRB tail model、cost、constraints 和 LCM 通信组织起来。其执行入口为 `mhpc_run`。

在论文中，这个模块对应：

```text
reference / motion library
        ↓
CAFE-MPC 33~50 Hz
        ↓
MS-iLQR / HS-DDP solver
        ↓
action-value function Q_k
        ↓
VWBC 500 Hz
        ↓
joint torque / position / velocity command
```

### 2.3 HKDMPC

`HKDMPC` 是 hybrid kinodynamic MPC 应用。它与 MHPC 共用多相 DDP 思路，但动态模型和应用目标不同，可看作 CAFE-MPC 中“中等保真模型”的一个实现参考。

---

## 3. CAFE-MPC 控制栈

完整闭环可分为五层：

```text
1. Reference / contact schedule
   - gait scheduler
   - offline trajectory or heuristic reference
   - phase duration, stance/swing foot, target CoM/base/foot pose

2. CAFE-MPC planner
   - near-term whole-body / kinodynamics phase
   - far-term simplified centroidal/SRB phase
   - multi-phase MS-iLQR / DDP
   - 输出 nominal state/control、feedback gain、action-value Q

3. Value-based WBC
   - 以 action-value Q_k 为 QP objective
   - enforce rigid-body dynamics, non-slip, torque limit, friction cone
   - 若去掉约束，解退化为 Riccati feedback

4. Low-level actuator command
   - torque / q / dq command
   - motor limit, safety clamp, watchdog

5. State estimation
   - q, dq, base pose/velocity, contact state, foot pose, measured/estimated GRF
```

你当前库里已经有这些层的雏形：

- `algorithm/gait_scheduler.cpp`：提供双支撑、单支撑、左右支撑、相位时间、触地切换等 contact schedule。
- `common/data_bus.h`：集中保存 `q/dq/ddq`、CoM、足端位姿、接触状态、NMPC 输出、WBC 输入输出。
- `algorithm/g1_centroidal_nmpc.*`：已有 G1 centroidal NMPC，状态为 CoM、线动量、角动量，控制为双脚 6D wrench。
- `algorithm/g1_kinodynamics_nmpc.*`：已有 G1 kinodynamic NMPC，使用 Pinocchio multibody phase space 和 Aligator ProxDDP。
- `algorithm/wbc_priority.*` / `algorithm/wbc_bruce_weighted.*`：已有 WBC-QP 结构，可承接 CAFE-MPC 的 nominal plan 或 action-value objective。
- 顶层 `CMakeLists.txt` 已经把上述 NMPC/WBC 源文件编进 `core`，并链接 `aligator::aligator`、Pinocchio、qpOASES、Mujoco。

因此，移植 CAFE-MPC 不应该从零开始写 MPC，而应该把现有 `g1_centroidal_nmpc` 与 `g1_kinodynamics_nmpc` 组织成 cascaded multi-phase problem。

---

## 4. Multi-phase DDP / MS-iLQR 公式

### 4.1 多相 hybrid system

设一条 horizon 被切成 `n_p` 个 phase。第 `i` 个 phase 有状态 `x_k^{[i]}`、控制 `u_k^{[i]}`、步数 `N_i`、离散动力学 `f_i`。Phase 之间通过 reset map `P_i` 连接：

```math
x_{k+1}^{[i]} = f_i(x_k^{[i]}, u_k^{[i]}),
\qquad k=0,\ldots,N_i-1
```

```math
x_0^{[i+1]} = P_i(x_{N_i}^{[i]}).
```

若 transition 是 state-triggered，例如足端触地，则有 switching / terminal constraint：

```math
g_i(x_{N_i}^{[i]}) = 0.
```

一般约束写作：

```math
h_i(x_k^{[i]}, u_k^{[i]}) \ge 0.
```

总优化问题为：

```math
\min_{\{X^{[i]},U^{[i]}\}_{i=1}^{n_p}}
\sum_{i=1}^{n_p}
\left(
\sum_{k=0}^{N_i-1}
\ell_k^{[i]}(x_k^{[i]},u_k^{[i]})
+
\phi_i(x_{N_i}^{[i]})
\right)
```

subject to dynamics、reset、switching constraints 和 inequalities。

注意：不同 phase 的状态维度、控制维度、动力学、成本和约束都可以不同。这一点是 CAFE-MPC 的关键，因为 whole-body state 和 centroidal/SRB state 通常不是同一维度。

---

### 4.2 Multiple shooting defect

在 multiple shooting 中，`X` 和 `U` 都是决策变量。当前 nominal trajectory 为：

```math
\bar X = \{\bar x_k\}_{k=0}^{N}, \qquad
\bar U = \{\bar u_k\}_{k=0}^{N-1}.
```

动力学 rollout 与 nominal 下一状态之间的误差称为 defect：

```math
\bar d_{k+1}
=
f(\bar x_k,\bar u_k)
-
\bar x_{k+1}.
```

线性化动力学为：

```math
\delta x_{k+1}
=
A_k \delta x_k
+
B_k \delta u_k
+
\bar d_{k+1},
```

其中：

```math
A_k = \frac{\partial f}{\partial x}\bigg|_{\bar x_k,\bar u_k},
\qquad
B_k = \frac{\partial f}{\partial u}\bigg|_{\bar x_k,\bar u_k}.
```

多个 shooting node 可以允许 warm-start 初期不完全可行。DDP 迭代会同时降低 cost 和 defect。

---

### 4.3 Action-value 二阶展开

在每个 knot，DDP 对局部 action-value function 做二阶近似：

```math
Q_k(\delta x_k,\delta u_k)
=
\delta \ell_k(\delta x_k,\delta u_k)
+
V_{k+1}(\delta x_{k+1}).
```

二阶展开为：

```math
Q(\delta x,\delta u)
\approx
\frac{1}{2}
\begin{bmatrix}
\delta x \\
\delta u
\end{bmatrix}^{\top}
\begin{bmatrix}
Q_{xx} & Q_{xu}\\
Q_{ux} & Q_{uu}
\end{bmatrix}
\begin{bmatrix}
\delta x \\
\delta u
\end{bmatrix}
+
Q_x^{\top}\delta x
+
Q_u^{\top}\delta u.
```

在 multiple-shooting DDP 中，先修正 value gradient：

```math
\hat s_{k+1}
=
s_{k+1}
+
S_{k+1}\bar d_{k+1}.
```

设 running cost 的一阶导为 `q_k = \ell_x`、`r_k = \ell_u`，二阶导为 `Q_k^{\ell}=\ell_{xx}`、`R_k=\ell_{uu}`、`P_k=\ell_{ux}`。则：

```math
Q_{x,k}
=
q_k + A_k^\top \hat s_{k+1}
```

```math
Q_{u,k}
=
r_k + B_k^\top \hat s_{k+1}
```

```math
Q_{xx,k}
=
Q_k^\ell + A_k^\top S_{k+1} A_k
+
s_{k+1}\cdot f_{xx,k}
```

```math
Q_{uu,k}
=
R_k + B_k^\top S_{k+1} B_k
+
s_{k+1}\cdot f_{uu,k}
```

```math
Q_{ux,k}
=
P_k + B_k^\top S_{k+1} A_k
+
s_{k+1}\cdot f_{ux,k}.
```

其中 `s·f_xx` 表示 value gradient 与 dynamics 二阶张量的 contraction。实时 MS-iLQR 通常省略这些张量项：

```math
Q_{xx,k}
\approx
Q_k^\ell + A_k^\top S_{k+1} A_k,
```

```math
Q_{uu,k}
\approx
R_k + B_k^\top S_{k+1} B_k,
```

```math
Q_{ux,k}
\approx
P_k + B_k^\top S_{k+1} A_k.
```

这正是 CAFE-MPC 论文中用于 real-time 的选择。

---

### 4.4 局部控制律与 Riccati gain

最小化二阶 `Q` 得到局部控制律：

```math
\delta u_k^*
=
k_k + K_k\delta x_k,
```

其中 feedforward increment 和 feedback gain 为：

```math
k_k = -Q_{uu,k}^{-1} Q_{u,k},
```

```math
K_k = -Q_{uu,k}^{-1} Q_{ux,k}.
```

实际 line search 中使用：

```math
u'_k
=
\bar u_k
+
\alpha k_k
+
K_k(x'_k-\bar x_k),
\qquad \alpha \in (0,1].
```

`Q_uu` 必须正定；若不正定，需要 regularization：

```math
Q_{uu,k}^{reg} = Q_{uu,k} + \lambda I.
```

---

### 4.5 Value function backward recursion

Backward sweep 更新 value function：

```math
S_k
=
Q_{xx,k}
-
Q_{ux,k}^{\top}
Q_{uu,k}^{-1}
Q_{ux,k}
```

```math
s_k
=
Q_{x,k}
-
Q_{ux,k}^{\top}
Q_{uu,k}^{-1}
Q_{u,k}
```

```math
s^0_k
=
s^0_{k+1}
-
\frac{1}{2}
Q_{u,k}^{\top}
Q_{uu,k}^{-1}
Q_{u,k}.
```

边界条件：

```math
S_N=\phi_{xx}(x_N),
\qquad
s_N=\phi_x(x_N),
\qquad
s^0_N=0.
```

这里 `S_k` 是 Hessian，`s_k` 是 gradient，`s^0_k` 是 scalar drift / expected cost change 项。

---

### 4.6 Forward sweep

Forward sweep 的目标是产生 trial trajectory `(X',U')`，并用 merit function 检查 cost 和 defects 是否下降。

对 shooting state，先用线性化 dynamics 更新：

```math
x'_{k+1}
=
\bar x_{k+1}
+
A_k(x'_k-\bar x_k)
+
B_k\delta u_k(\alpha)
+
\bar d_{k+1}.
```

对 rollout state，则真实积分：

```math
x'_{k+1}=f(x'_k,u'_k).
```

其中：

```math
\delta u_k(\alpha)
=
\alpha k_k + K_k(x'_k-\bar x_k).
```

实际工程中，一般从每个 shooting node 并行 rollout 到下一个 shooting node，以提升速度。

---

### 4.7 Reset map 处的 backward recursion

这是 multi-phase DDP 区别于普通 DDP 的关键。

Phase `i` 到 `i+1` 的 reset defect：

```math
\bar d^{[i+1]}_0
=
P_i(\bar x^{[i]}_{N_i})
-
\bar x^{[i+1]}_0.
```

先修正下一 phase 初值处的 value gradient：

```math
\hat s^{[i+1]}_0
=
s^{[i+1]}_0
+
S^{[i+1]}_0
\bar d^{[i+1]}_0.
```

然后把 value function 通过 reset map 回传到上一 phase 的 terminal state：

```math
s^{[i]}_{N_i}
=
\phi_{x,N_i}^{[i]}
+
P_{x,i}^{\top}
\hat s^{[i+1]}_0,
```

```math
S^{[i]}_{N_i}
=
\phi_{xx,N_i}^{[i]}
+
P_{x,i}^{\top}
S^{[i+1]}_0
P_{x,i}
+
s^{[i+1]}_0 \cdot P_{xx,i}.
```

其中：

```math
P_{x,i}
=
\frac{\partial P_i}{\partial x},
\qquad
P_{xx,i}
=
\frac{\partial^2 P_i}{\partial x^2}.
```

实时 MS-iLQR 中通常省略 reset map 的二阶项：

```math
S^{[i]}_{N_i}
\approx
\phi_{xx,N_i}^{[i]}
+
P_{x,i}^{\top}
S^{[i+1]}_0
P_{x,i}.
```

这个式子可以理解为：tail phase 的 long-term value 被投影回 near-term whole-body phase 的 terminal state，成为一个低秩未来成本近似。

---

### 4.8 Switching constraint 与 inequalities

多相受约束问题：

```math
\min \sum_i J^{[i]}(X^{[i]},U^{[i]})
```

subject to：

```math
x_{k+1}^{[i]}=f_i(x_k^{[i]},u_k^{[i]}),
\qquad
x_0^{[i+1]}=P_i(x_{N_i}^{[i]}),
```

```math
g_i(x_{N_i}^{[i]})=0,
\qquad
h_i(x_k^{[i]},u_k^{[i]})\ge0.
```

CAFE-MPC 的处理方式：

1. Switching / terminal equality 用 Augmented Lagrangian：

```math
\tilde \phi_i(x)
=
\phi_i(x)
+
\lambda_i^\top g_i(x)
+
\frac{\rho_i}{2}\|g_i(x)\|^2.
```

2. Inequality 用 Relaxed Barrier，例如：

```math
\tilde \ell_k(x,u)
=
\ell_k(x,u)
+
\sum_j B_{\mu}(h_{i,j}(x,u)).
```

3. 外层更新 `lambda_i, rho_i, mu`；内层用 MS-iLQR 解 smooth unconstrained approximation。

---

## 5. CAFE-MPC 中的机器人模型

### 5.1 Whole-body leading phase

对人形机器人，whole-body state 可写成：

```math
x_w = [q^\top, v^\top]^\top.
```

`q` 是 floating-base generalized configuration，`v` 是 generalized velocity。控制可以是：

```math
u_w = \tau
```

或者 kinodynamic 形式：

```math
u_w = [F_c^\top,\ddot q_j^\top]^\top.
```

硬接触 dynamics：

```math
M(q)\dot v + h(q,v)
=
S^\top \tau + J_c(q)^\top \lambda,
```

接触不滑动约束：

```math
J_c(q)\dot v + \dot J_c(q,v)v = -\alpha J_c(q)v.
```

可整理成 KKT 系统：

```math
\begin{bmatrix}
M(q) & -J_c(q)^\top \\
J_c(q) & 0
\end{bmatrix}
\begin{bmatrix}
\dot v\\
\lambda
\end{bmatrix}
=
\begin{bmatrix}
S^\top\tau-h(q,v)\\
-\dot J_c v-\alpha J_c v
\end{bmatrix}.
```

离散化：

```math
x_{k+1}=f_w(x_k,u_k)
```

可以用 explicit Euler、semi-implicit Euler 或 Lie-group integration。当前库的 `g1_kinodynamics_nmpc` 已经使用 Pinocchio + Aligator 的 `MultibodyPhaseSpace` 和 semi-implicit Euler 风格的 kinodynamic model，可作为 CAFE-MPC 近端 phase 的基础。

#### Impact reset

触地时速度发生冲量跳变：

```math
\begin{bmatrix}
M(q) & -J_c(q)^\top\\
J_c(q) & 0
\end{bmatrix}
\begin{bmatrix}
v^+\\
\Lambda
\end{bmatrix}
=
\begin{bmatrix}
M(q)v^-\\
0
\end{bmatrix}.
```

Reset map：

```math
x^+ = P_{\mathrm{impact}}(x^-)
=
[q^{-\top}, v^{+\top}]^\top.
```

起飞时通常可用 identity reset：

```math
P_{\mathrm{takeoff}}(x)=x.
```

### 5.2 Whole-body constraints

人形机器人 near-term phase 建议至少包含：

```math
\tau_{\min} \le \tau \le \tau_{\max}
```

```math
q_{\min} \le q \le q_{\max}
```

```math
v_{\min} \le v \le v_{\max}
```

```math
\lambda_z \ge 0
```

```math
|\lambda_x| \le \mu \lambda_z,
\qquad
|\lambda_y| \le \mu \lambda_z
```

对平面足，还应加入 CoP / wrench cone：

```math
|m_x| \le y_{\max} f_z,
\qquad
|m_y| \le x_{\max} f_z,
\qquad
|m_z| \le \mu_z f_z.
```

以及：

- swing foot clearance；
- touchdown foot height equality；
- stance foot no-slip；
- pelvis / torso attitude bound；
- knee/ankle joint soft-limit；
- self-collision margin；
- 对 G1 这类双足机器人，建议保留 ZMP / CoP 支持域约束，不要像原 CAFE-MPC quadruped tail 那样一开始完全去掉所有 tail constraints。

### 5.3 Tail phase：centroidal / SRB / LIP

原 CAFE-MPC 用 SRB 作为 tail：

```math
x_s=[\theta^\top,c^\top,\omega^\top,\dot c^\top]^\top,
\qquad
u_s=[f_1^\top,\ldots,f_m^\top]^\top.
```

对人形机器人，更建议以 centroidal model 作为第一版 tail，因为你的库已经有 `G1CentroidalNmpc`。状态可定义为：

```math
x_c =
[c^\top, l^\top, h^\top]^\top
```

其中：

- `c`：CoM position；
- `l=m\dot c`：linear momentum；
- `h`：angular momentum。

动力学：

```math
\dot c = \frac{1}{m}l,
```

```math
\dot l = m g + \sum_j f_j,
```

```math
\dot h = \sum_j (p_j-c)\times f_j + \tau_j.
```

当前 `g1_centroidal_nmpc.cpp` 已经接近该形式：`kNx=9`，包含 CoM、linear momentum、angular momentum；每只脚控制 6D wrench；contact table 和 contact positions 可以沿 horizon 给定。

---

## 6. Whole-body 与 tail 的连接

设 whole-body state：

```math
x_w=[q^\top,v^\top]^\top
```

tail centroidal state：

```math
x_c=[c^\top,l^\top,h^\top]^\top.
```

定义 projection：

```math
\Psi(x_w)
=
\begin{bmatrix}
c(q)\\
mJ_{\mathrm{com}}(q)v\\
A_g(q)v
\end{bmatrix}.
```

其中：

- `c(q)` 来自 Pinocchio center of mass；
- `J_com(q)v` 是 CoM velocity；
- `A_g(q)v` 是 centroidal momentum。

Model transition constraint：

```math
x_{c,0}^{+}
=
\Psi(P_{\mathrm{impact}}(x_{w,N_w}^{-})).
```

也可以写为 reset map：

```math
P_{w\rightarrow c}(x_w)
=
\Psi(P_{\mathrm{impact}}(x_w)).
```

在 multi-phase DDP 中，这个 `P_{w→c}` 的 Jacobian 会把 tail value Hessian 投影回 whole-body terminal：

```math
S_{w,N_w}
\leftarrow
P_{x,w\to c}^{\top}
S_{c,0}
P_{x,w\to c}.
```

工程上，如果暂时不实现完整 multi-phase variable-dimension DDP，可以先使用近似：

```math
\phi_w(x_{w,N_w})
=
\|\Psi(x_{w,N_w}) - x_{c,0}^{ref}\|_{W_c}^2,
```

或将 tail solver 计算出的 nominal centroidal trajectory 作为 terminal / tracking cost 注入 whole-body NMPC。

---

## 7. 如何写进当前 Unitree-Dyn-Control 库

### 7.1 当前库现状

当前库已具备 CAFE-MPC 的大部分基础组件：

```text
algorithm/
  g1_centroidal_nmpc.cpp       # centroidal NMPC，可作为 tail model
  g1_kinodynamics_nmpc.cpp     # multibody kinodynamics NMPC，可作为 near-term model
  g1_srbd_mpc*.cpp             # SRB / reduced-order MPC 参考
  gait_scheduler.cpp           # 接触相位和步态时序
  pino_kin_dyn.cpp             # Pinocchio kinematics/dynamics
  wbc_priority.cpp             # WBC-QP
  wbc_bruce_weighted.cpp       # weighted WBC variant
common/
  data_bus.h                   # 状态、命令、MPC/WBC 数据中枢
demo/
  g1_cd_nmpc_wbc_walk.cpp
  g1_kino_nmpc_id_stand.cpp
  ...
```

顶层 `CMakeLists.txt` 已经包含：

- C++17；
- Pinocchio；
- Eigen；
- Aligator；
- qpOASES；
- Mujoco；
- `core` library；
- G1 centroidal / kinodynamic NMPC demo。

因此推荐分三阶段集成。

---

## 8. 集成路线 A：最小可运行 CAFE-like NMPC

目标：不改 solver 内核，先把现有 centroidal tail 信息注入 kinodynamic NMPC。

### 8.1 新增文件

建议新增：

```text
algorithm/g1_cafe_mpc.h
algorithm/g1_cafe_mpc.cpp
```

类接口：

```cpp
class G1CafeMpc {
public:
  struct Options {
    int wbHorizon = 10;
    double wbDt = 0.02;
    int tailHorizon = 20;
    double tailDt = 0.05;
    double tailTerminalWeight = 1.0;
    bool useTailValueTerminalCost = true;
  };

  explicit G1CafeMpc(const pinocchio::Model& model,
                     const std::array<pinocchio::FrameIndex, 2>& feet,
                     const Options& options);

  bool solveFromDataBus(const DataBus& robotState, double mass);
  void writeToDataBus(DataBus& robotState) const;

private:
  G1KinodynamicsNmpc wbNmpc_;
  G1CentroidalNmpc tailNmpc_;
};
```

### 8.2 计算流程

```text
1. 从 DataBus 读取当前 q, v, CoM, momentum, foot pose, contact state。
2. 用 gait_scheduler 生成未来 contact table：
   - DSt: 双支撑
   - LSt: 左脚支撑
   - RSt: 右脚支撑
   - touchdown / takeoff 时刻
3. 先解 centroidal tail NMPC：
   - state = [CoM, linear momentum, angular momentum]
   - control = left/right foot wrench
   - horizon = tailHorizon, dt = tailDt
4. 将 tail 的第 0 个状态或 terminal value 变成 kinodynamic terminal cost：
   - cost on CoM
   - cost on linear momentum
   - cost on angular momentum
5. 解 near-term kinodynamic NMPC：
   - horizon = wbHorizon, dt = wbDt
   - contactActive 来自 gait schedule
   - foot pose reference 来自 foot placement / swing trajectory
6. 输出：
   - first whole-body state/control
   - first contact wrench
   - optional joint acceleration
7. WBC 使用 near-term result 生成 torque。
```

这是 CAFE-MPC 的工程近似版本，不需要马上实现 variable-dimension multi-phase DDP。它的本质是：

```math
\min_{X_w,U_w}
J_w(X_w,U_w)
+
\phi_{\mathrm{tail}}(\Psi(x_{w,N_w})).
```

其中 `\phi_tail` 来自 tail centroidal NMPC 的长期计划。

### 8.3 对现有代码的改动点

1. 顶层 `CMakeLists.txt`：

```cmake
set(CPP_SOURCES
  ...
  algorithm/g1_cafe_mpc.cpp
)
```

2. `common/data_bus.h` 增加 CAFE-MPC 输出缓存：

```cpp
bool cafe_mpc_enabled{false};
int cafe_mpc_status{0};
double cafe_mpc_cpuTime{0.0};
Eigen::VectorXd cafe_mpc_q_ref;
Eigen::VectorXd cafe_mpc_v_ref;
Eigen::VectorXd cafe_mpc_tau_ff;
Eigen::VectorXd cafe_mpc_wrench_ff;
Eigen::MatrixXd cafe_mpc_K;      // optional Riccati gain
Eigen::MatrixXd cafe_mpc_Qxx;    // optional action-value Hessian
Eigen::VectorXd cafe_mpc_Qu;     // optional action-value gradient
```

3. demo 中增加：

```text
demo/g1_cafe_nmpc_wbc_walk.cpp
```

4. WBC 中先复用 feedforward：

```text
cafe_mpc_wrench_ff -> Fr_ff
cafe_mpc_q_ref     -> des_q
cafe_mpc_v_ref     -> des_dq
```

---

## 9. 集成路线 B：真正的 multi-phase CAFE-MPC

目标：实现接近 CAFE-MPC 论文的 multi-phase DDP，而不是只把 tail 变成 terminal cost。

### 9.1 建议新增目录

```text
algorithm/cafe_mpc/
  core/
    phase.h
    multi_phase_problem.h
    multi_phase_ddp_solver.h
    shooting_trajectory.h
    value_expansion.h
  models/
    g1_whole_body_phase.h
    g1_centroidal_tail_phase.h
    reset_maps.h
  costs/
    tracking_costs.h
    humanoid_cost_builder.h
  constraints/
    contact_constraints.h
    joint_limit_constraints.h
    touchdown_constraints.h
  adapters/
    data_bus_adapter.h
    aligator_adapter.h
```

### 9.2 Phase 抽象

```cpp
struct PhaseModel {
  int nx;
  int nu;
  int N;
  double dt;

  virtual void dynamics(const Eigen::VectorXd& x,
                        const Eigen::VectorXd& u,
                        Eigen::VectorXd& xnext) const = 0;

  virtual void derivatives(const Eigen::VectorXd& x,
                           const Eigen::VectorXd& u,
                           Eigen::MatrixXd& A,
                           Eigen::MatrixXd& B) const = 0;

  virtual void cost(const Eigen::VectorXd& x,
                    const Eigen::VectorXd& u,
                    CostExpansion& out) const = 0;

  virtual void constraints(const Eigen::VectorXd& x,
                           const Eigen::VectorXd& u,
                           ConstraintValues& out) const = 0;
};
```

### 9.3 Reset map 抽象

```cpp
struct ResetMap {
  int nx_before;
  int nx_after;

  virtual void eval(const Eigen::VectorXd& x_before,
                    Eigen::VectorXd& x_after) const = 0;

  virtual void jacobian(const Eigen::VectorXd& x_before,
                        Eigen::MatrixXd& Px) const = 0;
};
```

对 humanoid：

```cpp
class WholeBodyToCentroidalReset : public ResetMap {
  // x_c = [com(q), m Jcom(q) v, Ag(q) v]
};
```

触地 impact：

```cpp
class ImpactReset : public ResetMap {
  // solve KKT impact equation and return [q, v_plus]
};
```

起飞：

```cpp
class IdentityReset : public ResetMap {};
```

### 9.4 Solver 数据结构

```cpp
struct KnotData {
  Eigen::VectorXd x, u;
  Eigen::VectorXd defect;
  Eigen::MatrixXd A, B;
  CostExpansion cost;
  ValueExpansion value;
  ValueExpansion actionValue;
  Eigen::VectorXd kff;
  Eigen::MatrixXd K;
};

struct PhaseTrajectory {
  std::shared_ptr<PhaseModel> model;
  std::vector<KnotData> knots;
  Eigen::VectorXd resetDefectToNext;
};

struct MultiPhaseTrajectory {
  std::vector<PhaseTrajectory> phases;
};
```

### 9.5 Backward sweep 伪代码

```cpp
for phase i = last_phase downto first_phase:
  if i is last_phase:
    initialize terminal value from terminal cost
  else:
    d0 = P_i(x_N_i) - x_0_{i+1}
    shat = s_next0 + S_next0 * d0
    s_N = phi_x + Px.transpose() * shat
    S_N = phi_xx + Px.transpose() * S_next0 * Px
    // DDP exact version add tensor term: s_next0 dot Pxx

  for k = N_i-1 downto 0:
    shat = s_{k+1} + S_{k+1} * defect_{k+1}
    Qx  = lx + A.transpose() * shat
    Qu  = lu + B.transpose() * shat
    Qxx = lxx + A.transpose() * S_{k+1} * A
    Quu = luu + B.transpose() * S_{k+1} * B
    Qux = lux + B.transpose() * S_{k+1} * A

    regularize Quu
    kff = -Quu.ldlt().solve(Qu)
    K   = -Quu.ldlt().solve(Qux)

    s_k = Qx - Qux.transpose() * Quu.inverse() * Qu
    S_k = Qxx - Qux.transpose() * Quu.inverse() * Qux
```

### 9.6 Forward sweep 伪代码

```cpp
for alpha in {1, 0.5, 0.25, ...}:
  x_trial[0] = measured_x0

  for each phase i:
    if i > 0:
      x_trial_i[0] = P_{i-1}(x_trial_{i-1}[N_{i-1}])

    for k in 0..N_i-1:
      dx = x_trial_i[k] - x_nom_i[k]
      u_trial_i[k] = u_nom_i[k] + alpha * kff_i[k] + K_i[k] * dx
      x_trial_i[k+1] = f_i(x_trial_i[k], u_trial_i[k])

  evaluate cost, defects, AL merit, barriers
  accept if merit decreases
```

---

## 10. 人形机器人 G1 的具体 NMPC 设计

### 10.1 Contact phases

G1 双足 walking 的 phase 建议：

```text
DSt      double support
LSt      left support, right swing
DSt      double support
RSt      right support, left swing
DSt      double support
```

`gait_scheduler.cpp` 已经在 `DataBus` 中写入：

```cpp
walk_is_double_support
walk_left_contact
walk_right_contact
walk_phase_time
walk_time_to_impact
walk_stance_leg
```

CAFE-MPC 应直接使用这些变量生成 horizon contact table：

```cpp
Eigen::Matrix<int, Eigen::Dynamic, 2> contactTable;
```

每个 phase 的开始和结束由 contact pattern 变化决定：

```text
[1,1] -> [1,0] -> [1,1] -> [0,1] -> [1,1]
```

### 10.2 Near-term whole-body / kinodynamic phase

第一版推荐使用现有 `G1KinodynamicsNmpc`：

```text
state:
  x = [q, v]

control:
  u = [left_foot_wrench, right_foot_wrench, joint_acceleration]
```

cost：

```math
\ell_w =
\|x-x^{ref}\|_{W_x}^2
+
\|u-u^{ref}\|_{W_u}^2
+
\sum_{foot}
\|p_{foot}(q)-p_{foot}^{ref}\|_{W_p}^2.
```

constraints：

```text
- force / wrench cone
- contact foot placement equality
- joint acceleration bounds
- torque bounds, if inverse dynamics is explicitly included
```

第二版再升级为真正 torque-control whole-body hard-contact dynamics：

```text
state:
  x = [q, v]

control:
  u = tau

algebraic:
  lambda from KKT contact solve
```

### 10.3 Tail centroidal phase

直接复用 `G1CentroidalNmpc`：

```text
state:
  x = [CoM position, linear momentum, angular momentum]
  nx = 9

control:
  u = [left foot 6D wrench, right foot 6D wrench]
  nu = 12
```

cost：

```math
\ell_c =
\|c-c^{ref}\|_{W_c}^2
+
\|l-l^{ref}\|_{W_l}^2
+
\|h-h^{ref}\|_{W_h}^2
+
\|u-u^{nom}\|_{R}^2.
```

constraints：

```text
- wrench cone / CoP cone
- min/max normal force
- force box
```

与原 CAFE-MPC 不同，人形机器人 tail 不建议一开始完全 unconstrained。原因是双足机器人支撑域小，tail 若完全无约束，可能给 near-term 提供不物理的 long-term value。

### 10.4 Whole-body to centroidal projection

在 `pino_kin_dyn` 或新 adapter 中实现：

```cpp
Eigen::Matrix<double,9,1> projectWholeBodyToCentroidal(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    double mass)
{
  Eigen::Matrix<double,9,1> xc;
  xc.segment<3>(0) = com(q);
  xc.segment<3>(3) = mass * Jcom(q) * v;
  xc.segment<3>(6) = Ag(q) * v; // angular momentum part
  return xc;
}
```

`DataBus` 已有：

```cpp
pCoM_W
Jcom_W
dyn_Ag
q
dq
```

所以第一版可以直接从 `DataBus` 拼出 centroidal state，而不必每次重新调用 Pinocchio。

---

## 11. VWBC：从 Riccati feedback 到 value-based QP

### 11.1 普通 Riccati feedback

DDP 自然输出：

```math
u = u^* + K(x-x^*).
```

这能提高 MPC between-tick 的反馈能力，但不能保证接触力、力矩、摩擦锥约束始终满足。

### 11.2 Value-based WBC

CAFE-MPC 的 VWBC 使用 action-value function 作为 QP objective：

```math
\min_{\tau,\ddot q,\lambda}
Q_k(x-x_k^*, \tau-\tau_k^*)
```

subject to：

```math
M(q)\ddot q+h(q,\dot q)
=
S^\top\tau+J_c^\top\lambda
```

```math
J_c\ddot q+\dot J_c\dot q
=
-\alpha J_c\dot q
```

```math
\tau_{\min}\le\tau\le\tau_{\max}
```

```math
\lambda_z\ge0,
\qquad
|\lambda_x|\le\mu\lambda_z,
\qquad
|\lambda_y|\le\mu\lambda_z.
```

如果不加约束，该 QP 的解等价于 Riccati feedback；加上约束后，WBC 会选择最接近 DDP value-optimal 的可行 torque。

### 11.3 当前库中的落地方式

当前 `wbc_priority.cpp` 已经有：

- floating-base selector；
- contact Jacobian；
- plane foot wrench cone；
- qpOASES QP；
- `eigen_ddq_Opt`、`eigen_fr_Opt`、`eigen_tau_Opt` 输出。

短期路线：

```text
CAFE-MPC 输出 nominal q/v/wrench/joint acceleration
        ↓
现有 WBC 继续按 task priority 或 weighted QP 求解
```

中期路线：

```text
CAFE-MPC 输出 Qxx, Quu, Qux, Qx, Qu
        ↓
WBC QP objective 替换为 action-value objective
```

需要新增：

```cpp
struct ActionValueApprox {
  Eigen::MatrixXd Qxx;
  Eigen::MatrixXd Quu;
  Eigen::MatrixXd Qxu;
  Eigen::VectorXd Qx;
  Eigen::VectorXd Qu;
  Eigen::VectorXd xNom;
  Eigen::VectorXd uNom;
};
```

并写入 `DataBus`，由 WBC 读取。

---

## 12. 推荐实现顺序

### Phase 1：文档与接口准备

- 新增本文件。
- 新增 `algorithm/g1_cafe_mpc.h/.cpp` 的空壳。
- 在 `DataBus` 增加 CAFE-MPC 输出字段。
- 在 CMake 中加入源文件。
- 新增 `demo/g1_cafe_nmpc_wbc_walk.cpp`，先只打印状态。

### Phase 2：CAFE-like terminal-value 版本

- 用 `G1CentroidalNmpc` 解 tail。
- 把 tail 的 CoM / momentum reference 注入 `G1KinodynamicsNmpc` 的 terminal / running cost。
- 不做 variable-dimension reset。
- WBC 使用现有接口跟踪 near-term kinodynamic 输出。
- 目标：Mujoco 中稳定走路，频率 20~50 Hz。

### Phase 3：phase-aware horizon

- 从 `gait_scheduler` 生成 contact table。
- 每当 contact pattern 变化，创建新 phase。
- 每个 phase 使用不同 constraints：
  - double support；
  - left support；
  - right support；
  - touchdown；
  - takeoff。
- Warm start 做 phase-aware shifting，而不是简单整体左移。

### Phase 4：完整 multi-phase DDP

- 新增 `algorithm/cafe_mpc/core/multi_phase_ddp_solver.*`。
- 支持不同 phase state/control dimension。
- 实现 reset defect 和 value projection。
- 对 touchdown 实现 impact reset。
- 对 whole-body → centroidal 实现 projection reset。
- 支持 AL / ReB 外层循环。

### Phase 5：Value-based WBC

- 从 solver 导出 action-value expansion。
- 将 WBC objective 从 task-weighted LS 改为 action-value QP，或混合：
  
```math
\min
Q_{\mathrm{CAFE}}(\delta x,\delta u)
+
\epsilon \|r_{\mathrm{task}}\|^2.
```

- 保留动力学、接触、力矩、摩擦锥约束。

---

## 13. 关键工程风险与处理

### 13.1 变量维度切换

完整 CAFE-MPC 允许不同 phase 有不同 `nx/nu`。现有 Aligator problem 很可能更适合同一 state space 的 stage sequence。解决路线：

1. 短期：不要切换维度，用 tail value terminal cost 近似；
2. 中期：把 centroidal tail 嵌入 full-state cost；
3. 长期：自研 multi-phase DDP core。

### 13.2 接触时刻固定

CAFE-MPC 假设 phase sequence 和 timing 固定。G1 实机/仿真中 touchdown 时间会有误差。处理方式：

- gait scheduler 给 nominal timing；
- state estimator 检测 early/late contact；
- MPC 每 tick 重新构造 phase list；
- 若 contact 提前，则立刻切换相位并 warm-start；
- 若 contact 延迟，则延长 swing phase 或触发 recovery。

### 13.3 Quaternion / Lie group

论文使用 Euler angle 表达 SRB orientation。人形机器人最好不要在 full-body NMPC 中直接对 floating-base quaternion 做简单欧式差。建议：

- full-body state 用 Pinocchio `MultibodyPhaseSpace`；
- state difference 用 Lie group `difference/integrate`；
- centroidal tail 可使用 Euler 或 yaw-separated representation，但 roll/pitch 大角度动作时需谨慎。

### 13.4 Tail constraints

Quadruped acrobatic CAFE-MPC 远端 tail 可去掉 constraints；G1 walking 不建议一开始这么做。建议先保留：

- normal force lower/upper；
- friction pyramid；
- foot wrench cone；
- support polygon / CoP；
- CoM height band。

稳定后再逐步 relax，用 solve time 与 tracking error 做 ablation。

### 13.5 Solver 输出 action-value

如果继续使用 Aligator ProxDDP，需要确认是否能导出每个 knot 的 `Qxx/Quu/Qux/Qx/Qu/K`。若不能：

- 初期只使用 nominal plan；
- 中期从 solver data 里取 Riccati gains；
- 长期在自研 multi-phase DDP 中保存 action-value expansion。

---

## 14. 建议参数初值

| 项目 | 初值 |
|---|---:|
| near-term whole-body horizon | 8~15 knots |
| near-term dt | 0.01~0.02 s |
| tail centroidal horizon | 15~30 knots |
| tail dt | 0.04~0.08 s |
| MPC update | 20~50 Hz |
| WBC update | 500 Hz 左右 |
| MS-iLQR max inner iterations | 3~10 real-time / 20+ debug |
| AL outer iterations | 1~3 |
| Q_uu regularization | `1e-8` 起，失败则 ×10 |
| line-search alpha | `1, 0.5, 0.25, 0.1, 0.05` |

---

## 15. 最小闭环伪代码

```cpp
while (running) {
  // 1. state estimation
  DataBus robotState = readState();

  // 2. update gait schedule
  gaitScheduler.dataBusRead(robotState);
  gaitScheduler.step();
  gaitScheduler.dataBusWrite(robotState);

  // 3. build CAFE-MPC problem
  G1CafeMpc::Input input;
  input.q = robotState.q;
  input.v = robotState.dq;
  input.contactTable = buildContactTable(robotState);
  input.footRefs = buildFootReferences(robotState);
  input.centroidalRefs = buildCentroidalReference(robotState);

  // 4. solve cascaded NMPC
  bool ok = cafeMpc.solve(input);

  // 5. write nominal plan / value approximation
  cafeMpc.writeToDataBus(robotState);

  // 6. WBC-QP
  wbc.dataBusRead(robotState);
  wbc.computeDdq();
  wbc.computeTau();
  wbc.dataBusWrite(robotState);

  // 7. low-level command
  sendMotorCommand(robotState.motors_tor_out,
                   robotState.motors_pos_des,
                   robotState.motors_vel_des);
}
```

---

## 16. 最推荐的第一版 PR 内容

如果要让当前库快速获得 CAFE-MPC 能力，建议第一版只做这些：

1. `algorithm/g1_cafe_mpc.h/.cpp`
   - 包含 `G1CentroidalNmpc tailNmpc_`；
   - 包含 `G1KinodynamicsNmpc wbNmpc_`；
   - `solveFromDataBus()` 先 tail 后 whole-body；
   - 通过 projection 把 tail reference 注入 whole-body terminal tracking。

2. `common/data_bus.h`
   - 增加 CAFE-MPC 输出；
   - 不破坏已有 `centroidal_nmpc_*` 字段。

3. `demo/g1_cafe_nmpc_wbc_walk.cpp`
   - 复制 `g1_cd_nmpc_wbc_walk` 的主循环；
   - 把 controller 替换成 `G1CafeMpc`；
   - WBC 仍使用现有 `wbc_priority`。

4. 暂不引入 CAFE-MPC 原仓库 `HSDDPSolver`。
   - 原 solver 与本库 Aligator/Pinocchio/qpOASES 体系重叠；
   - 先复用本库已有 NMPC 更稳；
   - 等 terminal-value 版本稳定后，再补自研 multi-phase DDP。

---

## 17. 验证路线

### 17.1 单元测试

- `projectWholeBodyToCentroidal()` 数值正确；
- contact table 与 `gait_scheduler` 相位一致；
- force cone residual 正负号正确；
- warm start shift 后首状态等于当前状态；
- tail terminal cost 对 CoM / momentum 的梯度方向正确。

### 17.2 Mujoco 仿真

依次测试：

1. 双支撑站立：tail + WB 都不应产生横向漂移；
2. 原地小步走：确认 phase 切换；
3. 低速前进：检查 CoP / wrench cone；
4. 中速前进：检查 solve time；
5. 推扰恢复：比较无 tail / 有 tail 的恢复；
6. 随机初始 yaw / lateral velocity：检查 horizon viability。

### 17.3 指标

```text
- MPC solve time mean / p95
- WBC QP solve time
- CoM tracking RMS
- base roll/pitch RMS
- foot slip distance
- friction cone violation
- torque saturation ratio
- touchdown timing error
- fall / recovery statistics
```

---

## 18. 结论

CAFE-MPC 对当前 `Unitree-Dyn-Control` 的最佳落地方式不是直接整体拷贝原仓库，而是把思想嵌入现有架构：

```text
gait_scheduler
   ↓
G1CentroidalNmpc as tail value planner
   ↓
G1KinodynamicsNmpc as near-term whole-body planner
   ↓
WBC priority / weighted QP
   ↓
Mujoco / hardware low-level command
```

第一阶段应实现 CAFE-like terminal-value NMPC；第二阶段再做真正的 multi-phase DDP 和 reset-aware backward recursion；第三阶段将 action-value function 接入 WBC-QP，形成完整的 CAFE-MPC + VWBC。

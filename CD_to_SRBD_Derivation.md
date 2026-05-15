# 从 CppAD Centroidal Dynamics 到 OpenLoong SRBD-MPC 的推导

本文记录如何从

- `/home/songchao/OPTControl_env/wb_humanoid_mpc_ws/cppad_code_gen`
- `/home/songchao/OPTControl_env/wb_humanoid_mpc_ws/cppad_code_gen/cppad_centroidal_mpc_g1`

中的 CppAD 质心动力学模型（Centroidal Dynamics, CD）推导到当前仓库
`algorithm/mpc.cpp` 使用的单刚体动力学（Single Rigid Body Dynamics, SRBD）MPC 形式。

重点不是重新解释 MPC，而是把两套代码里的变量、假设和公式一层层对齐。

## 1. 两套模型的核心差异

`cppad_centroidal_mpc_g1` 是 OCS2 风格的全身质心模型。配置文件
`/home/songchao/OPTControl_env/wb_humanoid_mpc_ws/config/g1_centroidal/mpc/task.info`
里有：

```text
centroidalModelType 0 // 0: FullCentroidalDynamics, 1: Single Rigid Body Dynamics
```

也就是说当前生成出来的 `dynamics_systemFlowMap` 是 Full Centroidal Dynamics。

OpenLoong 当前的 `algorithm/mpc.cpp` 是一个线性 SRBD-QP：

```text
状态:   x = [rpy, p_base, omega, v_base]
输入:   u = [left foot wrench, right foot wrench, gravity pseudo input]
求解器: qpOASES
形式:   线性离散模型 + 二次目标 + 线性接触约束
```

两者关系可以粗略写成：

```text
Full CD
  保留: 质心动量、base pose、所有 MPC 关节角、关节速度输入、构型相关 CoM/惯量/接触位置
  动力学: 非线性，CppAD 自动微分

SRBD
  保留: base/CoM 姿态、位置、角速度、线速度、左右脚接触 wrench
  删除: 关节角状态、关节速度输入、内部形变量对质心惯量和动量的影响
  动力学: 每个 MPC 周期在当前状态冻结/线性化，得到线性 QP
```

## 2. CppAD CD 的状态和输入

`PinocchioCentroidalDynamicsAD.h` 对模型的描述是：

```text
State:
x = [
  linear_momentum / mass,
  angular_momentum / mass,
  base_position,
  base_orientation_zyx,
  joint_positions
]'

Input:
u = [
  contact_forces,
  contact_wrenches,
  joint_velocities
]'
```

G1 的 `task.info` 里状态维度是 35，输入维度也是 35。CppAD 生成的
`dynamics_systemFlowMap_info.c` 里也能看到：

```c
*m = 35;
*n = 70;
```

这里 `m=35` 是输出维度，也就是 \(\dot{x}\) 维度。`n=70` 是 CppAD 独立变量维度，通常是把 state 和 input 拼在一起：

\[
z =
\begin{bmatrix}
x\\
u
\end{bmatrix}
\in \mathbb{R}^{70}
\]

对应索引可以由 `task.info` 和生成代码反推：

```text
state x, 35维:
  x[0:3]    h_lin / m
  x[3:6]    h_ang / m
  x[6:9]    p_base
  x[9:12]   theta_base_zyx = [yaw, pitch, roll]
  x[12:35]  q_joints

input u, 35维:
  z[35:41]  left foot wrench  [fx, fy, fz, mx, my, mz]
  z[41:47]  right foot wrench [fx, fy, fz, mx, my, mz]
  z[47:70]  joint velocities
```

注意 CppAD 的 `z[35]` 才是输入的第 0 维，因为 `z[0:35]` 已经是状态。

从生成代码的开头可以看到线动量变化：

```c
y[0] = (x[35] + x[41]) / 35.11514202;
y[1] = (x[36] + x[42]) / 35.11514202;
y[2] = (-344.4795432162 + x[37] + x[43]) / 35.11514202;
```

其中：

\[
35.11514202 = m
\]

\[
344.4795432162 \approx m \cdot 9.81
\]

所以：

\[
\dot{h}_{lin}/m =
\frac{f_L + f_R + mg}{m}
\]

因为重力沿世界系 \(z\) 轴负方向：

\[
g_W =
\begin{bmatrix}
0\\0\\-9.81
\end{bmatrix}
\]

所以前三维是：

\[
\frac{d}{dt}
\left(\frac{p_{lin}}{m}\right)
=
\frac{f_L+f_R}{m}
 + g_W
\]

这里 \(p_{lin}\) 是总线动量，\(p_{lin}/m\) 就是 CoM 线速度。

生成代码后面：

```c
y[12] = x[47];
...
y[34] = x[69];
```

说明关节角状态的导数就是输入里的关节速度：

\[
\dot{q}_j = u_{qdot}
\]

## 3. Full CD 的连续动力学

全身质心动力学最核心的物理方程是：

\[
\dot{h}
=
\sum_i
\begin{bmatrix}
f_i\\
(p_i - c) \times f_i + \tau_i
\end{bmatrix}
+
\begin{bmatrix}
m g\\
0
\end{bmatrix}
\]

其中：

```text
h      = [p_lin, L]^T，全身质心动量
p_lin  = m * c_dot
L      = 关于 CoM 的全身角动量
c      = CoM 位置
p_i    = 第 i 个接触点位置
f_i    = 第 i 个接触力，世界系表达
tau_i  = 第 i 个接触力矩，世界系表达
```

OCS2 代码中使用的是 mass-normalized momentum：

\[
\bar{h} = h/m
=
\begin{bmatrix}
p_{lin}/m\\
L/m
\end{bmatrix}
=
\begin{bmatrix}
\dot{c}\\
L/m
\end{bmatrix}
\]

因此：

\[
\dot{\bar{h}}
=
\frac{1}{m}
\sum_i
\begin{bmatrix}
f_i\\
(p_i-c)\times f_i+\tau_i
\end{bmatrix}
+
\begin{bmatrix}
g\\
0
\end{bmatrix}
\]

这正是 `src/compat/CentroidalModelLocal.cpp` 中
`getNormalizedCentroidalMomentumRateLocal()` 的形式：

```cpp
centroidalMomentumRate << info.robotMass * gravityVector, Zero3;

for each contact:
  centroidalMomentumRate.head<3>() += contactForce;
  centroidalMomentumRate.tail<3>() += r_com_to_contact.cross(contactForce) + contactTorque;

centroidalMomentumRate /= info.robotMass;
```

写成左右脚 6D contact wrench：

\[
w_L =
\begin{bmatrix}
f_L\\
\tau_L
\end{bmatrix},
\quad
w_R =
\begin{bmatrix}
f_R\\
\tau_R
\end{bmatrix}
\]

则：

\[
\dot{\bar{h}}_{lin}
=
\frac{f_L+f_R}{m}+g
\]

\[
\dot{\bar{h}}_{ang}
=
\frac{
(p_L-c)\times f_L+\tau_L
+(p_R-c)\times f_R+\tau_R
}{m}
\]

这是 CD 和 SRBD 共用的外力/外力矩平衡核心。

## 4. Full CD 里为什么还需要关节状态

Full CD 的状态不是只放 \(\bar{h}\)。它还放了 base pose 和所有 MPC 关节角：

\[
x_{CD}
=
\begin{bmatrix}
\bar{h}_{lin}\\
\bar{h}_{ang}\\
p_b\\
\theta_b\\
q_j
\end{bmatrix}
\]

原因是：

1. CoM \(c(q)\) 不是固定等于 base position \(p_b\)，它随全身关节姿态变化。
2. 接触点 \(p_i(q)\) 由 Pinocchio 正运动学得到，也随关节变化。
3. 质心动量矩阵 \(A_g(q)\) 随构型变化。
4. 全身角动量 \(L\) 不只来自 base 刚体转动，还包含腿、腰、手臂摆动产生的内部角动量。

Pinocchio 的质心动量矩阵满足：

\[
h = A_g(q) v
\]

其中：

```text
q = [base pose, joint positions]
v = [base velocity, joint velocities]
```

在 OCS2 的 flow map 里，给定状态里的 \(\bar{h}\) 和输入里的 \(\dot{q}_j\)，需要反过来求出 base velocity：

\[
v_b =
A_b(q)^{-1}
\left(
h - A_j(q)\dot{q}_j
\right)
\]

然后才得到：

\[
\dot{p}_b = v_{b,lin}
\]

\[
\dot{\theta}_b = T(\theta_b)^{-1}\omega_b
\]

这就是生成代码中 `y[6]` 到 `y[11]` 很复杂的原因：它不是简单的 \(\dot{p}=v\)，而是通过全身 \(A_g(q)\)、关节速度和欧拉角映射算出来的 base pose 导数。

## 5. 从 Full CD 到 SRBD 的建模假设

要从 Full CD 化到 SRBD，需要做以下近似。

### 5.1 删除关节自由度

假设全身质量分布相对 base 不变，或者关节只在名义姿态附近小幅运动：

\[
q_j \approx q_{j,0}
\]

\[
\dot{q}_j \approx 0
\]

于是：

\[
x_{CD}
=
\begin{bmatrix}
\bar{h}\\
p_b\\
\theta_b\\
q_j
\end{bmatrix}
\quad
\Longrightarrow
\quad
x_{SRBD}
\approx
\begin{bmatrix}
\theta\\
p\\
\omega\\
v
\end{bmatrix}
\]

Full CD 的输入：

\[
u_{CD}
=
\begin{bmatrix}
w_L\\
w_R\\
\dot{q}_j
\end{bmatrix}
\]

变成 SRBD 的输入：

\[
u_{SRBD}
=
\begin{bmatrix}
w_L\\
w_R
\end{bmatrix}
\]

OpenLoong 代码里又额外加了一个固定的 gravity pseudo input：

\[
u =
\begin{bmatrix}
f_L\\
\tau_L\\
f_R\\
\tau_R\\
f_g
\end{bmatrix}
\in \mathbb{R}^{13}
\]

其中 \(f_g = mg\) 被上下界固定住。

### 5.2 固定 CoM 和 base 的相对位置

SRBD 通常把机器人等效成一个刚体，其 CoM 与机身坐标系之间的偏移近似固定。

如果取 CoM 作为刚体参考点，则：

\[
p \equiv c
\]

OpenLoong 当前 `mpc.cpp` 实际上把状态里的位置用作 base/CoM tracking 位置，并用：

```cpp
pCoM = X_cur.block<3,1>(3,0);
pf2com = foot_pos_world - pCoM;
```

所以在 MPC 动力学中：

\[
p \approx c
\]

### 5.3 固定惯量

Full CD 里质心惯量来自 Pinocchio：

\[
I_G(q)
\]

随全身姿态变化。SRBD 近似为常数惯量：

\[
I_B \approx I_G(q_0)
\]

世界系惯量：

\[
I_W(\theta)
=
R(\theta) I_B R(\theta)^T
\]

OpenLoong 里进一步只使用 yaw：

\[
I_W
\approx
R_z(yaw) I_B R_z(yaw)^T
\]

对应代码：

```cpp
Ic_W_inv = (R_curz[i] * Ic * R_curz[i].transpose()).inverse();
```

### 5.4 用角速度替代归一化角动量

Full CD 的角动量状态是：

\[
\bar{h}_{ang} = L/m
\]

SRBD 更常用角速度：

\[
L = I_W \omega
\]

因此：

\[
\bar{h}_{ang}
=
\frac{1}{m} I_W \omega
\]

如果 \(I_W\) 冻结或缓慢变化，则：

\[
\dot{L}
=
I_W \dot{\omega}
\]

于是：

\[
\dot{\omega}
=
I_W^{-1}
\sum_i
\left((p_i-c)\times f_i+\tau_i\right)
\]

这一步就是从 CD 的角动量率方程到 SRBD 角加速度方程的关键。

### 5.5 冻结预测窗口内的接触点和力臂

完整 SRBD 中：

\[
r_i(k)=p_i(k)-c(k)
\]

如果 \(c(k)\) 是预测状态，而 \(f_i(k)\) 是优化变量，则：

\[
r_i(k)\times f_i(k)
=
(p_i(k)-c(k))\times f_i(k)
\]

包含“状态乘控制”的双线性项，QP 会变成非线性问题。

OpenLoong 当前做法是每次 MPC 求解前取当前值并冻结：

\[
r_i(k)
\approx
r_i^0
=
p_i^{now}-c^{now}
\]

所以：

\[
r_i(k)\times f_i(k)
\approx
r_i^0 \times f_i(k)
\]

这样输入仍然线性。

对应代码：

```cpp
pf2com = foot_pos_world - pCoM;

for (int i = 0; i < mpc_N; i++) {
  pf2comi[i] = pf2com;
}
```

## 6. SRBD 连续动力学

经过上述近似，得到 SRBD 状态：

\[
x =
\begin{bmatrix}
\theta\\
p\\
\omega\\
v
\end{bmatrix}
\in \mathbb{R}^{12}
\]

其中 OpenLoong 顺序是：

```text
theta = [roll, pitch, yaw]
p     = [x, y, z]
omega = [wx, wy, wz]
v     = [vx, vy, vz]
```

输入：

\[
u =
\begin{bmatrix}
f_L\\
\tau_L\\
f_R\\
\tau_R\\
f_g
\end{bmatrix}
\in \mathbb{R}^{13}
\]

其中：

\[
f_g = m g
\]

连续动力学：

\[
\dot{\theta}
=
E(\theta)\omega
\]

\[
\dot{p}=v
\]

\[
\dot{\omega}
=
I_W^{-1}
\left(
r_L \times f_L+\tau_L
+
r_R \times f_R+\tau_R
\right)
\]

\[
\dot{v}
=
\frac{1}{m}(f_L+f_R)
+
\frac{1}{m}
\begin{bmatrix}
0\\0\\f_g
\end{bmatrix}
\]

OpenLoong 里：

\[
E(\theta) \approx R_z(yaw)^T
\]

并且 \(g=-9.8\)，所以 \(f_g=mg<0\)。

## 7. 写成线性系统矩阵

定义叉乘矩阵：

\[
[r]_\times f = r \times f
\]

OpenLoong 中 `CrossProduct_A(r)` 就是 \([r]_\times\)：

```cpp
M << 0, -r_z, r_y,
     r_z, 0, -r_x,
    -r_y, r_x, 0;
```

连续系统写成：

\[
\dot{x} = A_c x + B_c u
\]

其中 \(A_c\) 只有两个主要块：

\[
A_c[0:3, 6:9] = R_z(yaw)^T
\]

\[
A_c[3:6, 9:12] = I_3
\]

也就是：

\[
\dot{\theta}=R_z^T\omega
\]

\[
\dot{p}=v
\]

输入矩阵 \(B_c\) 的非零块：

\[
B_c[6:9,0:3]
=
I_W^{-1}[r_L]_\times
\]

\[
B_c[6:9,3:6]
=
I_W^{-1}
\]

\[
B_c[6:9,6:9]
=
I_W^{-1}[r_R]_\times
\]

\[
B_c[6:9,9:12]
=
I_W^{-1}
\]

\[
B_c[9:12,0:3]
=
\frac{1}{m}I_3
\]

\[
B_c[9:12,6:9]
=
\frac{1}{m}I_3
\]

\[
B_c[11,12]
=
\frac{1}{m}
\]

最后一个块表示：

\[
\dot{v}_z \leftarrow f_g/m
\]

对应代码：

```cpp
Ac[i].block<3, 3>(0, 6) = R_curz[i].transpose();
Ac[i].block<3, 3>(3, 9) = Identity3;

Bc[i].block<3, 3>(6, 0) = Ic_W_inv * CrossProduct_A(rL);
Bc[i].block<3, 3>(6, 3) = Ic_W_inv;
Bc[i].block<3, 3>(6, 6) = Ic_W_inv * CrossProduct_A(rR);
Bc[i].block<3, 3>(6, 9) = Ic_W_inv;

Bc[i].block<3, 3>(9, 0) = Identity3 / m;
Bc[i].block<3, 3>(9, 6) = Identity3 / m;
Bc[i](11, 12) = 1.0 / m;
```

## 8. 离散化

OpenLoong 用前向欧拉：

\[
x_{k+1}
=
A_k x_k+B_k u_k
\]

\[
A_k = I + dt A_c
\]

\[
B_k = dt B_c
\]

代码：

```cpp
A[i] = Identity(nx,nx) + dt * Ac[i];
B[i] = dt * Bc[i];
```

然后把 horizon 内状态堆叠：

\[
X =
\begin{bmatrix}
x_1\\
x_2\\
\vdots\\
x_N
\end{bmatrix}
\]

得到 condensed prediction：

\[
X = A_{qp} x_0 + B_{qp} U
\]

当前代码：

```text
mpc_N = 10
ch    = 3
nx    = 12
nu    = 13
```

也就是预测 10 步状态，但只优化 3 步控制：

\[
U =
\begin{bmatrix}
u_0\\u_1\\u_2
\end{bmatrix}
\]

第 4 到第 10 步复用 \(u_2\)。这由 `Bqp11` 实现。

## 9. SRBD-QP 的目标函数

OpenLoong 的 QP 变量是 \(U\)。目标函数写成：

\[
\min_U
(X-X_d)^T L (X-X_d)
+
\alpha (U+\delta U)^T K (U+\delta U)
\]

代入：

\[
X=A_{qp}x_0+B_{qp}U
\]

得到：

\[
\min_U
\frac{1}{2}U^T H U + c^T U
\]

其中代码中：

\[
H =
2(B_{qp}^T L B_{qp}+\alpha K)+10^{-10}I
\]

\[
c =
2B_{qp}^T L(A_{qp}x_0-X_d)
+
2\alpha K\delta U
\]

对应代码：

```cpp
H = 2 * (Bqp.transpose() * L * Bqp + alpha * K)
    + 1e-10 * Identity;

c = 2 * Bqp.transpose() * L * (Aqp * X_cur - Xd)
    + 2 * alpha * K * delta_U;
```

这里 \(\delta U\) 是重力补偿附近的 nominal wrench。因为 OpenLoong 里 \(g=-9.8\)，所以 `delta_U` 中的 \(m g\) 是负数，而目标写成 \(U+\delta U\)，等价于鼓励 \(U\) 接近向上的支撑力。

## 10. 接触约束如何从 CD 继承到 SRBD

CD 和 SRBD 对接触 wrench 的物理约束本质一样：

```text
接触脚:
  fz >= fz_min
  |fx| <= mu fz
  |fy| <= mu fz
  CoP 在足底矩形内
  yaw torque 有限

摆动脚:
  wrench = 0
```

OpenLoong 使用线性不等式：

\[
A_s U \le b
\]

单脚 11 条：

\[
-f_z \le -f_{z,min}
\]

\[
f_x - \frac{\mu}{\sqrt{2}}f_z \le 0
\]

\[
-f_x - \frac{\mu}{\sqrt{2}}f_z \le 0
\]

\[
f_y - \frac{\mu}{\sqrt{2}}f_z \le 0
\]

\[
-f_y - \frac{\mu}{\sqrt{2}}f_z \le 0
\]

\[
\tau_x - w f_z \le 0
\]

\[
-\tau_x - w f_z \le 0
\]

\[
\tau_y - l_f f_z \le 0
\]

\[
-\tau_y + l_b f_z \le 0
\]

\[
\tau_z - \gamma f_z \le 0
\]

\[
-\tau_z - \gamma f_z \le 0
\]

其中：

```text
w      = foot_half_width
l_f    = foot_l_front
l_b    = foot_l_back
gamma  = foot_yaw_friction
```

代码中会先把 world-frame wrench 转到 foot yaw frame：

\[
w_{foot} =
\begin{bmatrix}
R_{w2f} & 0\\
0 & R_{w2f}
\end{bmatrix}
w_{world}
\]

再套用足底约束。

## 11. 与 CppAD CD 的逐项对应关系

### 11.1 线动量项

CD：

\[
\dot{\bar{h}}_{lin}
=
\frac{f_L+f_R}{m}+g
\]

SRBD：

\[
\dot{v}
=
\frac{f_L+f_R}{m}+g
\]

由于：

\[
\bar{h}_{lin}=p_{lin}/m=\dot{c}=v
\]

两者完全一致。

OpenLoong 只是把重力写成固定输入：

\[
g =
\frac{1}{m}
\begin{bmatrix}
0\\0\\f_g
\end{bmatrix},
\quad
f_g=mg
\]

### 11.2 角动量项

CD：

\[
\dot{\bar{h}}_{ang}
=
\frac{
r_L\times f_L+\tau_L+
r_R\times f_R+\tau_R
}{m}
\]

SRBD 先恢复未归一化角动量：

\[
L=m\bar{h}_{ang}
\]

于是：

\[
\dot{L}
=
r_L\times f_L+\tau_L+
r_R\times f_R+\tau_R
\]

再用单刚体关系：

\[
L=I_W\omega
\]

冻结 \(I_W\) 后：

\[
I_W\dot{\omega}
=
r_L\times f_L+\tau_L+
r_R\times f_R+\tau_R
\]

\[
\dot{\omega}
=
I_W^{-1}
\left(
r_L\times f_L+\tau_L+
r_R\times f_R+\tau_R
\right)
\]

这正是 `Bc` 的角速度行。

### 11.3 base pose 项

CD：

\[
\dot{q}_b
=
\text{function}(A_g(q),\bar{h},\dot{q}_j,\theta_b)
\]

SRBD：

\[
\dot{p}=v
\]

\[
\dot{\theta}=E(\theta)\omega
\]

OpenLoong 又近似：

\[
E(\theta)\approx R_z(yaw)^T
\]

这是从全身质心模型到工程 SRBD 的一处明显简化。

### 11.4 关节项

CD：

\[
\dot{q}_j = u_{qdot}
\]

SRBD：

删除该状态和输入。其影响被吸收到常数质量、常数惯量、冻结 CoM 偏移和冻结足端力臂中。

## 12. 当前 OpenLoong SRBD 的具体近似与影响

### 12.1 预测窗口内力臂冻结

每次求解时：

\[
r_L^0 = p_L^{now}-p^{now}
\]

\[
r_R^0 = p_R^{now}-p^{now}
\]

整个 horizon 都使用 \(r_i^0\)：

\[
r_i(k) \approx r_i^0
\]

它没有对未来 \(p_{com,k}\) 更新力臂。这不是取平均，而是取当前瞬时值并冻结。

好处：

```text
保持线性 QP，速度快。
```

代价：

```text
CoM 在预测窗口内大幅移动、支撑切换、跳跃或快速步态时，
角动量预测会偏粗。
```

### 12.2 只用 yaw 旋转惯量和权重

理论 SRBD 可用完整姿态：

\[
I_W=R(\theta)I_BR(\theta)^T
\]

当前代码用：

\[
I_W\approx R_z(yaw)I_BR_z(yaw)^T
\]

这适合 roll/pitch 小的行走场景。

### 12.3 OpenLoong 状态里的位置更像 base/CoM 混用

`mpc.cpp` 中：

```cpp
X_cur[3:6] = Data.q[0:3]
pCoM = X_cur[3:6]
```

而 `pino_kin_dyn.cpp` 里也有真实 CoM：

```cpp
robotState.pCoM_W = CoM_pos;
```

所以需要注意：当前 SRBD-MPC 用的是 `Data.q` 的浮动基位置作为 `pCoM`，并不显式使用 `Data.pCoM_W`。如果 base 和 CoM 偏移不可忽略，这里会带来系统误差。

## 13. 最终从 CD 到 SRBD 的压缩公式

从 Full CD：

\[
x_{CD}
=
\begin{bmatrix}
\dot{c}\\
L/m\\
p_b\\
\theta_b\\
q_j
\end{bmatrix}
\]

\[
u_{CD}
=
\begin{bmatrix}
f_L\\
\tau_L\\
f_R\\
\tau_R\\
\dot{q}_j
\end{bmatrix}
\]

\[
\dot{\bar{h}}
=
\frac{1}{m}
\left[
\begin{array}{c}
f_L+f_R+mg\\
r_L\times f_L+\tau_L+r_R\times f_R+\tau_R
\end{array}
\right]
\]

通过假设：

```text
q_j 固定
qdot_j = 0
c ≈ p
I(q) ≈ I_B 常数
L = I_W omega
r_i = p_i - c 在每次 QP 内冻结
```

得到 OpenLoong SRBD：

\[
x_{SRBD}
=
\begin{bmatrix}
\theta\\
p\\
\omega\\
v
\end{bmatrix}
\]

\[
u_{SRBD}
=
\begin{bmatrix}
f_L\\
\tau_L\\
f_R\\
\tau_R\\
f_g
\end{bmatrix}
\]

\[
\dot{x}
=
\begin{bmatrix}
R_z(yaw)^T\omega\\
v\\
I_W^{-1}
\left(
[r_L]_\times f_L+\tau_L+[r_R]_\times f_R+\tau_R
\right)\\
\frac{1}{m}(f_L+f_R+[0,0,f_g]^T)
\end{bmatrix}
\]

其中：

\[
I_W = R_z(yaw) I_B R_z(yaw)^T
\]

\[
f_g = mg
\]

再前向欧拉离散化：

\[
x_{k+1}
=
(I+dtA_c)x_k+dtB_cu_k
\]

这就是 `algorithm/mpc.cpp` 中的 SRBD-MPC 形式。

## 14. 如果要把当前 SRBD 向 CD 靠近，优先改哪里

可以按复杂度逐级增强：

1. 使用真实 CoM：

   ```cpp
   pCoM = Data.pCoM_W;
   ```

   而不是直接用 floating base position。

2. 在 horizon 中用参考轨迹更新力臂：

   \[
   r_i(k) \approx p_i^{ref}(k)-p_{com}^{ref}(k)
   \]

   这样仍是线性 QP，因为力臂来自参考/上一轮预测，不是当前优化变量。

3. 使用完整姿态旋转惯量：

   \[
   I_W=R(roll,pitch,yaw)I_BR^T
   \]

4. 把状态换成质心动量：

   \[
   x=[c,\theta,\bar{h}_{lin},\bar{h}_{ang}]
   \]

   这样更接近 OCS2 CD，但需要重新定义跟踪项和 WBC 接口。

5. 引入关节/上身动量影响：

   这就接近 Full CD/NMPC，普通线性 QP 会不够，需要 SQP/DDP/NMPC 或者 successive linearization。

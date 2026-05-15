下面是一份可以直接提交给 Codex 的 **完整改进流程说明**。目标是把当前 `G1SrbdMpcRp` 从“标准 SRBD 上附加两个 torso torque 输入”的版本，改成更合理的：

[
\boxed{
\text{pitch/roll enhanced SRBD-MPC}
}
]

也可以称为：

[
\boxed{
\text{RP-DSRB / Roll-Pitch Decomposed SRBD}
}
]

---

# 目标

当前代码已经添加了：

[
\phi_T,\theta_T,\dot\phi_T,\dot\theta_T
]

以及输入：

[
\tau_\phi,\tau_\theta
]

但目前主要问题是：

1. base angular dynamics 仍然使用 whole-body inertia；
2. lower-body inertia 和 torso inertia 没有清晰分离；
3. MPC 求出的 torso torque 没有真正写入 `DataBus` 并执行；
4. 没有 torso angle/rate 状态约束；
5. 没有输出 waist desired position/rate 给 IK/PD；
6. torque-rate cost 没有惩罚首个 knot 和上一帧 torque 的跳变；
7. contact schedule 只复制当前接触状态，没有 horizon preview；
8. `current` 中的 CoM velocity 来源需要确认；
9. torque 坐标系和 joint index 需要显式检查。

目标是改成如下动力学结构：

[
I_{\mathrm{LB,rp}}\dot\omega_B
==============================

## M_{\mathrm{ct}}

S_{rp}^{W}\tau_{rp}
]

[
I_{\mathrm{TR,rp}}\ddot q_{rp}
==============================

\tau_{rp}
]

其中：

[
q_{rp}
======

\begin{bmatrix}
\phi_T\
\theta_T
\end{bmatrix}
]

[
\tau_{rp}
=========

\begin{bmatrix}
\tau_\phi\
\tau_\theta
\end{bmatrix}
]

[
I_{\mathrm{LB,rp}}
==================

\mathrm{diag}
(I_{\mathrm{LB},x},I_{\mathrm{LB},y},I_{\mathrm{COM},z})
]

最终满足：

[
\dot H_{\mathrm{LB}}
+
\dot H_{\mathrm{TR}}
====================

M_{\mathrm{ct}}
]

---

# Phase 1：整理模型参数

## 1.1 在 `G1SrbdMpcRp` 类中新增参数

在 `g1_srbd_mpc_rp.h` 中添加：

```cpp
// RP-DSRB inertia parameters.
// Roll/pitch use lower-body inertia; yaw keeps whole-body inertia.
double lowerBodyInertiaRoll = 1.0;   // I_LB,x
double lowerBodyInertiaPitch = 1.0;  // I_LB,y
double wholeBodyInertiaYaw = 1.0;    // I_COM,z

// Torso/upper-body inertia about waist roll/pitch joint axes.
double torsoInertiaRoll = 1.0;       // I_TR,x^J
double torsoInertiaPitch = 1.0;      // I_TR,y^J

// Torso torque limits.
double torsoTorqueMaxRoll = 20.0;
double torsoTorqueMaxPitch = 20.0;

// Torso angle limits.
double torsoRollMax = 0.35;
double torsoPitchMax = 0.35;

// Torso rate limits.
double torsoRollRateMax = 2.0;
double torsoPitchRateMax = 2.0;

// Optional torso reference.
double torsoRollRef = 0.0;
double torsoPitchRef = 0.0;

// Torque smoothing.
double torsoTorqueRateWeight = 1e-3;

// Last solved torso torque for rate regularization.
Eigen::Vector2d lastTorsoTorqueRp_ = Eigen::Vector2d::Zero();
```

如果原来已经存在部分变量，则保留已有变量，但语义必须改成：

* `lowerBodyInertiaRoll`：lower-body about whole-body CoM 的 roll inertia；
* `lowerBodyInertiaPitch`：lower-body about whole-body CoM 的 pitch inertia；
* `wholeBodyInertiaYaw`：whole-body about CoM 的 yaw inertia；
* `torsoInertiaRoll`：upper-body about waist roll joint axis；
* `torsoInertiaPitch`：upper-body about waist pitch joint axis。

---

# Phase 2：修正 `buildDiscreteModel()`

这是最核心部分。

当前代码中：

```cpp
const Eigen::Matrix3d R = quatToMat(input.orientation);
const Eigen::Matrix3d IWorld = R * input.inertiaBody * R.transpose();
const Eigen::Matrix3d IInv = IWorld.inverse();
```

需要替换成 RP inertia。

---

## 2.1 替换 angular inertia

改成：

```cpp
const Eigen::Matrix3d R = quatToMat(input.orientation);

// RP-DSRB angular inertia.
// Roll/pitch use lower-body inertia.
// Yaw keeps whole-body inertia.
Eigen::Matrix3d IangBody = Eigen::Matrix3d::Zero();
IangBody(0, 0) = std::max(1e-6, lowerBodyInertiaRoll);
IangBody(1, 1) = std::max(1e-6, lowerBodyInertiaPitch);
IangBody(2, 2) = std::max(1e-6, wholeBodyInertiaYaw);

// Convert body-frame diagonal inertia inverse to world frame.
const Eigen::Matrix3d IangInvWorld =
    R * IangBody.inverse() * R.transpose();
```

然后把原来的：

```cpp
IInv
```

全部替换成：

```cpp
IangInvWorld
```

也就是 contact angular dynamics 改成：

```cpp
Bc.block(kIdxOmega, 6 * leg, 3, 6) =
    IangInvWorld * planeToCentroidal.bottomRows<3>();
```

torso reaction torque 改成：

```cpp
Eigen::Matrix<double, 3, 2> SrpBody;
SrpBody << 1.0, 0.0,
           0.0, 1.0,
           0.0, 0.0;

const Eigen::Matrix<double, 3, 2> SrpWorld = R * SrpBody;

// Positive torso roll/pitch torque produces negative reaction torque on lower body.
Bc.block<3, 2>(kIdxOmega, kIdxTorsoInput) =
    -IangInvWorld * SrpWorld;
```

---

## 2.2 保留 torso relative dynamics

当前这部分基本正确：

```cpp
Eigen::Matrix2d ItrInv = Eigen::Matrix2d::Zero();
ItrInv(0, 0) = 1.0 / std::max(1e-6, torsoInertiaRoll);
ItrInv(1, 1) = 1.0 / std::max(1e-6, torsoInertiaPitch);
Bc.block<2, 2>(kIdxTorsoRpRate, kIdxTorsoInput) = ItrInv;
```

保留它。

它对应：

[
\ddot\phi_T=\frac{\tau_\phi}{I_{\mathrm{TR},x}^{J}}
]

[
\ddot\theta_T=\frac{\tau_\theta}{I_{\mathrm{TR},y}^{J}}
]

---

## 2.3 修改后的 `buildDiscreteModel()` 目标结构

最终该函数核心应为：

```cpp
void G1SrbdMpcRp::buildDiscreteModel(
    const Input &input,
    int knot,
    Eigen::Matrix<double, kNx, kNx> &A,
    Eigen::Matrix<double, kNx, kNu> &B) const
{
    A.setIdentity();
    B.setZero();

    const Eigen::Matrix3d R = quatToMat(input.orientation);

    Eigen::Matrix3d IangBody = Eigen::Matrix3d::Zero();
    IangBody(0, 0) = std::max(1e-6, lowerBodyInertiaRoll);
    IangBody(1, 1) = std::max(1e-6, lowerBodyInertiaPitch);
    IangBody(2, 2) = std::max(1e-6, wholeBodyInertiaYaw);

    const Eigen::Matrix3d IangInvWorld =
        R * IangBody.inverse() * R.transpose();

    const Eigen::Matrix<double, 6, 6> HContact =
        g1_kin_dyn::planeForceToWrenchMap();

    Eigen::Matrix<double, kNx, kNx> Ac =
        Eigen::Matrix<double, kNx, kNx>::Zero();

    Ac.block<3, 3>(kIdxRpy, kIdxOmega) = Rz3(input.current(2));
    Ac.block<3, 3>(kIdxPos, kIdxVel).setIdentity();
    Ac(kIdxVel + 2, kIdxGravity) = -1.0;
    Ac.block<2, 2>(kIdxTorsoRp, kIdxTorsoRpRate).setIdentity();

    Eigen::Matrix<double, kNx, kNu> Bc =
        Eigen::Matrix<double, kNx, kNu>::Zero();

    Eigen::Vector3d com = input.current.segment<3>(kIdxPos);
    if (input.reference.cols() > knot)
    {
        com = input.reference.col(knot).segment<3>(kIdxPos);
    }

    const std::array<Eigen::Vector3d, 2> *contactPositions =
        &input.contactPositionWorld;

    if (static_cast<int>(input.contactPositionWorldHorizon.size()) > knot)
    {
        contactPositions = &input.contactPositionWorldHorizon[knot];
    }

    for (int leg = 0; leg < 2; ++leg)
    {
        const Eigen::Vector3d r = (*contactPositions)[leg] - com;

        Eigen::Matrix<double, 6, 6> G =
            Eigen::Matrix<double, 6, 6>::Zero();

        G.block<3, 3>(0, 0).setIdentity();
        G.block<3, 3>(3, 0) = skew(r);
        G.block<3, 3>(3, 3).setIdentity();

        const Eigen::Matrix<double, 6, 6> planeToCentroidal =
            G * HContact;

        Bc.block(kIdxVel, 6 * leg, 3, 6) =
            planeToCentroidal.topRows<3>() / input.mass;

        Bc.block(kIdxOmega, 6 * leg, 3, 6) =
            IangInvWorld * planeToCentroidal.bottomRows<3>();
    }

    Eigen::Matrix<double, 3, 2> SrpBody;
    SrpBody << 1.0, 0.0,
               0.0, 1.0,
               0.0, 0.0;

    const Eigen::Matrix<double, 3, 2> SrpWorld = R * SrpBody;

    Bc.block<3, 2>(kIdxOmega, kIdxTorsoInput) =
        -IangInvWorld * SrpWorld;

    Eigen::Matrix2d ItrInv = Eigen::Matrix2d::Zero();
    ItrInv(0, 0) = 1.0 / std::max(1e-6, torsoInertiaRoll);
    ItrInv(1, 1) = 1.0 / std::max(1e-6, torsoInertiaPitch);

    Bc.block<2, 2>(kIdxTorsoRpRate, kIdxTorsoInput) =
        ItrInv;

    A += dt_ * Ac;
    B = dt_ * Bc;
}
```

---

# Phase 3：修正 `buildInputFromDataBus()`

## 3.1 检查 torso joint index

当前代码：

```cpp
robotState.motors_pos_cur.size() > 13 ? robotState.motors_pos_cur[13] : 0.0,
robotState.motors_pos_cur.size() > 14 ? robotState.motors_pos_cur[14] : 0.0,
robotState.motors_vel_cur.size() > 13 ? robotState.motors_vel_cur[13] : 0.0,
robotState.motors_vel_cur.size() > 14 ? robotState.motors_vel_cur[14] : 0.0;
```

必须确认：

```text
motors_pos_cur[13] = waist_roll
motors_pos_cur[14] = waist_pitch
```

如果实际 joint order 是：

```text
waist_yaw, waist_roll, waist_pitch
```

那么 index 可能是：

```cpp
waist_roll_id = 13;
waist_pitch_id = 14;
```

如果实际是：

```text
waist_roll, waist_pitch, waist_yaw
```

也可能不同。

建议不要硬编码，改成类成员：

```cpp
int waistRollJointId = 13;
int waistPitchJointId = 14;
```

然后写：

```cpp
const double waistRoll =
    robotState.motors_pos_cur.size() > waistRollJointId
        ? robotState.motors_pos_cur[waistRollJointId]
        : 0.0;

const double waistPitch =
    robotState.motors_pos_cur.size() > waistPitchJointId
        ? robotState.motors_pos_cur[waistPitchJointId]
        : 0.0;

const double waistRollRate =
    robotState.motors_vel_cur.size() > waistRollJointId
        ? robotState.motors_vel_cur[waistRollJointId]
        : 0.0;

const double waistPitchRate =
    robotState.motors_vel_cur.size() > waistPitchJointId
        ? robotState.motors_vel_cur[waistPitchJointId]
        : 0.0;
```

然后填入 state。

---

## 3.2 检查 CoM velocity

当前代码使用：

```cpp
robotState.dq(0), robotState.dq(1), robotState.dq(2)
```

作为 CoM velocity。

这需要确认。如果 `dq(0:2)` 不是 world-frame CoM velocity，应改成：

```cpp
robotState.vCoM_W
```

如果 DataBus 没有 `vCoM_W`，建议新增字段，或者至少改成明确的：

```cpp
robotState.base_vel_W
```

目标是：

```cpp
input.current.segment<3>(kIdxVel) = robotState.vCoM_W;
```

不要混用 generalized velocity 和 CoM velocity。

---

## 3.3 torso reference 支持 pitch lean

当前 torso reference 是：

```cpp
0.0, 0.0,
0.0, 0.0;
```

建议改成：

```cpp
torsoRollRef, torsoPitchRef,
0.0, 0.0;
```

完整片段：

```cpp
input.reference.col(k) << 0.0, 0.0, robotState.walk_target_yaw,
    pRef.x(), pRef.y(), pRef.z(),
    0.0, 0.0, robotState.walk_target_yaw_rate,
    desiredVel.x(), desiredVel.y(), desiredVel.z(),
    kGravity,
    torsoRollRef, torsoPitchRef,
    0.0, 0.0;
```

如果机器人容易后仰，可以设置：

```cpp
torsoPitchRef = -0.05;  // sign depends on robot convention
```

---

# Phase 4：改进 contact schedule

当前 contact table 是：

```cpp
input.contactTable(k, 0) =
    (robotState.motionState == DataBus::Stand || robotState.walk_left_contact) ? 1 : 0;

input.contactTable(k, 1) =
    (robotState.motionState == DataBus::Stand || robotState.walk_right_contact) ? 1 : 0;
```

这等价于把当前接触状态复制到整个 horizon。

建议改成 horizon preview。

如果已有 gait schedule，例如：

```cpp
robotState.walk_phase
robotState.step_time
robotState.double_support_time
robotState.swing_leg
```

则实现函数：

```cpp
std::array<int, 2> getContactAtFutureTime(const DataBus &robotState, double tFuture);
```

然后：

```cpp
for (int k = 0; k < horizon_; ++k)
{
    const double tFuture = static_cast<double>(k) * dt_;
    const auto contact = getContactAtFutureTime(robotState, tFuture);

    input.contactTable(k, 0) = contact[0];
    input.contactTable(k, 1) = contact[1];
}
```

如果暂时没有 gait preview，可以保留当前逻辑，但需要加注释：

```cpp
// TODO: Replace current-contact replication with horizon gait schedule.
// Current implementation assumes contact mode is constant over the MPC horizon.
```

---

# Phase 5：增加 torso state constraints

当前只有输入约束：

[
|\tau_\phi|\le \tau_{\phi,\max}
]

[
|\tau_\theta|\le \tau_{\theta,\max}
]

必须增加状态约束：

[
|\phi_T|\le \phi_{\max}
]

[
|\theta_T|\le \theta_{\max}
]

[
|\dot\phi_T|\le \dot\phi_{\max}
]

[
|\dot\theta_T|\le \dot\theta_{\max}
]

由于当前 QP 是 condensed input-only：

[
X = A_{\mathrm{qp}}x_0 + B_{\mathrm{qp}}U
]

因此状态约束要写成：

[
x_{\min} - A_i x_0
\le
B_i U
\le
x_{\max} - A_i x_0
]

---

## 5.1 修改约束行数

当前：

```cpp
const int torsoTorqueRowsPerKnot = 4;
const int rowsPerKnot = contactRowsPerKnot + torsoTorqueRowsPerKnot;
```

改成：

```cpp
const int torsoTorqueRowsPerKnot = 4;
const int torsoStateRowsPerKnot = 4;
const int rowsPerKnot =
    contactRowsPerKnot + torsoTorqueRowsPerKnot + torsoStateRowsPerKnot;
```

注意：因为使用 `lbA <= A U <= ubA`，每个状态只需要一行双边约束，所以 4 个 torso 状态只需要 4 行。

---

## 5.2 添加 helper lambda

在构建 `Acon/lb/ub` 时，加入：

```cpp
auto addCondensedStateBound =
    [&](int row,
        int knot,
        int stateIdx,
        double lower,
        double upper)
{
    const Eigen::RowVectorXd Bi =
        Bqp.block(knot * nx + stateIdx, 0, 1, nVar);

    const double ai =
        (Aqp.block(knot * nx + stateIdx, 0, 1, nx) * x0)(0);

    Acon.row(row) = Bi;
    lb(row) = lower - ai;
    ub(row) = upper - ai;
};
```

注意这个 lambda 需要在 `x0`、`Aqp`、`Bqp` 已经构建之后使用。

---

## 5.3 每个 knot 添加 torso state bounds

在每个 knot 的约束构造里，torso torque rows 后面添加：

```cpp
const int torsoStateRow = torsoRow + torsoTorqueRowsPerKnot;

addCondensedStateBound(
    torsoStateRow + 0,
    k,
    kIdxTorsoRp + 0,
    -torsoRollMax,
    torsoRollMax);

addCondensedStateBound(
    torsoStateRow + 1,
    k,
    kIdxTorsoRp + 1,
    -torsoPitchMax,
    torsoPitchMax);

addCondensedStateBound(
    torsoStateRow + 2,
    k,
    kIdxTorsoRpRate + 0,
    -torsoRollRateMax,
    torsoRollRateMax);

addCondensedStateBound(
    torsoStateRow + 3,
    k,
    kIdxTorsoRpRate + 1,
    -torsoPitchRateMax,
    torsoPitchRateMax);
```

---

# Phase 6：改进 cost 权重

当前 torso 权重是：

```cpp
10.0, 10.0,
1.0, 1.0;
```

建议改成 pitch 更强：

```cpp
stateWeights(kIdxTorsoRp + 0) = 20.0;       // torso roll
stateWeights(kIdxTorsoRp + 1) = 80.0;       // torso pitch
stateWeights(kIdxTorsoRpRate + 0) = 2.0;    // torso roll rate
stateWeights(kIdxTorsoRpRate + 1) = 8.0;    // torso pitch rate
```

input weights 建议：

```cpp
inputWeights(kIdxTorsoInput + 0) = 1e-3;    // tau roll
inputWeights(kIdxTorsoInput + 1) = 2e-3;    // tau pitch
```

如果腰部使用太多，增大：

```cpp
inputWeights(kIdxTorsoInput + 0)
inputWeights(kIdxTorsoInput + 1)
```

如果后仰仍然明显，优先增大：

```cpp
stateWeights(kIdxTorsoRp + 1)
stateWeights(kIdxTorsoRpRate + 1)
```

不要无限降低 `tau_pitch` cost，否则会出现腰部过度摆动。

---

# Phase 7：增加首 knot torque-rate cost

当前 torque-rate cost 只惩罚：

[
\tau_k-\tau_{k-1},\quad k\ge 1
]

没有惩罚：

[
\tau_0-\tau_{\mathrm{prev}}
]

需要新增：

```cpp
for (int j = 0; j < kNuTorso; ++j)
{
    const int idx0 = kIdxTorsoInput + j;
    H(idx0, idx0) += 2.0 * torsoTorqueRateWeight;
    g(idx0) -= 2.0 * torsoTorqueRateWeight * lastTorsoTorqueRp_(j);
}
```

位置放在现有 torque-rate cost 前后都可以，但要在求解前。

现有代码：

```cpp
for (int k = 1; k < horizon_; ++k)
{
    for (int j = 0; j < kNuTorso; ++j)
    {
        const int idxCurr = k * nu + kIdxTorsoInput + j;
        const int idxPrev = (k - 1) * nu + kIdxTorsoInput + j;
        H(idxCurr, idxCurr) += 2.0 * torsoTorqueRateWeight;
        H(idxPrev, idxPrev) += 2.0 * torsoTorqueRateWeight;
        H(idxCurr, idxPrev) -= 2.0 * torsoTorqueRateWeight;
        H(idxPrev, idxCurr) -= 2.0 * torsoTorqueRateWeight;
    }
}
```

保留。

求解成功后更新：

```cpp
lastTorsoTorqueRp_ = firstTorsoTorqueRp_;
```

如果求解失败，不建议更新。

---

# Phase 8：恢复预测状态并输出 torso desired state

当前只取了第一步输入：

```cpp
firstPlaneForces_
firstTorsoTorqueRp_
```

建议恢复预测状态：

```cpp
Eigen::VectorXd U = Eigen::VectorXd::Zero(nVar);
for (int i = 0; i < nVar; ++i)
{
    U(i) = sol[i];
}

Eigen::VectorXd Xpred = Aqp * x0 + Bqp * U;
```

然后取第一个 knot：

```cpp
const Eigen::VectorXd x1 = Xpred.segment(0, nx);

firstTorsoRp_(0) = x1(kIdxTorsoRp + 0);
firstTorsoRp_(1) = x1(kIdxTorsoRp + 1);

firstTorsoRpRate_(0) = x1(kIdxTorsoRpRate + 0);
firstTorsoRpRate_(1) = x1(kIdxTorsoRpRate + 1);
```

需要在 class 中新增：

```cpp
Eigen::Vector2d firstTorsoRp_ = Eigen::Vector2d::Zero();
Eigen::Vector2d firstTorsoRpRate_ = Eigen::Vector2d::Zero();
```

在 `solve()` 开始时清零：

```cpp
firstTorsoRp_.setZero();
firstTorsoRpRate_.setZero();
```

---

# Phase 9：DataBus 输出

当前 `dataBusWrite()` 只输出：

```cpp
robotState.Fr_ff = firstWrenches_;
robotState.qpStatus_MPC = qpStatus_;
```

需要扩展 DataBus。

---

## 9.1 在 DataBus 中新增字段

在 `DataBus` 定义中添加：

```cpp
bool srbd_mpc_rp_enabled = false;

Eigen::Vector2d srbd_mpc_torso_tau_rp = Eigen::Vector2d::Zero();

Eigen::Vector2d srbd_mpc_torso_rp_des = Eigen::Vector2d::Zero();
Eigen::Vector2d srbd_mpc_torso_rp_rate_des = Eigen::Vector2d::Zero();
```

如果项目不希望 DataBus 依赖 Eigen，可以改成：

```cpp
double srbd_mpc_tau_roll = 0.0;
double srbd_mpc_tau_pitch = 0.0;

double srbd_mpc_waist_roll_des = 0.0;
double srbd_mpc_waist_pitch_des = 0.0;

double srbd_mpc_waist_roll_vel_des = 0.0;
double srbd_mpc_waist_pitch_vel_des = 0.0;
```

---

## 9.2 修改 `dataBusWrite()`

```cpp
void G1SrbdMpcRp::dataBusWrite(DataBus &robotState) const
{
    robotState.Fr_ff = firstWrenches_;
    robotState.qpStatus_MPC = qpStatus_;

    robotState.srbd_mpc_rp_enabled = (qpStatus_ == 0);

    robotState.srbd_mpc_torso_tau_rp = firstTorsoTorqueRp_;
    robotState.srbd_mpc_torso_rp_des = firstTorsoRp_;
    robotState.srbd_mpc_torso_rp_rate_des = firstTorsoRpRate_;
}
```

如果 `qpStatus_ == 0` 不代表成功，请用项目中实际成功状态判断。

---

# Phase 10：底层 torque mapping 修改

在 low-level torque 输出处，不要把：

[
\tau_\phi,\tau_\theta
]

塞进 foot Jacobian。

错误写法：

```cpp
tau += Jfoot.transpose() * [F, M, tau_phi, tau_theta];
```

正确写法：

```cpp
tau_joint =
    J_foot.transpose() * contact_wrench
    + waist_torque
    + PD;
```

也就是：

```cpp
if (robotState.srbd_mpc_rp_enabled)
{
    tau[waistRollJointId] += robotState.srbd_mpc_torso_tau_rp(0);
    tau[waistPitchJointId] += robotState.srbd_mpc_torso_tau_rp(1);
}
```

如果底层是 IK + PD，建议更进一步：

```cpp
if (robotState.srbd_mpc_rp_enabled)
{
    const double rollDes =
        robotState.srbd_mpc_torso_rp_des(0);
    const double pitchDes =
        robotState.srbd_mpc_torso_rp_des(1);

    const double rollRateDes =
        robotState.srbd_mpc_torso_rp_rate_des(0);
    const double pitchRateDes =
        robotState.srbd_mpc_torso_rp_rate_des(1);

    const double tauRollFf =
        robotState.srbd_mpc_torso_tau_rp(0);
    const double tauPitchFf =
        robotState.srbd_mpc_torso_tau_rp(1);

    tau[waistRollJointId] =
        kpWaistRoll * (rollDes - q[waistRollJointId])
        + kdWaistRoll * (rollRateDes - dq[waistRollJointId])
        + tauRollFf;

    tau[waistPitchJointId] =
        kpWaistPitch * (pitchDes - q[waistPitchJointId])
        + kdWaistPitch * (pitchRateDes - dq[waistPitchJointId])
        + tauPitchFf;
}
```

注意：如果已有全身 IK/PD 正在控制 waist，那么不要重复覆盖。可以使用：

```cpp
tau[waistRollJointId] += tauRollFf;
tau[waistPitchJointId] += tauPitchFf;
```

或者把 MPC torso desired state 合入 IK 的 `q_des`。

---

# Phase 11：增加符号和坐标系测试

必须做以下单元测试或仿真测试。

---

## 11.1 zero torque test

令：

```cpp
tau_phi = 0
tau_theta = 0
```

结果应该退化为普通 SRBD-MPC。

检查：

```text
firstWrenches_
CoM tracking
base rpy tracking
```

应该和原始 SRBD 接近。

---

## 11.2 positive roll torque test

固定 contact wrench 为零或很小，手动设置：

```cpp
tau_phi > 0
tau_theta = 0
```

检查预测中：

```cpp
dot_phi_T increases
omega_x changes in opposite direction
```

也就是：

[
\ddot\phi_T > 0
]

[
\dot\omega_x < 0
]

如果方向反了，说明 `tau_phi` 符号或者 waist roll joint 符号需要翻转。

---

## 11.3 positive pitch torque test

手动设置：

```cpp
tau_phi = 0
tau_theta > 0
```

检查：

[
\ddot\theta_T > 0
]

[
\dot\omega_y < 0
]

如果方向反了，修改：

```cpp
SrpBody
```

或者底层 joint torque sign。

---

## 11.4 torque execution test

检查 MPC 输出：

```cpp
firstTorsoTorqueRp_
```

是否真正进入了：

```cpp
tau[waist_roll_id]
tau[waist_pitch_id]
```

如果 `firstTorsoTorqueRp_` 非零，但 motor torque 没有变化，说明 DataBus 或 low-level mapping 没接上。

---

# Phase 12：推荐调参顺序

不要一次性打开全部效果。按下面顺序调。

---

## 12.1 第一阶段：只打开 torque bounds，不输出到底层

目的：检查 QP 可解性。

```cpp
torsoTorqueMaxRoll = 5.0;
torsoTorqueMaxPitch = 5.0;
```

看：

```text
QP status
firstTorsoTorqueRp_
firstWrenches_
```

是否合理。

---

## 12.2 第二阶段：输出 feedforward torque

打开：

```cpp
tau[waist_roll_id] += tau_phi;
tau[waist_pitch_id] += tau_theta;
```

小 torque limit：

```cpp
torsoTorqueMaxRoll = 5.0;
torsoTorqueMaxPitch = 5.0;
```

观察腰是否抖动。

---

## 12.3 第三阶段：打开 torso desired state

输出：

```cpp
waist_roll_des
waist_pitch_des
waist_roll_vel_des
waist_pitch_vel_des
```

用低增益 PD 跟踪：

```cpp
kpWaistRoll, kpWaistPitch
kdWaistRoll, kdWaistPitch
```

不要一开始给太大。

---

## 12.4 第四阶段：逐渐增加 torque limit

例如：

```cpp
torsoTorqueMaxRoll = 10.0;
torsoTorqueMaxPitch = 15.0;
```

再到：

```cpp
torsoTorqueMaxRoll = 20.0;
torsoTorqueMaxPitch = 30.0;
```

根据 G1 实际 waist actuator 能力设置。

---

# Phase 13：建议日志输出

增加 debug log：

```cpp
robotState.debug_srbd_rp_tau_phi = firstTorsoTorqueRp_(0);
robotState.debug_srbd_rp_tau_theta = firstTorsoTorqueRp_(1);

robotState.debug_srbd_rp_phi_des = firstTorsoRp_(0);
robotState.debug_srbd_rp_theta_des = firstTorsoRp_(1);

robotState.debug_srbd_rp_phid_des = firstTorsoRpRate_(0);
robotState.debug_srbd_rp_thetad_des = firstTorsoRpRate_(1);
```

建议记录：

```text
base pitch
base pitch rate
waist pitch
waist pitch rate
tau_theta
contact pitch moment
CoM x velocity
QP status
```

重点看：

[
\tau_\theta
]

是否在机器人后仰时产生合理方向的反作用。

---

# Phase 14：最终验收标准

改完后，RP-SRBD-MPC 应满足以下标准。

---

## 14.1 动力学结构正确

代码应明确对应：

[
I_{\mathrm{LB,rp}}\dot\omega_B
==============================

## M_{\mathrm{ct}}

S_{rp}^{W}\tau_{rp}
]

[
I_{\mathrm{TR,rp}}\ddot q_{rp}
==============================

\tau_{rp}
]

不是：

[
I_{\mathrm{whole}}\dot\omega_B
==============================

## M_{\mathrm{ct}}

S_{rp}^{W}\tau_{rp}
]

---

## 14.2 QP 约束完整

至少包含：

```text
contact wrench constraints
torso torque bounds
torso angle bounds
torso rate bounds
```

---

## 14.3 输出完整

`solve()` 后应有：

```text
firstWrenches_
firstTorsoTorqueRp_
firstTorsoRp_
firstTorsoRpRate_
```

---

## 14.4 执行闭环一致

底层必须使用：

```text
firstTorsoTorqueRp_
```

最好还使用：

```text
firstTorsoRp_
firstTorsoRpRate_
```

否则 MPC 预测和真实执行不一致。

---

## 14.5 符号正确

正向 waist pitch torque 应该满足：

[
\ddot\theta_T
]

和：

[
\dot\omega_y
]

方向相反。

正向 waist roll torque 应该满足：

[
\ddot\phi_T
]

和：

[
\dot\omega_x
]

方向相反。

---

# 最终给 Codex 的核心任务清单

可以直接让 Codex 按下面 checklist 修改：

```text
1. In G1SrbdMpcRp, add explicit RP-DSRB inertia parameters:
   lowerBodyInertiaRoll, lowerBodyInertiaPitch, wholeBodyInertiaYaw,
   torsoInertiaRoll, torsoInertiaPitch.

2. Replace whole-body angular inertia inverse in buildDiscreteModel() with:
   IangBody = diag(I_LB_x, I_LB_y, I_COM_z)
   IangInvWorld = R * inverse(IangBody) * R.transpose().

3. Use IangInvWorld for both contact angular dynamics and waist reaction torque.

4. Keep torso relative dynamics:
   ddot_phi = tau_phi / torsoInertiaRoll
   ddot_theta = tau_theta / torsoInertiaPitch.

5. Replace hard-coded waist joint indices 13 and 14 with class members:
   waistRollJointId and waistPitchJointId.

6. Verify input.current uses world-frame CoM velocity, not arbitrary dq(0:2).

7. Change torso reference from zero constants to:
   torsoRollRef and torsoPitchRef.

8. Add condensed torso state constraints for:
   phi_T, theta_T, dot_phi_T, dot_theta_T.

9. Add torque-rate cost for the first knot relative to lastTorsoTorqueRp_.

10. After successful solve, recover predicted state:
    Xpred = Aqp * x0 + Bqp * U.

11. Store first predicted torso angle/rate:
    firstTorsoRp_, firstTorsoRpRate_.

12. Write firstTorsoTorqueRp_, firstTorsoRp_, and firstTorsoRpRate_ into DataBus.

13. Modify low-level torque mapping:
    foot wrenches go through J_foot^T;
    tau_phi/tau_theta go directly to waist_roll/waist_pitch torque.

14. If low-level uses IK+PD, also use firstTorsoRp_ and firstTorsoRpRate_
    as waist desired position/rate.

15. Add debug logs for:
    tau_phi, tau_theta, torso angle/rate desired,
    base roll/pitch, base angular velocity, QP status.

16. Add sign tests:
    positive tau_phi should accelerate torso roll and create opposite base roll acceleration;
    positive tau_theta should accelerate torso pitch and create opposite base pitch acceleration.

17. Replace constant contactTable horizon with gait schedule preview if available.
```

---

# 最终结果应该是什么

修改完成后，当前控制器不再只是：

[
\text{SRBD-MPC} + \text{two extra torque variables}
]

而是变成：

[
\boxed{
\text{contact wrench controls whole-body motion}
+
\text{waist roll/pitch torque redistributes angular momentum between torso and lower body}
}
]

也就是：

[
\boxed{
\text{RP-DSRB-MPC}
}
]

它能更合理地处理：

```text
后仰
前倾
roll/pitch 姿态恢复
腰部参与动量调节
减少对 ankle/foot moment 的过度依赖
```

但仍然保持原 SRBD-MPC 的主要优点：

```text
线性动力学
QP 形式
实现简单
计算速度快
可以继续复用现有 contact wrench constraints
```

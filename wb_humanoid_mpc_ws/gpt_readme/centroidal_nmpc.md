你的目标可以按 **“OCS2 如何从代码构建一个 OptimalControlProblem”** 来看。这个仓库里真正构建 MPC 的核心入口是：

```cpp
CentroidalMpcInterface::setupOptimalControlProblem()
```

它把：

```text
dynamics
cost
soft constraints
equality constraints
pre-computation
rollout
initializer
```

全部塞进 OCS2 的 `OptimalControlProblem`。

---

# 1. 总构建链路

构造过程是：

```cpp
CentroidalMpcInterface::CentroidalMpcInterface(...)
```

里面先做基础初始化：

```text
1. 读取 task.info / reference.info / urdf
2. 加载 ddp/mpc/rollout/sqp 配置
3. 创建 PinocchioInterface
4. 创建 CentroidalModelInfo
5. 创建 CentroidalMpcRobotModel<scalar_t>
6. 创建 CentroidalMpcRobotModel<ad_scalar_t>
7. 创建 SwingTrajectoryPlanner
8. 创建 SwitchedModelReferenceManager
9. 读取 initialState
10. 调用 setupOptimalControlProblem()
```

核心代码在：

```cpp
if (setupOCP) {
  setupOptimalControlProblem();
}
```

所以你看懂 MPC 构建，主要看这几个文件：

```text
humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp
humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp
humanoid_centroidal_mpc/src/dynamics/CentroidalDynamicsAD.cpp
humanoid_common_mpc/src/cost/StateInputQuadraticCost.cpp
humanoid_centroidal_mpc/src/cost/ICPCost.cpp
humanoid_centroidal_mpc/src/cost/CentroidalMpcEndEffectorFootCost.cpp
humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp
humanoid_common_mpc/src/constraint/*
humanoid_centroidal_mpc/src/constraint/*
humanoid_common_mpc/src/HumanoidPreComputation.cpp
```

---

# 2. 状态和输入定义

这个仓库的 MPC 状态在 `CentroidalMpcRobotModel.h` 里定义得很清楚。

状态是：

```text
x = [h, q_b, q_j]
```

其中：

```text
h   : 6D normalized centroidal momentum
q_b : floating base pose = [x, y, z, yaw, pitch, roll]
q_j : active joint angles
```

也就是：

```text
x =
[
  h_x, h_y, h_z, h_Lx, h_Ly, h_Lz,
  base_x, base_y, base_z,
  yaw, pitch, roll,
  q_joints...
]
```

输入是：

```text
u = [W_l, W_r, qdot_j]
```

其中：

```text
W_l : left foot wrench  = [fx, fy, fz, mx, my, mz]
W_r : right foot wrench = [fx, fy, fz, mx, my, mz]
qdot_j : active joint velocities
```

所以这个 MPC 不是直接优化关节力矩，而是优化：

```text
足端 wrench + 关节速度
```

后面在 MuJoCo 闭环里，再通过逆动力学和 PD 转成 torque。

---

# 3. `OptimalControlProblem` 是怎么装出来的

核心函数：

```cpp
void CentroidalMpcInterface::setupOptimalControlProblem()
```

里面的结构是：

```cpp
problemPtr_.reset(new OptimalControlProblem);
```

然后依次添加：

```cpp
problemPtr_->dynamicsPtr
problemPtr_->costPtr
problemPtr_->finalCostPtr
problemPtr_->stateSoftConstraintPtr
problemPtr_->softConstraintPtr
problemPtr_->equalityConstraintPtr
problemPtr_->preComputationPtr
rolloutPtr_
initializerPtr_
```

整个构建顺序可以画成：

```text
setupOptimalControlProblem()
│
├── Dynamics
│   └── CentroidalDynamicsAD
│
├── Cost
│   ├── stateInputQuadraticCost
│   ├── terminalCost
│   ├── task-space kinematics costs
│   ├── ICPCost
│   ├── foot tracking costs
│   └── external torque costs
│
├── Soft Constraints
│   ├── jointLimits
│   ├── FootCollisionSoftConstraint
│   ├── frictionForceCone
│   └── contactMomentXY
│
├── Equality Constraints
│   ├── zeroWrench
│   ├── zeroVelocity
│   ├── normalVelocity
│   └── kneeJointMimic
│
├── PreComputation
│   └── HumanoidPreComputation
│
├── Rollout
│   └── TimeTriggeredRollout
│
└── Initializer
    └── CentroidalWeightCompInitializer
```

---

# 4. Dynamics 是怎么构建的

代码：

```cpp
std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
const std::string modelName = "dynamics";
dynamicsPtr.reset(
  new CentroidalDynamicsAD(
    *pinocchioInterfacePtr_,
    centroidalModelInfo_,
    modelName,
    modelSettings_
  )
);

problemPtr_->dynamicsPtr = std::move(dynamicsPtr);
```

也就是 MPC 动力学由：

```cpp
CentroidalDynamicsAD
```

提供。

它内部只包了一层 OCS2 的：

```cpp
PinocchioCentroidalDynamicsAD
```

核心代码：

```cpp
CentroidalDynamicsAD::CentroidalDynamicsAD(...)
    : pinocchioCentroidalDynamicsAd_(
        pinocchioInterface,
        info,
        modelName,
        modelSettings.modelFolderCppAd,
        modelSettings.recompileLibrariesCppAd,
        modelSettings.verboseCppAd
      ) {}
```

然后 OCS2 求解器调用：

```cpp
computeFlowMap()
```

实际返回：

```cpp
pinocchioCentroidalDynamicsAd_.getValue(time, state, input);
```

求线性化时调用：

```cpp
linearApproximation()
```

实际返回：

```cpp
pinocchioCentroidalDynamicsAd_.getLinearApproximation(time, state, input);
```

所以动力学的数学形式是：

```text
xdot = f(x, u)
```

但是代码里没有手写 `df/dx` 和 `df/du`，而是由：

```cpp
PinocchioCentroidalDynamicsAD
```

通过 CppAD 生成。

---

# 5. Dynamics 的自动微分怎么生成

关键点是构造 `PinocchioCentroidalDynamicsAD` 时传入了：

```cpp
modelSettings.modelFolderCppAd
modelSettings.recompileLibrariesCppAd
modelSettings.verboseCppAd
```

也就是：

```cpp
PinocchioCentroidalDynamicsAD(
  pinocchioInterface,
  info,
  modelName,
  modelFolderCppAd,
  recompileLibrariesCppAd,
  verboseCppAd
)
```

这说明它会用 OCS2 的 CppADCodeGen 机制生成动态库。

大致过程是：

```text
1. 用 ad_scalar_t 版本的变量构造 f_ad(x, u)
2. CppAD trace 记录计算图
3. CppADCodeGen 生成 C/C++ 代码
4. 编译成动态库
5. 后续运行时直接调用动态库计算：
   - f(x,u)
   - df/dx
   - df/du
```

所以你在代码里看到：

```cpp
getValue()
getLinearApproximation()
```

但没有看到显式雅可比，是因为雅可比已经由 OCS2 的 AD 模块封装了。

控制开关来自 `task.info` 里的类似字段：

```text
modelFolderCppAd
recompileLibrariesCppAd
verboseCppAd
```

如果：

```text
recompileLibrariesCppAd = true
```

它会重新生成和编译 AD 库。

如果：

```text
recompileLibrariesCppAd = false
```

它会尝试复用已有的动态库。

---

# 6. Cost 是怎么构建的

Cost 分几类。

在 `setupOptimalControlProblem()` 中添加：

```cpp
problemPtr_->costPtr->add("stateInputQuadraticCost", factory.getStateInputQuadraticCost());
problemPtr_->finalCostPtr->add("terminalCost", factory.getTerminalCost());
```

后面还有：

```cpp
addTaskSpaceKinematicsCosts(...)
```

以及：

```cpp
problemPtr_->costPtr->add("icp_Cost", new ICPCost(...));
```

在每个脚循环里又添加：

```cpp
problemPtr_->costPtr->add(footName + "_TaskSpaceKinematicsCost", new CentroidalMpcEndEffectorFootCost(...));

problemPtr_->costPtr->add(footName + "_ExternalTorqueQuadraticCost", factory.getExternalTorqueQuadraticCost(i));
```

所以 running cost 总体是：

```text
L(x,u) =
  state-input tracking cost
+ task-space kinematics cost
+ ICP cost
+ foot tracking cost
+ external torque cost
+ soft constraint penalties
```

terminal cost 是：

```text
Lf(x) = terminal quadratic state cost
```

---

# 7. `stateInputQuadraticCost`

文件：

```text
humanoid_common_mpc/src/cost/StateInputQuadraticCost.cpp
```

构建函数：

```cpp
factory.getStateInputQuadraticCost()
```

读取：

```cpp
Q
R
```

然后创建：

```cpp
StateInputQuadraticCost(Q, R, referenceManager, pinocchioInterface, mpcRobotModel)
```

它继承自：

```cpp
QuadraticStateInputCost
```

核心误差是：

```cpp
return {state - xNominal, input - uNominal};
```

其中：

```cpp
xNominal = referenceManagerPtr_->getDesiredState(targetTrajectories, state, time);
```

输入 nominal 不是 target trajectory 直接给的，而是：

```cpp
uNominal = weightCompensatingInput(pinInterface_, contactFlags, *mpcRobotModelPtr_);
```

也就是说：

```text
状态跟踪：跟踪 referenceManager 给的目标状态
输入跟踪：跟踪重力补偿接触 wrench
```

所以这个 cost 本质是：

```text
0.5 * (x - x_ref)^T Q (x - x_ref)
+ 0.5 * (u - u_gravity)^T R (u - u_gravity)
```

这里 `u_gravity` 会根据当前接触状态分配重力补偿力。

---

# 8. `terminalCost`

构建函数：

```cpp
factory.getTerminalCost()
```

读取：

```cpp
terminalCostScaling
Q_final
```

然后：

```cpp
Qf *= terminalCostScaling;
return new QuadraticStateCost(Qf);
```

所以 terminal cost 是：

```text
0.5 * (x - x_ref)^T Q_final (x - x_ref)
```

不过这里用了 OCS2 的 `QuadraticStateCost`，具体 reference 怎么进来取决于 OCS2 的 target trajectories 机制。

---

# 9. Task-space kinematics cost

函数：

```cpp
CentroidalMpcInterface::addTaskSpaceKinematicsCosts(...)
```

它读取 `task.info` 中：

```text
task_space_costs
```

每个 task-space cost 读取：

```text
link_name
weights
```

然后创建：

```cpp
PinocchioEndEffectorKinematicsCppAd
```

这里已经开始自动微分了：

```cpp
new PinocchioEndEffectorKinematicsCppAd(
  *pinocchioInterfacePtr_,
  pinocchioMappingCppAd,
  {linkName},
  centroidalModelInfo_.stateDim,
  centroidalModelInfo_.inputDim,
  velocityUpdateCallback,
  linkName,
  modelSettings_.modelFolderCppAd,
  modelSettings_.recompileLibrariesCppAd,
  modelSettings_.verboseCppAd
)
```

然后把这个 kinematics 对象放入：

```cpp
EndEffectorKinematicsQuadraticCost
```

这个 cost 用于某些 link 的任务空间跟踪。

这里的 AD 逻辑是：

```text
x,u
  -> generalized coordinates q
  -> generalized velocity v
  -> Pinocchio forward kinematics
  -> link pose / velocity
  -> residual
  -> 自动微分得到 residual 对 x,u 的导数
```

---

# 10. `ICPCost`

文件：

```text
humanoid_centroidal_mpc/src/cost/ICPCost.cpp
```

构建：

```cpp
new ICPCost(
  *referenceManagerPtr_,
  icpWeights,
  *pinocchioInterfacePtr_,
  *mpcRobotModelADPtr_,
  "icp_Cost",
  modelSettings_
)
```

它继承：

```cpp
StateInputCostGaussNewtonAd
```

构造时调用：

```cpp
initialize(
  mpcRobotModelAD.getStateDim(),
  mpcRobotModelAD.getInputDim(),
  2,
  costName,
  modelSettings.modelFolderCppAd,
  modelSettings.recompileLibrariesCppAd
);
```

这就是自动微分注册的位置。

它的 residual 是：

```cpp
ad_vector_t errors = desiredCOMPosition - capturePoint;
return errors.cwiseProduct(sqrtWeightParams);
```

里面：

```cpp
pinocchio::centerOfMass(model, data, q, false);
com = data.com[0].head(2);
```

然后接触点平均位置：

```cpp
desiredCOMPosition = (contactPositions[0] + contactPositions[1]).head(2) / 2.0;
```

当前代码里：

```cpp
capturePoint = com;
```

注意它原本可能想做：

```cpp
capturePoint = com + com_vel / omega;
```

但是这行被注释掉了：

```cpp
// ad_vector_t capturePoint = com + com_vel / ad_scalar_t(omega);
```

所以当前 `ICPCost` 实际更像是：

```text
COM horizontal position should stay near midpoint of two feet
```

而不是严格 ICP：

```text
ICP = COM + COM velocity / omega
```

它的 cost 是 Gauss-Newton 形式：

```text
0.5 * || r(x,u) ||^2
```

其中：

```text
r = sqrt(W_icp) * (foot_midpoint_xy - com_xy)
```

---

# 11. 足端 tracking cost

文件：

```text
humanoid_centroidal_mpc/src/cost/CentroidalMpcEndEffectorFootCost.cpp
```

每个脚都会添加一个：

```cpp
CentroidalMpcEndEffectorFootCost
```

构造时：

```cpp
initialize(
  mpcRobotModelAD.getStateDim(),
  mpcRobotModelAD.getInputDim(),
  25,
  costName,
  modelSettings.modelFolderCppAd,
  modelSettings.recompileLibrariesCppAd
);
```

所以它也是 AD Gauss-Newton cost。

它的 residual 是 12 维：

```cpp
errors << 
  position - reference.getPosition(),
  rotationMatrixDistanceToPlane(orientation, reference.getPlaneNormal()),
  (linearVelocity - reference.getLinearVelocity()) * impactProximityScaler,
  angularVelocity - reference.getAngularVelocity();
```

也就是：

```text
r_foot =
[
  p_foot - p_ref,
  orientation_plane_error,
  v_foot - v_ref,
  omega_foot - omega_ref
]
```

然后乘权重：

```cpp
return errors.cwiseProduct(sqrtWeightParams);
```

不过当前 `getParameters()` 里面 reference 很简单：

```cpp
parameters.head(3) = [0,0,0]          // Reference position
parameters.segment(3,3) = [0,0,1]    // Ground plane normal
parameters.segment(6,3) = [0,0,0]    // Reference linear velocity
parameters.segment(9,3) = [0,0,0]    // Reference angular velocity
parameters.segment(12,12) = sqrtWeights
parameters[24] = impactProximityScaler
```

所以当前代码更多是在约束脚相对地面的高度、姿态和平面速度，而不是显式 tracking 一个复杂足端轨迹的位置。

---

# 12. External torque cost

文件：

```text
humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp
```

每个脚添加一个：

```cpp
factory.getExternalTorqueQuadraticCost(i)
```

它也是：

```cpp
StateInputCostGaussNewtonAd
```

构造时调用：

```cpp
initialize(
  stateDim,
  inputDim,
  n_parameters_,
  endEffectorName + "_ExternalTorqueQuadraticCost",
  modelFolderCppAd,
  recompileLibrariesCppAd,
  verboseCppAd
);
```

它的 residual 计算是：

```cpp
J_ee = computeFrameJacobian(...)
tauExt = J_ee.transpose() * contactWrench
```

然后取 active joints 对应的外力矩：

```cpp
tauExtActive[i] = tauExt[6 + jointIndex]
```

最后：

```cpp
return tauExtActive.cwiseProduct(sqrtWeightsAD) * midSwingScaler;
```

所以这个 cost 是：

```text
惩罚接触 wrench 在关节空间诱导出来的外部关节力矩
```

数学上近似是：

```text
r_tau = sqrt(W_tau) * J_foot(q)^T W_foot
```

然后：

```text
cost = 0.5 * ||r_tau||^2
```

这个 cost 的作用是防止 MPC 通过很大的足端 wrench 产生不合理的关节负担。

---

# 13. Constraints 分成三类

在 `setupOptimalControlProblem()` 里有三种约束容器：

```cpp
problemPtr_->stateSoftConstraintPtr
problemPtr_->softConstraintPtr
problemPtr_->equalityConstraintPtr
```

含义大致是：

```text
stateSoftConstraintPtr:
  只依赖 x 的软约束，以 penalty 形式进 cost

softConstraintPtr:
  依赖 x,u 的软约束，以 penalty 形式进 cost

equalityConstraintPtr:
  等式约束，SQP 直接处理
```

这个仓库里：

```cpp
problemPtr_->stateSoftConstraintPtr->add("jointLimits", ...)
problemPtr_->stateSoftConstraintPtr->add("FootCollisionSoftConstraint", ...)
```

每个脚：

```cpp
problemPtr_->softConstraintPtr->add(footName + "_frictionForceCone", ...)
problemPtr_->softConstraintPtr->add(footName + "_contactMomentXY", ...)
```

每个脚：

```cpp
problemPtr_->equalityConstraintPtr->add(footName + "_zeroWrench", ...)
problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", ...)
problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity", ...)
problemPtr_->equalityConstraintPtr->add(footName + "_kneeJointMimic", ...)
```

---

# 14. Joint limits soft constraint

构建：

```cpp
factory.getJointLimitsConstraint()
```

读取：

```text
jointLimits.mu
jointLimits.delta
```

然后读取 Pinocchio joint limit：

```cpp
readPinocchioJointLimits(...)
```

返回：

```cpp
JointLimitsSoftConstraint
```

它本质是对关节角越界加 barrier penalty。

它不是等式约束，而是：

```cpp
stateSoftConstraintPtr
```

所以求解器不会硬性保证不越界，而是通过 penalty 推回可行域。

---

# 15. Foot collision soft constraint

构建：

```cpp
factory.getFootCollisionConstraint()
```

读取：

```text
collision_constraint.*
```

包括：

```text
leftAnkleFrame
rightAnkleFrame
footCollisionSphereRadius
leftKneeFrame
rightKneeFrame
kneeCollisionSphereRadius
mu
delta
```

它创建：

```cpp
FootCollisionConstraint
```

再包一层：

```cpp
StateSoftConstraint
```

核心作用是：

```text
左脚-右脚
左膝-右膝
脚-对侧膝/踝
```

之间保持最小距离。

这个约束也是 AD 约束，因为 `FootCollisionConstraint` 构造里调用了：

```cpp
initialize(
  stateDim,
  2,
  costName,
  modelFolderCppAd,
  recompileLibrariesCppAd,
  verboseCppAd
)
```

其 residual 是各个碰撞球距离减去最小距离。

---

# 16. Friction cone soft constraint

构建：

```cpp
factory.getFrictionForceConeConstraint(i)
```

读取：

```text
contacts.frictionForceConeSoftConstraint.frictionCoefficient
contacts.frictionForceConeSoftConstraint.mu
contacts.frictionForceConeSoftConstraint.delta
```

它创建：

```cpp
FrictionForceConeConstraint
```

然后包成：

```cpp
StateInputSoftConstraint
```

这个约束只有在对应脚接触时激活：

```cpp
return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
```

它的约束值来自：

```cpp
localForce = t_R_w * forcesInWorldFrame;
return coneConstraint(localForce);
```

它不是 AD，而是手写了一阶、二阶导数：

```cpp
getLinearApproximation()
getQuadraticApproximation()
```

所以 friction cone 是手动微分。

大致形式是：

```text
mu * Fz >= sqrt(Fx^2 + Fy^2)
```

具体代码把它写成一个 cone constraint scalar，然后用 barrier penalty 变成软约束。

---

# 17. Contact moment XY soft constraint

构建：

```cpp
factory.getContactMomentXYConstraint(i, name)
```

它创建：

```cpp
ContactMomentXYConstraintCppAd
```

再包成：

```cpp
StateInputSoftConstraint
```

这个约束只在接触时激活：

```cpp
return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
```

它的 `constraintFunction()` 是：

```cpp
constraintValue << 
  localMoments.x() - y_min * localForce.z(),
 -localMoments.x() + y_max * localForce.z(),
 -localMoments.y() - x_min * localForce.z(),
  localMoments.y() + x_max * localForce.z();
```

这其实是在限制 CoP / foot contact moment 在脚底矩形内。

如果接触点在足底矩形内，需要满足类似：

```text
Mx ∈ [y_min * Fz, y_max * Fz]
My ∈ [-x_max * Fz, -x_min * Fz]
```

这个约束是 AD 的，因为它继承：

```cpp
StateInputConstraintCppAd
```

构造时调用：

```cpp
initialize(
  stateDim,
  inputDim,
  0,
  costName,
  modelFolderCppAd,
  recompileLibrariesCppAd,
  verboseCppAd
)
```

---

# 18. Zero wrench equality constraint

构建：

```cpp
factory.getZeroWrenchConstraint(i)
```

它只在对应脚不接触时激活：

```cpp
return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
```

约束值是：

```cpp
return mpcRobotModelPtr_->getContactWrench(input, contactPointIndex_);
```

所以当脚不接触时，强制：

```text
W_i = 0
```

这是等式约束：

```cpp
equalityConstraintPtr
```

也就是说 swing foot 不能产生接触力。

---

# 19. Zero velocity equality constraint

构建：

```cpp
getStanceFootConstraint(...)
```

内部创建：

```cpp
ZeroVelocityConstraintCppAd
```

它只在对应脚接触时激活：

```cpp
return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
```

它内部包了一层：

```cpp
EndEffectorKinematicsTwistConstraint
```

约束形式大致是：

```text
foot twist = 0
```

也就是 stance foot 不允许滑动、不允许转动。

另外它还在 z 方向加了位置误差反馈：

```cpp
config.Ax(2, 2) = positionErrorGain;
config.b[2] = -config.Ax(2, 2) * footReferenceHeight;
```

所以更准确地说，它不是单纯：

```text
v_foot = 0
```

而是类似：

```text
v_foot_z + Kz * (z_foot - z_ref) = 0
```

以及：

```text
omega_foot + K_ori * orientation_error = 0
```

这样可以让 stance foot 稳定压在参考高度和平面姿态上。

---

# 20. Normal velocity equality constraint

构建：

```cpp
getNormalVelocityConstraint(...)
```

它只在脚不接触时激活：

```cpp
return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
```

内部包：

```cpp
EndEffectorKinematicsLinearVelConstraint
```

它从 `HumanoidPreComputation` 取配置：

```cpp
eeLinearConstraintPtr_->configure(
  humanoidPreComp.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]
);
```

`HumanoidPreComputation` 里根据摆动足轨迹生成：

```cpp
config.b = -swingTrajectoryPlanner->getZvelocityConstraint(footIndex, t)
config.Av = [0, 0, 1]
```

如果有位置误差反馈：

```cpp
config.b -= Kz * swingTrajectoryPlanner->getZpositionConstraint(footIndex, t)
config.Ax = [0, 0, Kz]
```

所以 swing foot 需要满足：

```text
v_foot_z ≈ desired swing z velocity
```

加上 z 方向位置反馈。

这就是 MPC 里让摆动脚抬脚/落脚的关键。

---

# 21. Joint mimic equality constraint

构建：

```cpp
getJointMimicConstraint(i)
```

它读取：

```text
mimicJoints.left_knee.*
mimicJoints.right_knee.*
```

约束值是：

```cpp
posError = multiplier * q_parent - q_child
velError = multiplier * qdot_parent - qdot_child

value = positionGain * posError + velError
```

所以等式约束是：

```text
positionGain * (multiplier * q_parent - q_child)
+ (multiplier * qdot_parent - qdot_child)
= 0
```

它是手写线性化：

```cpp
dfdx(parent) = positionGain * multiplier
dfdx(child)  = -positionGain

dfdu(parent_velocity) = multiplier
dfdu(child_velocity)  = -1
```

这个不是 AD。

---

# 22. PreComputation 的作用

构建：

```cpp
problemPtr_->preComputationPtr.reset(
  new HumanoidPreComputation(
    *pinocchioInterfacePtr_,
    *referenceManagerPtr_->getSwingTrajectoryPlanner(),
    *mpcRobotModelPtr_
  )
);
```

OCS2 在评估 cost/constraint 前会调用：

```cpp
HumanoidPreComputation::request(...)
```

它做两件关键事：

第一，更新 Pinocchio kinematics：

```cpp
updatePinocchioModelKinematics(q)
```

第二，为足端约束准备缓存：

```cpp
eeNormalVelConConfigs_[i]
R_world_to_contacts_[i]
footHeightReferences_[i]
```

尤其是 swing foot normal velocity constraint 依赖：

```text
SwingTrajectoryPlanner 给出的 z position / z velocity reference
```

所以这个模块是：

```text
MPC 当前时刻 t, x, u
    -> 更新 Pinocchio
    -> 根据 gait/swing planner 更新足端约束参数
```

没有它，`ZeroVelocityConstraint` 和 `NormalVelocityConstraint` 就拿不到实时的摆动足高度参考。

---

# 23. Initializer 的作用

构建：

```cpp
initializerPtr_.reset(
  new CentroidalWeightCompInitializer(
    centroidalModelInfo_,
    *referenceManagerPtr_,
    *mpcRobotModelPtr_,
    extendNormalizedMomentum
  )
);
```

它给 SQP/MPC 初始解用。

核心代码：

```cpp
input = weightCompensatingInput(info_, contactFlags, *mpcRobotModelPtr_);
nextState = state;
```

也就是说初始输入不是零，而是根据当前接触状态给一个重力补偿 wrench。

例如双脚接触时：

```text
左脚和右脚分担 mg
```

单脚接触时：

```text
支撑脚承担 mg
```

这样 SQP 初始点更合理，否则 MPC 一开始输入为 0 会导致质心直接自由落体，优化更难收敛。

---

# 24. 自动微分在这个仓库里有四种形式

你看代码时可以按这四类区分。

## 第一类：Dynamics AD

```cpp
CentroidalDynamicsAD
```

内部：

```cpp
PinocchioCentroidalDynamicsAD
```

自动生成：

```text
f(x,u)
df/dx
df/du
```

---

## 第二类：Gauss-Newton cost AD

这些类继承：

```cpp
StateInputCostGaussNewtonAd
```

典型包括：

```text
ICPCost
CentroidalMpcEndEffectorFootCost
ExternalTorqueQuadraticCostAD
```

它们都实现：

```cpp
ad_vector_t costVectorFunction(
  ad_scalar_t time,
  const ad_vector_t& state,
  const ad_vector_t& input,
  const ad_vector_t& parameters
)
```

然后在构造函数里调用：

```cpp
initialize(
  stateDim,
  inputDim,
  parameterDim,
  costName,
  modelFolderCppAd,
  recompileLibrariesCppAd,
  verboseCppAd
)
```

OCS2 会自动得到：

```text
r(x,u,p)
dr/dx
dr/du
```

并用 Gauss-Newton 近似 Hessian：

```text
H ≈ J^T J
```

所以这些 cost 不需要你手写一阶、二阶导数。

---

## 第三类：Constraint AD

这些类继承：

```cpp
StateInputConstraintCppAd
```

典型：

```text
ContactMomentXYConstraintCppAd
```

它实现：

```cpp
ad_vector_t constraintFunction(
  ad_scalar_t time,
  const ad_vector_t& state,
  const ad_vector_t& input,
  const ad_vector_t& parameters
) const
```

构造时调用：

```cpp
initialize(...)
```

OCS2 自动生成：

```text
g(x,u)
dg/dx
dg/du
```

---

## 第四类：Pinocchio kinematics AD

例如：

```cpp
PinocchioEndEffectorKinematicsCppAd
```

在 `addTaskSpaceKinematicsCosts()` 和每个脚的约束中被创建。

它负责生成：

```text
foot position
foot orientation
foot linear velocity
foot angular velocity
```

相对于 `x,u` 的导数。

核心是它需要一个 callback：

```cpp
velocityUpdateCallback
```

代码：

```cpp
auto velocityUpdateCallback =
  [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
    const ad_vector_t q =
      centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
    updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
  };
```

为什么需要它？

因为这个 MPC 的状态不是标准：

```text
[q, v]
```

而是：

```text
[h, q]
```

所以要从 centroidal momentum `h` 和 joint velocity input 恢复 Pinocchio 需要的 generalized velocity `v`。这就需要更新 centroidal dynamics mapping。

---

# 25. 哪些东西不是 AD，而是手写导数

不是所有东西都是 AD。

明显手写导数的包括：

```text
FrictionForceConeConstraint
ZeroWrenchConstraint
JointMimicKinematicConstraint
JointLimitsSoftConstraint
```

例如 friction cone 手写：

```cpp
getLinearApproximation()
getQuadraticApproximation()
```

Joint mimic 也手写：

```cpp
dfdx
dfdu
```

Zero wrench 因为就是从 input 里取 6 维 wrench，所以导数很简单。

---

# 26. 你应该怎么阅读这个 MPC

建议按这个顺序看。

## 第一步：看 MPC 总装配

```text
CentroidalMpcInterface.cpp
```

重点看：

```cpp
setupOptimalControlProblem()
```

把所有 add 的项列出来。

---

## 第二步：看状态/输入映射

```text
CentroidalMpcRobotModel.h
```

重点看：

```cpp
getGeneralizedCoordinates()
getGeneralizedVelocities()
getContactWrench()
getContactForce()
getContactMoment()
getJointAngles()
getJointVelocities()
```

因为所有 cost/constraint 都通过它解释 `x,u`。

---

## 第三步：看 dynamics

```text
CentroidalDynamicsAD.cpp
```

你会发现它只是包 OCS2 的：

```cpp
PinocchioCentroidalDynamicsAD
```

真正复杂的 centroidal dynamics 在 OCS2 库里，不在这个 zip 里。

---

## 第四步：看普通 cost

```text
StateInputQuadraticCost.cpp
```

弄清楚：

```text
x_ref 从 referenceManager 来
u_ref 从 weightCompensatingInput 来
```

---

## 第五步：看 AD cost

```text
ICPCost.cpp
CentroidalMpcEndEffectorFootCost.cpp
ExternalTorqueQuadraticCostAD.cpp
```

重点只看两个函数：

```cpp
initialize(...)
costVectorFunction(...)
```

凡是 `costVectorFunction()` 返回的 residual，就是 OCS2 优化的误差项。

---

## 第六步：看 constraint

```text
FrictionForceConeConstraint.cpp
ContactMomentXYConstraintCppAd.cpp
ZeroWrenchConstraint.cpp
ZeroVelocityConstraintCppAd.cpp
NormalVelocityConstraintCppAd.cpp
JointMimicKinematicConstraint.cpp
```

重点看：

```cpp
isActive()
getValue()
constraintFunction()
getLinearApproximation()
```

尤其要看 `isActive()`，因为接触模式切换时，哪些约束激活完全由它决定。

---

# 27. 最核心的一句话总结

这个仓库的 MPC 构建逻辑是：

```text
用 OCS2 的 OptimalControlProblem 表达一个 centroidal humanoid OCP：
状态 x = [normalized centroidal momentum, base pose, joint angles]
输入 u = [left/right foot wrench, joint velocity]
动力学用 PinocchioCentroidalDynamicsAD 自动微分生成
普通 tracking cost 用 Q/R
复杂 task-space / ICP / torque cost 用 Gauss-Newton CppAD
接触力约束、足端速度约束、mimic 约束根据 gait contact flags 切换激活
最后由 SQP-MPC 在线求解，再通过 inverse dynamics + PD 转成 MuJoCo torque
```

你如果想真正“看懂构建过程”，不要从 MuJoCo 主循环开始，而是先把下面这段代码完全读透：

```cpp
void CentroidalMpcInterface::setupOptimalControlProblem()
```

它就是整个 MPC 问题的总目录。

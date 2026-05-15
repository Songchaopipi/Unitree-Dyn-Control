# humanoid_cd_nmpc_aligator

这个目录实现“路线 A”：保持 OCS2 G1 centroidal MPC 的模型配置、CppADCodeGen 动力学/代价/约束导数，把外层求解器换成 Aligator `SolverProxDDP`。

当前可执行文件：

```bash
cd /home/songchao/OPTControl_env/OpenLoong-Dyn-Control/humanoid_cd_nmpc_aligator
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/proxddp_centroidal_layer1
```

如果这个目录作为独立仓库 clone 到其他位置，需要显式指定依赖所在的 OpenLoong 根目录：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DOPENLOONG_ROOT=/home/songchao/OPTControl_env/OpenLoong-Dyn-Control
```

默认读取：

```text
/home/songchao/OPTControl_env/wb_humanoid_mpc_ws/config/g1_centroidal/mpc/task.info
/home/songchao/OPTControl_env/wb_humanoid_mpc_ws/config/g1_centroidal/command/reference.info
/home/songchao/OPTControl_env/wb_humanoid_mpc_ws/models/g1_description/urdf/g1_29dof.urdf
```

## 问题模式

可执行文件现在支持两种问题：

```bash
./build/proxddp_centroidal_layer1 --problem layer1
./build/proxddp_centroidal_layer1 --problem standard
```

默认是 `standard`。

| 模式 | 内容 |
| --- | --- |
| `layer1` | 最小 centroidal dynamics + Q/R + terminal，用来测纯前端和求解器开销 |
| `standard` | 按 OCS2 `humanoid_centroidal_mpc` 标准装配：task-space、foot、ICP、external torque、zeroWrench、zeroVelocity、normalVelocity、mimic equality、friction cone、contact moment、joint limits、collision soft penalty |

`standard` 模式没有包含 ROS message 相关接口；它在本文件里复刻 `CentroidalMpcInterface::setupOptimalControlProblem()` 的 OCP 装配，避免引入 `humanoid_mpc_msgs`。

## Layer1 对齐了什么

`layer1` 只保留最小问题：

```text
x = [h / m, base pose, joint positions]
u = [left foot wrench, right foot wrench, joint velocities]
x_{k+1} = x_k + dt * f_ocs2_codegen(x_k, u_k)
cost = 0.5 * ||x - x_ref||_Q^2 + 0.5 * ||u - u_ref||_R^2
terminal = 0.5 * ||x_N - x_ref||_{Q_final * terminalCostScaling}^2
```

其中 `f, df/dx, df/du` 来自同一个 OCS2 `PinocchioCentroidalDynamicsAD` CppADCodeGen 对象。程序会同时跑：

```text
Aligator SolverProxDDP
OCS2 SqpSolver
```

Aligator 的 running Q/R 已按 `dt` 缩放，和 OCS2 连续时间 running cost 积分语义对齐；terminal cost 不缩放。

## Aligator frontend 模式

可执行文件支持三种 Aligator 组装路径：

```bash
./build/proxddp_centroidal_layer1 --frontend legacy
./build/proxddp_centroidal_layer1 --frontend direct
./build/proxddp_centroidal_layer1 --frontend thin
./build/proxddp_centroidal_layer1 --rollout nonlinear
```

默认是 `thin + nonlinear rollout`。如果只想复现实验里的线性 forward-pass timing，可以显式传 `--rollout linear`。

| 模式 | 保留 | 绕开 |
| --- | --- | --- |
| `legacy` | 原始 `StageModel + CostStack + QuadraticResidualCost + ODEAbstract + IntegratorEuler` | 无 |
| `direct` | `StageModel + SolverProxDDP` | `CostStack`、`QuadraticResidualCost`、generic `IntegratorEuler` |
| `thin` | `SolverProxDDP`、nonlinear rollout、line search、AL 约束框架 | `CostStack`、`QuadraticResidualCost`、generic `IntegratorEuler`，并让 stage 直接调用 direct dynamics/cost |

`direct/thin` 使用：

```text
DirectCentroidalEulerDynamics:
  xnext = x + dt * f_ocs2_codegen(x, u)
  Jx    = I + dt * dfdx
  Ju    = dt * dfdu

DirectStageCostPackAligatorCost:
  terms = [DirectQuadraticTrackingTerm]
  lx  = Q * (x - x_ref)
  lu  = R * (u - u_ref)
  lxx = Q
  luu = R
```

所以它仍然是完整 ProxDDP，不是只测 Riccati/LQ kernel。

## 严格对比边界

当前 layer1 的数学 OCP 是一致的：

| 项 | 当前状态 |
| --- | --- |
| 状态/输入维度 | 一致，`nx=35, nu=35` |
| robot model / contacts / mass | 一致，来自同一个 OCS2 `ModelSettings + PinocchioInterface + CentroidalModelInfo` |
| 连续动力学 | 一致，共用同一个 `PinocchioCentroidalDynamicsAD` 对象 |
| 连续动力学导数 | 一致，共用 `getLinearApproximation()` |
| 离散化数学形式 | 一致，Euler |
| running cost | 一致，Aligator 侧显式乘 `dt` |
| terminal cost | 一致，不乘 `dt` |
| reference | 一致，`x_ref=x0`, `u_ref=mg/2` 双脚分配 |
| 初始轨迹 | 一致，同一条 Euler rollout |
| 物理约束/软约束 | 一致，都没有加入 |

但实现路径还不是“除求解器外完全相同”：

| 差异 | 影响 |
| --- | --- |
| `legacy` 使用 `StageModel + CostStack + QuadraticResidualCost` | 会多做 residual Jacobian、`J^T W J`、动态矩阵装配 |
| `direct/thin` 使用 direct quadratic cost 和 direct Euler dynamics | 更接近 OCS2 的 LQ/QP 组装方式 |
| OCS2 使用 `OptimalControlProblem + SqpSolver::setupQuadraticSubproblem` | 更直接形成 SQP/HPIPM 需要的 LQ/QP 数据 |
| `legacy` 通过 `ODEAbstract + IntegratorEuler` 包装 codegen dynamics | 数学等价，但多一层 generic integration/transport |
| OCS2 通过 `SensitivityIntegratorType::EULER` 离散化 | 数学等价，但数据直接进入 OCS2 SQP 管线 |
| 收敛准则不同 | 即使 `max-iters` 相同，实际停止逻辑仍不同 |
| timer 范围不同 | `proxddp_derivatives` 是 Aligator 全 stage derivative 管线累计，不是裸 codegen 时间 |

所以当前 benchmark 可以说“数学问题一致，求解器和 frontend/装配路径可控”。`thin` 已经绕过了主要 generic cost/dynamics frontend；如果要做到“除求解器外实现路径也一致”，下一步需要进一步把同一份 OCS2 离散 LQ/QP 数据直接喂给 ProxDDP 的内部工作区，这属于更底层 adapter。

## 用它看 ProxDDP 和 SQP 的区别

在这一层里，导数来源、Q/R、初始轨迹都对齐，所以差异主要来自求解器：

```text
SQP:     每次把整条轨迹展开成一个稀疏 NLP/QP，再由 QP solver 求全局增量。
ProxDDP: 每次沿时间做 Riccati backward pass，利用 shooting 的动态规划结构求局部反馈增量。
```

因此 `derivatives` 更像两边共同要付的线性化成本；`ddp` 是 ProxDDP 的 Riccati/LQ 成本。后续如果加一个 OCS2 SQP 同问题 runner，对应要看的是 OCS2 的 linearization/precomputation 和 QP solve 时间。

## Standard 模式接入内容

`standard` 模式把 OCS2 标准 collection 直接 clone 到 Aligator adapter：

```text
OCS2 StateInputCostCollection  -> DirectStageCostPackAligatorCost
OCS2 StateCostCollection       -> DirectStageCostPackAligatorCost
OCS2 StateInputConstraintCollection -> DirectConstraintPackAligatorFunction + EqualityConstraint
```

因此这些项已经接入：

```text
Layer2B:
  task_space_costs.*_TaskSpaceKinematicsCost
  left/right foot TaskSpaceKinematicsCost
  icp_Cost
  left/right ExternalTorqueQuadraticCost

Layer3:
  zeroWrench
  zeroVelocity
  normalVelocity
  knee mimic equality（task.info 存在 mimicJoints 时）

Layer4:
  frictionForceCone soft constraint
  contactMomentXY soft constraint
  jointLimits state soft constraint
  FootCollisionSoftConstraint
```

soft constraint 没有重新手写 relaxed barrier，而是直接使用 OCS2 factory 返回的 `StateInputSoftConstraint` / `StateSoftConstraint`，所以 penalty、Jacobian、Gauss-Newton/二阶近似语义和 OCS2 标准问题一致。

第一次跑 `--problem standard` 可能会在当前工作目录生成并编译 `cppad_code_gen/`，这是 OCS2 CppAD 缓存，不属于仓库内容，已加入 `.gitignore`。

当前 `standard` SQP runner 使用同一份 OCP 和同一条 primal guess；initializer 仍是简单 `OperatingPoints`，没有接 `CentroidalWeightCompInitializer`。由于 benchmark 显式传入整条初始轨迹，这通常不影响同问题对比。

## 常用参数

```bash
./build/proxddp_centroidal_layer1 \
  --problem standard \
  --horizon 60 \
  --dt 0.02 \
  --max-iters 5 \
  --sqp-iters 5 \
  --max-al-iters 5 \
  --threads 1 \
  --frontend thin \
  --rollout nonlinear \
  --recompile false \
  --solver-verbose false
```

输出里：

```text
model_load    加载 OCS2 model settings、Pinocchio model、CentroidalModelInfo、initialState 的时间
codegen_load  构造/加载 OCS2 CppADCodeGen dynamics 的时间
build_problem 构建 Aligator shooting problem 的时间
raw_codegen_linearization 直接调用 OCS2 getLinearApproximation 跑完整 horizon 的裸时间
solver_setup  Aligator workspace/setup 时间
proxddp_solve       ProxDDP run 总时间
proxddp_derivatives ProxDDP 内部求 stage 导数的时间
proxddp_ddp         ProxDDP 内部 Riccati/DDP backward pass 时间
sqp_total           OCS2 SqpSolver::run 总时间
sqp_lq              OCS2 构建 linear-quadratic approximation 的时间
sqp_qp              OCS2 调 HPIPM 解 QP 的时间
sqp_linesearch      OCS2 SQP line search 时间
```

`layer1` 在 `--rollout linear`、默认 `N=20, dt=0.02`、cached codegen 后的一次参考输出：

```text
legacy:
  proxddp_solve ~= 8.4 ms
  proxddp_derivatives ~= 5.9 ms
  proxddp_ddp ~= 2.2 ms

direct:
  proxddp_solve ~= 4.0 ms
  proxddp_derivatives ~= 1.3 ms
  proxddp_ddp ~= 2.2 ms

thin:
  proxddp_solve ~= 4.0 ms
  proxddp_derivatives ~= 1.3 ms
  proxddp_ddp ~= 2.1 ms

raw_codegen_linearization ~= 0.24 ms
sqp_total ~= 4.0 ms
sqp_lq ~= 0.35 ms
sqp_qp ~= 1.2 ms
```

这里可以看到，旧差距主要来自 `CostStack + QuadraticResidualCost + ODEAbstract + IntegratorEuler` 前端。`direct/thin` 后，ProxDDP 总时间已经和 OCS2 SQP 同一量级；剩下差距主要是 ProxDDP 的 Riccati/DDP pass、线性化重复次数、以及 solver 结构差异。

输出中的 `strict_diff` 用来检查两边结果是否还在解同一个数学问题：

```text
strict_diff cost_abs=... first_u_inf=... next_x_inf=... math_ocp=same aligator_frontend=thin ocs2_frontend=sqp_direct
```

`standard` 的一个缓存后 sanity check（`N=10, max-iters=1, sqp-iters=1`）：

```text
proxddp_solve ~= 4.8 ms
proxddp_derivatives ~= 1.3 ms
proxddp_ddp ~= 0.7 ms
sqp_total ~= 2.6 ms
sqp_lq ~= 1.9 ms
sqp_qp ~= 0.5 ms
strict_diff cost_abs ~= 1e-4
```

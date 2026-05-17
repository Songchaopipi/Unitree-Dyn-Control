# ProxDDP Kino-Dyn NMPC 代码布局

这个目录保存 G1 kino-dyn + inverse dynamics demo 使用的 ProxDDP NMPC 代码。旧路径
`algorithm/g1_kinodynamics_nmpc.*` 只保留兼容入口，真实实现都在这里。

## 文件说明

- `g1_kinodynamics_nmpc.h`  
  对外接口：输入、求解入口、warm start、求解状态和参数。

- `g1_kinodynamics_nmpc.cpp`  
  构造/析构入口，只保留最外层对象生命周期逻辑。

- `g1_kinodynamics_nmpc_warm_start.cpp`  
  状态拼接、接触表、名义控制和 warm start 平移。

- `g1_kinodynamics_nmpc_problem.cpp`  
  Aligator stage、约束、problem 和 solver 的搭建/复用。

- `g1_kinodynamics_nmpc_solve.cpp`  
  `solve()` 主流程、timing 打印和求解结果输出。

- `detail/g1_kino_types.hpp`  
  Aligator/Pinocchio 类型别名。

- `detail/control_slice.hpp/.cpp`  
  控制向量切片 residual，用于 wrench 和关节加速度 box 约束。

- `detail/semi_implicit_kinodynamics.hpp/.cpp`  
  半隐式 Euler kino-dyn 离散动力学及其一阶导数。

- `detail/direct_kino_costs.hpp/.cpp`  
  直接打包的 running/terminal cost，不再走 `CostStack + QuadraticResidualCost`。

- `detail/kino_weights.hpp/.cpp`  
  状态和控制权重构造。

- `detail/mutable_contact_constraints.hpp/.cpp`  
  行走时使用的固定结构接触约束：接触表只更新激活 mask 和 wrench box 上下界，不重建 problem。

- `detail/solver_cache.hpp`  
  solver/problem cache 和结构签名。

- `detail/proxddp_timing.hpp`  
  兼容不同 Aligator 版本的 timing 字段读取。

- `demo/`  
  对应的站立和 ALIP 行走 demo。

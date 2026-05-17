#pragma once

#include <array>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>

class G1KinodynamicsNmpc
{
public:
    static constexpr int kNumFeet = 2;
    static constexpr int kForceSize = 6;

    // 输入说明：一次 NMPC 求解需要的当前状态、目标状态、足端目标和接触模式。
    struct Input
    {
        // 当前广义位置 q。
        Eigen::VectorXd q;
        // 当前广义速度 v。
        Eigen::VectorXd v;
        // 期望广义位置 q_ref。
        Eigen::VectorXd qReference;
        // 期望广义速度 v_ref。
        Eigen::VectorXd vReference;
        // 左右足端在世界系下的位姿参考。
        std::array<pinocchio::SE3, kNumFeet> footPoseReference{
            pinocchio::SE3::Identity(), pinocchio::SE3::Identity()};
        // 预测时域内每个 running knot 的左右足端位姿参考；为空时退回 footPoseReference。
        std::vector<std::array<pinocchio::SE3, kNumFeet>> footPoseReferenceHorizon;
        // 当前时刻左右脚是否接触，用作 contactTable 为空时的默认模式。
        std::array<bool, kNumFeet> contactActive{{true, true}};
        // 预测时域内每个 knot 的接触表，列 0/1 分别为左/右脚。
        Eigen::Matrix<int, Eigen::Dynamic, kNumFeet> contactTable;
        // 左右脚接触 wrench 的名义参考，未接触脚会被内部清零。
        std::array<Eigen::Matrix<double, kForceSize, 1>, kNumFeet> wrenchReference{
            Eigen::Matrix<double, kForceSize, 1>::Zero(),
            Eigen::Matrix<double, kForceSize, 1>::Zero()};
    };

    // 函数说明：保存 Pinocchio 模型、足端 frame id、预测步数和离散时间。
    G1KinodynamicsNmpc(const pinocchio::Model &model,
                       const std::array<pinocchio::FrameIndex, kNumFeet> &footFrameIds,
                       int horizon = 20,
                       double dt = 0.02);
    // 函数说明：释放 ProxDDP problem/solver cache。
    ~G1KinodynamicsNmpc();

    G1KinodynamicsNmpc(const G1KinodynamicsNmpc &) = delete;
    G1KinodynamicsNmpc &operator=(const G1KinodynamicsNmpc &) = delete;

    // 函数说明：用当前输入更新 problem/cache 并调用 Aligator ProxDDP 求解。
    bool solve(const Input &input);
    // 函数说明：清空状态和控制 warm start，通常在接触模式突变或异常后调用。
    void resetWarmStart();

    // 函数说明：读取求解得到的第一个控制 knot 的左右脚 wrench。
    Eigen::Matrix<double, kNumFeet * kForceSize, 1> firstWrenches() const { return firstWrenches_; }
    // 函数说明：读取第一个控制 knot 的关节加速度。
    Eigen::VectorXd firstJointAccelerations() const { return firstJointAccelerations_; }
    // 函数说明：读取第一个预测状态。
    Eigen::VectorXd firstState() const { return firstState_; }
    // 函数说明：读取第一个完整控制向量。
    Eigen::VectorXd firstControl() const { return firstControl_; }
    // 函数说明：返回求解状态，0 表示收敛，1 表示可用但未收敛，负值表示失败。
    int status() const { return status_; }
    // 函数说明：返回最近一次 ProxDDP 迭代次数。
    int iterations() const { return iterations_; }
    // 函数说明：返回最近一次 wrapper 总耗时，单位秒。
    double solveTime() const { return solveTime_; }
    // 函数说明：返回预测时域 knot 数。
    int horizon() const { return horizon_; }
    // 函数说明：返回离散时间间隔。
    double dt() const { return dt_; }
    // 函数说明：返回控制维度。
    int nu() const { return nu_; }
    // 函数说明：返回广义位置维度。
    int nq() const { return model_.nq; }
    // 函数说明：返回广义速度维度。
    int nv() const { return model_.nv; }

    // 参数说明：摩擦系数，用于接触 wrench cone。
    double mu{0.8};
    // 参数说明：足底半长，用于 CoP/yaw wrench cone。
    double footHalfLength{0.10};
    // 参数说明：足底半宽，用于 CoP/yaw wrench cone。
    double footHalfWidth{0.075};
    // 参数说明：单个 wrench force 分量的 box 限幅。
    double forceLimit{1200.0};
    // 参数说明：单个 wrench moment 分量的 box 限幅。
    double momentLimit{60.0};
    // 参数说明：关节加速度 box 限幅。
    double jointAccelerationLimit{20.0};
    // 参数说明：ProxDDP 收敛容差。
    double tolerance{1e-5};
    // 参数说明：AL/Prox 初始惩罚参数。
    double muInit{1e-8};
    // 参数说明：单次 NMPC 调用的 ProxDDP 最大迭代数。
    int maxIterations{5};
    // 参数说明：每次 ProxDDP 的 augmented Lagrangian 最大迭代数。
    int maxAlIterations{2};
    // 参数说明：line search 最小步长，用于限制接触切换时过深回溯。
    double lineSearchAlphaMin{1e-3};
    // 参数说明：line search 最大回溯次数，用于保证实时上界。
    int maxLineSearchSteps{4};
    // 参数说明：stage evaluate/derivatives 的并行线程数；非线性 rollout 和串行 Riccati 不受影响。
    int numThreads{1};
    // 参数说明：line search/forward pass 使用线性化 rollout；速度更快，但会弱化非线性动力学检查。
    bool useLinearRollout{false};
    // 参数说明：使用 Aligator 并行 Riccati/LQ 求解器；要求线性化 rollout。
    bool useParallelLq{false};
    // 参数说明：接触模式变化时清空 dual warm start，避免旧 AL 乘子污染新模式。
    bool resetDualOnContactSwitch{true};
    // 参数说明：是否启用接触 wrench cone 硬约束。
    bool useForceCone{true};
    // 参数说明：是否对支撑脚添加零速度等式约束。
    bool constrainStandingFeet{true};

private:
    pinocchio::Model model_;
    std::array<pinocchio::FrameIndex, kNumFeet> footFrameIds_{};
    int horizon_{20};
    double dt_{0.02};
    int nu_{0};
    int status_{0};
    int iterations_{0};
    double solveTime_{0.0};
    Eigen::Vector3d gravity_{0.0, 0.0, -9.80665};
    Eigen::Matrix<double, kNumFeet * kForceSize, 1> firstWrenches_ =
        Eigen::Matrix<double, kNumFeet * kForceSize, 1>::Zero();
    Eigen::VectorXd firstJointAccelerations_;
    Eigen::VectorXd firstState_;
    Eigen::VectorXd firstControl_;
    std::vector<Eigen::VectorXd> xsWarm_;
    std::vector<Eigen::VectorXd> usWarm_;
    struct SolverCache;
    std::unique_ptr<SolverCache> solverCache_;

    // 函数说明：把输入 q/v 拼成 Aligator 状态向量。
    Eigen::VectorXd makeState(const Input &input) const;
    // 函数说明：把输入 q_ref/v_ref 拼成状态参考。
    Eigen::VectorXd makeStateReference(const Input &input) const;
    // 函数说明：生成第 0 个 knot 的名义控制。
    Eigen::VectorXd nominalControl(const Input &input) const;
    // 函数说明：按指定 knot 的接触模式生成名义控制。
    Eigen::VectorXd nominalControlAt(const Input &input, int knot) const;
    // 函数说明：生成整段 horizon 的名义控制序列。
    std::vector<Eigen::VectorXd> nominalControlHorizon(const Input &input) const;
    // 函数说明：查询某个 knot 的左右脚接触状态。
    std::array<bool, kNumFeet> contactActiveAt(const Input &input, int knot) const;
    // 函数说明：生成用于检测接触切换的模式序列；不参与 problem 结构签名。
    std::vector<std::array<bool, kNumFeet>> contactModeHorizon(const Input &input) const;
    // 函数说明：用当前状态和名义控制初始化 warm start。
    void initializeWarmStart(const Input &input);
    // 函数说明：把上次求解轨迹向前平移一格，并投影接触 wrench。
    void shiftWarmStart(const Input &input);
    // 函数说明：确认 solver cache 里的 problem 和 reference 指针是否有效。
    bool ensureSolverCache(const Input &input,
                           const Eigen::VectorXd &x0,
                           const Eigen::VectorXd &xRef,
                           const Eigen::VectorXd &uRef,
                           double &solverSetupTime,
                           bool &rebuilt);
    // 函数说明：检查内部 problem/reference 是否已经建好。
    bool collectSolverCacheReferences();
    // 函数说明：problem 结构不变时，只更新初始状态和 cost reference。
    void updateCachedProblemReferences(const Input &input,
                                       const Eigen::VectorXd &x0,
                                       const Eigen::VectorXd &xRef,
                                       const Eigen::VectorXd &uRef);
    // 函数说明：从 solver 结果中取第一个控制和下一状态，并刷新 warm start。
    void updateOutputs(const std::vector<Eigen::VectorXd> &xs,
                       const std::vector<Eigen::VectorXd> &us);
};

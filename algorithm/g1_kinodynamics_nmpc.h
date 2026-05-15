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

    struct Input
    {
        Eigen::VectorXd q;
        Eigen::VectorXd v;
        Eigen::VectorXd qReference;
        Eigen::VectorXd vReference;
        std::array<pinocchio::SE3, kNumFeet> footPoseReference{
            pinocchio::SE3::Identity(), pinocchio::SE3::Identity()};
        std::array<bool, kNumFeet> contactActive{{true, true}};
        std::array<Eigen::Matrix<double, kForceSize, 1>, kNumFeet> wrenchReference{
            Eigen::Matrix<double, kForceSize, 1>::Zero(),
            Eigen::Matrix<double, kForceSize, 1>::Zero()};
    };

    G1KinodynamicsNmpc(const pinocchio::Model &model,
                       const std::array<pinocchio::FrameIndex, kNumFeet> &footFrameIds,
                       int horizon = 20,
                       double dt = 0.02);
    ~G1KinodynamicsNmpc();

    G1KinodynamicsNmpc(const G1KinodynamicsNmpc &) = delete;
    G1KinodynamicsNmpc &operator=(const G1KinodynamicsNmpc &) = delete;

    bool solve(const Input &input);

    Eigen::Matrix<double, kNumFeet * kForceSize, 1> firstWrenches() const { return firstWrenches_; }
    Eigen::VectorXd firstJointAccelerations() const { return firstJointAccelerations_; }
    Eigen::VectorXd firstState() const { return firstState_; }
    Eigen::VectorXd firstControl() const { return firstControl_; }
    int status() const { return status_; }
    int iterations() const { return iterations_; }
    double solveTime() const { return solveTime_; }
    int horizon() const { return horizon_; }
    double dt() const { return dt_; }
    int nu() const { return nu_; }
    int nq() const { return model_.nq; }
    int nv() const { return model_.nv; }

    double mu{0.8};
    double footHalfLength{0.10};
    double footHalfWidth{0.075};
    double forceLimit{1200.0};
    double momentLimit{60.0};
    double jointAccelerationLimit{20.0};
    double tolerance{1e-5};
    double muInit{1e-8};
    int maxIterations{5};
    int maxAlIterations{2};
    bool useForceCone{true};
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

    Eigen::VectorXd makeState(const Input &input) const;
    Eigen::VectorXd makeStateReference(const Input &input) const;
    Eigen::VectorXd nominalControl(const Input &input) const;
    void initializeWarmStart(const Input &input);
    void shiftWarmStart(const Input &input);
    bool ensureSolverCache(const Input &input,
                           const Eigen::VectorXd &x0,
                           const Eigen::VectorXd &xRef,
                           const Eigen::VectorXd &uRef,
                           double &solverSetupTime,
                           bool &rebuilt);
    bool collectSolverCacheReferences();
    void updateCachedProblemReferences(const Input &input,
                                       const Eigen::VectorXd &x0,
                                       const Eigen::VectorXd &xRef,
                                       const Eigen::VectorXd &uRef);
    void updateOutputs(const std::vector<Eigen::VectorXd> &xs,
                       const std::vector<Eigen::VectorXd> &us);
};

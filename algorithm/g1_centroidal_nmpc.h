#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>

#include "data_bus.h"

class G1CentroidalNmpc
{
public:
    static constexpr int kNx = 9;
    static constexpr int kForceSize = 6;
    static constexpr int kNumFeet = 2;
    static constexpr int kNu = kNumFeet * kForceSize;

    struct Input
    {
        Eigen::Matrix<double, kNx, 1> current = Eigen::Matrix<double, kNx, 1>::Zero();
        Eigen::Vector3d comReference = Eigen::Vector3d::Zero();
        Eigen::Vector3d comVelocityReference = Eigen::Vector3d::Zero();
        Eigen::Vector3d comAccelerationReference = Eigen::Vector3d::Zero();
        std::array<Eigen::Vector3d, kNumFeet> contactPositionWorld{};
        std::array<bool, kNumFeet> contactActive{{true, true}};
        Eigen::Matrix<double, kNx, Eigen::Dynamic> reference;
        Eigen::Matrix<int, Eigen::Dynamic, kNumFeet> contactTable;
        std::vector<std::array<Eigen::Vector3d, kNumFeet>> contactPositionWorldHorizon;
        double mass{1.0};
    };

    explicit G1CentroidalNmpc(int horizon = 20, double dt = 0.02);

    bool solve(const Input &input);
    bool solveFromDataBus(const DataBus &robotState, double mass);
    void dataBusWrite(DataBus &robotState) const;

    Eigen::Matrix<double, kNu, 1> firstContactForces() const { return firstContactForces_; }
    Eigen::Matrix<double, 12, 1> firstPlaneForces() const { return firstPlaneForces_; }
    Eigen::Matrix<double, 12, 1> firstWrenches() const { return firstWrenches_; }
    Eigen::Vector3d firstComPosition() const { return firstComPosition_; }
    Eigen::Vector3d firstComVelocity() const { return firstComVelocity_; }
    Eigen::Vector3d firstComAcceleration() const { return firstComAcceleration_; }
    int status() const { return status_; }
    int iterations() const { return iterations_; }
    double solveTime() const { return solveTime_; }
    int horizon() const { return horizon_; }
    double dt() const { return dt_; }

    double mu{1.0};
    double nominalForceWeight{1e-2};
    int maxIterations{30};
    int maxAlIterations{2};
    double tolerance{1e-5};
    double muInit{1e-8};
    bool logSolve{false};
    bool logSolverIterations{false};
    double fzLow{20.0};
    double fMax{1400.0};
    double singleSupportTangentialForceMax{6.0};
    double singleSupportFzMinSafetyFactor{0.92};
    double singleSupportFzSafetyFactor{1.18};
    double singleSupportRollMomentMax{0.5};
    double singleSupportPitchMomentMax{1.0};
    double singleSupportYawMomentMax{0.3};
    double footFront{0.12};
    double footBack{-0.05};
    double footHalfWidth{0.025};
    double yawFriction{0.046};

private:
    int horizon_{20};
    double dt_{0.02};
    int status_{0};
    int iterations_{0};
    double solveTime_{0.0};
    Eigen::Matrix<double, kNu, 1> firstContactForces_ = Eigen::Matrix<double, kNu, 1>::Zero();
    Eigen::Matrix<double, 12, 1> firstPlaneForces_ = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> firstWrenches_ = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Vector3d firstComPosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d firstComVelocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d firstComAcceleration_ = Eigen::Vector3d::Zero();
    std::vector<Eigen::Matrix<double, kNx, 1>> xsWarm_;
    std::vector<Eigen::Matrix<double, kNu, 1>> usWarm_;

    Input buildInputFromDataBus(const DataBus &robotState, double mass) const;
    Eigen::Matrix<double, kNu, 1> nominalControl(const Input &input) const;
    Eigen::Matrix<double, kNu, 1> nominalControlAt(const Input &input, int knot) const;
    std::array<bool, kNumFeet> contactActiveAt(const Input &input, int knot) const;
    std::array<Eigen::Vector3d, kNumFeet> contactPositionsAt(const Input &input, int knot) const;
    Eigen::Matrix<double, kNx, 1> referenceAt(const Input &input, int knot) const;
    void initializeWarmStart(const Input &input);
    void shiftWarmStart(const Input &input);
    void updateOutputs(const Input &input,
                       const std::vector<Eigen::VectorXd> &xs,
                       const std::vector<Eigen::VectorXd> &us);
};

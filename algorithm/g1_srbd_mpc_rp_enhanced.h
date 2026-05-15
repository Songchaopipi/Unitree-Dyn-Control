#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <vector>

#include "qpOASES.hpp"
#include "data_bus.h"

class G1SrbdMpcRpEnhanced
{
public:
    static constexpr int kNx = 17;
    static constexpr int kNu = 14;
    static constexpr int kNuContact = 12;
    static constexpr int kNuTorso = 2;

    static constexpr int kIdxRpy = 0;
    static constexpr int kIdxPos = 3;
    static constexpr int kIdxOmega = 6;
    static constexpr int kIdxVel = 9;
    static constexpr int kIdxGravity = 12;
    static constexpr int kIdxTorsoRp = 13;
    static constexpr int kIdxTorsoRpRate = 15;
    static constexpr int kIdxTorsoInput = 12;

    struct Input
    {
        Eigen::Matrix<double, kNx, 1> current = Eigen::Matrix<double, kNx, 1>::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        std::array<Eigen::Vector3d, 2> contactPositionWorld{};
        std::vector<std::array<Eigen::Vector3d, 2>> contactPositionWorldHorizon;
        Eigen::Matrix<double, kNx, Eigen::Dynamic> reference;
        Eigen::Matrix<int, Eigen::Dynamic, 2> contactTable;
        double mass{1.0};
        Eigen::Matrix3d inertiaBody = Eigen::Matrix3d::Identity();
    };

    explicit G1SrbdMpcRpEnhanced(int horizon = 10, double dt = 0.04);

    bool solve(const Input &input);
    bool solveFromDataBus(const DataBus &robotState, double mass, const Eigen::Matrix3d &inertiaBody);
    void dataBusWrite(DataBus &robotState) const;

    Eigen::Matrix<double, 12, 1> firstPlaneForces() const { return firstPlaneForces_; }
    Eigen::Matrix<double, 12, 1> firstWrenches() const { return firstWrenches_; }
    Eigen::Vector2d firstTorsoTorqueRp() const { return firstTorsoTorqueRp_; }
    Eigen::Vector2d firstTorsoRp() const { return firstTorsoRp_; }
    Eigen::Vector2d firstTorsoRpRate() const { return firstTorsoRpRate_; }
    int status() const { return qpStatus_; }
    int horizon() const { return horizon_; }
    double dt() const { return dt_; }

    double mu{1.0};
    double fMax{1400.0};
    double fzLow{20.0};
    double footFront{0.1};
    double footBack{-0.04};
    double footHalfWidth{0.025};
    double yawFriction{0.046};
    double nominalForceWeight{1.0};

    // RP-DSRB inertia parameters. Roll/pitch use lower-body inertia; yaw uses
    // whole-body centroidal inertia.
    double lowerBodyInertiaRoll{1.0};
    double lowerBodyInertiaPitch{1.0};
    double wholeBodyInertiaYaw{1.0};
    double torsoInertiaRoll{1.0};
    double torsoInertiaPitch{1.0};
    double torsoTorqueMaxRoll{20.0};
    double torsoTorqueMaxPitch{20.0};
    double torsoRollMax{0.35};
    double torsoPitchMax{0.35};
    double torsoRollRateMax{2.0};
    double torsoPitchRateMax{2.0};
    double torsoRollRef{0.0};
    double torsoPitchRef{0.0};
    double torsoTorqueRateWeight{1e-3};
    int waistRollJointId{13};
    int waistPitchJointId{14};

private:
    int horizon_{10};
    double dt_{0.04};
    int qpStatus_{0};
    Eigen::Matrix<double, 12, 1> firstPlaneForces_ = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> firstWrenches_ = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Vector2d firstTorsoTorqueRp_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d firstTorsoRp_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d firstTorsoRpRate_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d lastTorsoTorqueRp_ = Eigen::Vector2d::Zero();

    Input buildInputFromDataBus(const DataBus &robotState, double mass, const Eigen::Matrix3d &inertiaBody) const;
    void buildDiscreteModel(const Input &input,
                            int knot,
                            Eigen::Matrix<double, kNx, kNx> &A,
                            Eigen::Matrix<double, kNx, kNu> &B) const;
    void copyEigenToReal(qpOASES::real_t *target, const Eigen::MatrixXd &source) const;
    void copyEigenToReal(qpOASES::real_t *target, const Eigen::VectorXd &source) const;
};

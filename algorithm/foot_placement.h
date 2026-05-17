/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#pragma once

#include <Eigen/Dense>
#include "data_bus.h"

class FootPlacement
{
public:
    double kp_vx{0}, kp_vy{0}, kp_wz{0};
    double legLength{1};
    double stepHeight{0.1};
    double robotMass{1.0};
    bool useAngularMomentumState{true};
    double dt{0.001};
    double phi{0};      // phase varialbe for trajectory generation, must between 0 and 1
    double tSwing{0.4}; // swing time
    Eigen::Vector3d posStart_W, posDes_W, hipPos_W, STPos_W;
    Eigen::Vector3d desV_W, curV_W;
    Eigen::Vector3d swingVelDes_W{Eigen::Vector3d::Zero()};
    Eigen::Vector3d swingAccDes_W{Eigen::Vector3d::Zero()};
    bool hasLastPlan{false};
    Eigen::Vector2d lastHlipStepLocal{Eigen::Vector2d::Zero()};
    Eigen::Vector2d lastPlannedStepBaseYaw{Eigen::Vector2d::Zero()};
    Eigen::Vector3d lastPlannedStepWorld{Eigen::Vector3d::Zero()};
    Eigen::Vector3d lastPlannedTouchdownWorld{Eigen::Vector3d::Zero()};
    Eigen::VectorXd yd{Eigen::VectorXd::Zero(10)};
    Eigen::VectorXd dyd{Eigen::VectorXd::Zero(10)};
    Eigen::VectorXd d2yd{Eigen::VectorXd::Zero(10)};
    double desWz_W;
    Eigen::Vector3d base_pos;
    Eigen::Vector3d com_pos_W, com_vel_W;
    Eigen::Vector3d base_rpy;
    Eigen::VectorXd centroidalMomentum_W;
    double Trajectory(double phase, double des1, double des2);
    double Bezier(const std::vector<double> &coeff, double s) const;
    double BezierD1(const std::vector<double> &coeff, double s) const;
    double BezierD2(const std::vector<double> &coeff, double s) const;
    Eigen::Vector2d PlanHlipFootstep(const Eigen::Vector4d &reducedState,
                                     double tRemain,
                                     double zNom,
                                     double singleSupportTime,
                                     double doubleSupportTime,
                                     double stepWidthNominal) const;
    void getSwingPos();
    void dataBusRead(DataBus &robotState);
    void dataBusWrite(DataBus &robotState);
    DataBus::LegState legState;
    DataBus::LegState stanceLeg{DataBus::LSt};

private:
    double pDesCur[3]{0};
    double yawCur;
    double targetYaw{0.0};
    double targetYawRate{0.0};
    bool targetYawInitialized{false};
    bool referenceInitialized{false};
    double theta0;
    double doubleSupportTime{0.0};
    double omegaZ_W;
    double hip_width;
    bool stretchLeg{false};
    double zStretch{0};
    bool finish_Stretch;
    double lastFinalTargetY{0.0};
    Eigen::Vector3d lockedFinalTarget_W{Eigen::Vector3d::Zero()};
    Eigen::VectorXd referenceAtStepStart{Eigen::VectorXd::Zero(10)};
    Eigen::VectorXd comHeightCurve{Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd swingXCurve{Eigen::VectorXd::Zero(5)};
    Eigen::VectorXd swingYCurve{Eigen::VectorXd::Zero(5)};
    Eigen::VectorXd swingZCurve{Eigen::VectorXd::Zero(9)};
    Eigen::VectorXd stanceHipYawCurve{Eigen::VectorXd::Zero(5)};
    Eigen::VectorXd swingHipYawCurve{Eigen::VectorXd::Zero(5)};
    Eigen::Vector2d pivotMomentumFiltered{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d pivotMomentumP{Eigen::Matrix2d::Identity()};
    bool pivotMomentumInitialized{false};
    DataBus::LegState lastLegState{DataBus::DSt};
};

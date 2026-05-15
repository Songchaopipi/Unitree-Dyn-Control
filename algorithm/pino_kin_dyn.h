/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "data_bus.h"
#include <string>
#include "json/json.h"
#include <vector>

namespace g1_kin_dyn
{
constexpr double kFootContactFrontX = 0.12;
constexpr double kFootContactBackX = -0.05;
constexpr double kFootContactHalfWidth = 0.025;
constexpr double kBaseFrontX = 0.05;
constexpr double kBaseBackX = -0.05;
constexpr double kBaseLeftY = 0.05;
constexpr double kBaseRightY = -0.05;

struct Kin1D
{
    double position{0.0};
    double velocity{0.0};
    double dJdq{0.0};
    Eigen::RowVectorXd jacobian;
};

Eigen::Matrix3d yawRotationFromRotation(const Eigen::Matrix3d &rot_W);

Eigen::MatrixXd pointJacobianLocal(const Eigen::MatrixXd &J6,
                                   const Eigen::Matrix3d &frameRot_W,
                                   const Eigen::Vector3d &pointInFrame);

Eigen::Vector3d pointDjdqLocal(const Eigen::MatrixXd &J6,
                               const Eigen::MatrixXd &dJ6,
                               const Eigen::Matrix3d &frameRot_W,
                               const Eigen::Vector3d &pointInFrame,
                               const Eigen::VectorXd &dq);

Kin1D frameYawKinematics(const Eigen::Matrix3d &rot_W,
                         const Eigen::MatrixXd &J6,
                         const Eigen::MatrixXd &dJ6,
                         const Eigen::VectorXd &dq);

Kin1D footDeltaPitchKinematics(const Eigen::MatrixXd &J6,
                               const Eigen::MatrixXd &dJ6,
                               const Eigen::Matrix3d &footRot_W,
                               const Eigen::VectorXd &dq);

Kin1D footDeltaRollKinematics(const Eigen::MatrixXd &J6,
                              const Eigen::MatrixXd &dJ6,
                              const Eigen::Matrix3d &footRot_W,
                              const Eigen::VectorXd &dq);

Kin1D baseDeltaPitchKinematics(const Eigen::MatrixXd &Jbase,
                               const Eigen::MatrixXd &dJbase,
                               const Eigen::Matrix3d &baseRot_W,
                               const Eigen::VectorXd &dq);

Kin1D baseDeltaRollKinematics(const Eigen::MatrixXd &Jbase,
                              const Eigen::MatrixXd &dJbase,
                              const Eigen::Matrix3d &baseRot_W,
                              const Eigen::VectorXd &dq);

Eigen::Matrix<double, 6, 6> planeForceToWrenchMap();

Eigen::Matrix<double, 6, 1> distributeVerticalFootLoad(double fz);

Eigen::Matrix<double, 11, 6> roMoCoPlaneFootCone(double mu,
                                                 double footHalfWidth,
                                                 double footFront,
                                                 double footBack,
                                                 double yawFriction);

double motorTorqueLimit(int motorId);
} // namespace g1_kin_dyn

class Pin_KinDyn
{
public:
    std::vector<bool> motorReachLimit;
    const std::vector<std::string> motorName = {
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"}; // G1 joint order follows RoMoCo all_encoder_names.
    Eigen::VectorXd motorMaxTorque;
    Eigen::VectorXd motorMaxPos;
    Eigen::VectorXd motorMinPos;

    Eigen::VectorXd tauJointOld;
    std::string urdf_path;
    pinocchio::Model model_biped;
    pinocchio::Model model_biped_fixed;
    int model_nv;
    pinocchio::JointIndex r_ankle_joint, l_ankle_joint, base_joint, r_hip_joint, l_hip_joint, r_hip_roll_joint, l_hip_roll_joint, waist_yaw_joint;
    pinocchio::FrameIndex r_foot_frame, l_foot_frame, r_foot_frame_fixed, l_foot_frame_fixed;
    pinocchio::FrameIndex l_foot_lf_frame, l_foot_rf_frame, l_foot_lb_frame, l_foot_rb_frame;
    pinocchio::FrameIndex r_foot_lf_frame, r_foot_rf_frame, r_foot_lb_frame, r_foot_rb_frame;
    pinocchio::JointIndex r_ankle_joint_fixed, l_ankle_joint_fixed, r_hip_joint_fixed, l_hip_joint_fixed;
    pinocchio::JointIndex r_hand_joint, l_hand_joint, r_hand_joint_fixed, l_hand_joint_fixed;
    Eigen::VectorXd q, dq, ddq;
    Eigen::Matrix3d Rcur;
    Eigen::Quaternion<double> quatCur;
    Eigen::Matrix<double, 6, -1> J_r, J_l, J_hd_r, J_hd_l, J_base, J_hip_link, J_r_body, J_l_body;
    Eigen::Matrix<double, 6, -1> dJ_r, dJ_l, dJ_hd_r, dJ_hd_l, dJ_base, dJ_hip_link;
    Eigen::MatrixXd J_l_foot, J_r_foot;
    Eigen::VectorXd dJ_l_foot, dJ_r_foot;
    Eigen::Matrix<double, 3, -1> Jcom;
    Eigen::Vector3d dJcom;
    Eigen::Vector3d fe_r_pos, fe_l_pos, base_pos; // foot-end position in world frame
    Eigen::Vector3d fe_r_pos_body, fe_l_pos_body; // foot-end position in body frame
    Eigen::Vector3d hd_r_pos, hd_l_pos;           // hand position in world frame
    Eigen::Vector3d fe_r_vel_body, fe_l_vel_body; // foot-end velcity in body frame
    Eigen::Vector3d hd_r_pos_body, hd_l_pos_body; // hand position in body frame
    Eigen::Vector3d hip_r_pos, hip_l_pos, hip_link_pos;
    Eigen::Vector3d hip_r_pos_body, hip_l_pos_body;
    Eigen::Matrix3d hip_link_rot;
    Eigen::Matrix3d fe_r_rot, fe_l_rot, base_rot;
    Eigen::Matrix3d fe_r_rot_body, fe_l_rot_body;
    Eigen::Matrix3d hd_r_rot, hd_l_rot;
    Eigen::Matrix3d hd_r_rot_body, hd_l_rot_body;
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_G, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non;
    Eigen::Vector3d CoM_pos;
    Eigen::Matrix3d inertia;
    enum legIdx
    {
        left,
        right
    };
    struct IkRes
    {
        int status;
        int itr;
        Eigen::VectorXd err;
        Eigen::VectorXd jointPosRes;
    };
    struct CompositeInertia
    {
        double mass{0.0};
        Eigen::Matrix3d inertiaAboutReferenceWorld{Eigen::Matrix3d::Zero()};
    };

    Pin_KinDyn(std::string urdf_pathIn);
    void dataBusRead(DataBus const &robotState);
    void dataBusWrite(DataBus &robotState);
    void computeJ_dJ();
    void computeDyn();
    CompositeInertia computeLowerBodyInertia(const Eigen::Vector3d &referenceComWorld);
    IkRes computeInK_Leg(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R, const Eigen::Vector3d &Pdes_R);
    IkRes computeInK_Hand(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R, const Eigen::Vector3d &Pdes_R);
    Eigen::VectorXd integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI);
    static Eigen::Quaterniond intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w);
    void workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT);

private:
    pinocchio::Data data_biped, data_biped_fixed;
};

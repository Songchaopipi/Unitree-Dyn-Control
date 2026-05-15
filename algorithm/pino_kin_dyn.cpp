/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "pino_kin_dyn.h"

#include "useful_math.h"

#include <cmath>
#include <utility>

namespace g1_kin_dyn
{
Eigen::Matrix3d yawRotationFromRotation(const Eigen::Matrix3d &rot_W)
{
    return Rz3(std::atan2(rot_W(1, 0), rot_W(0, 0)));
}

Eigen::MatrixXd pointJacobianLocal(const Eigen::MatrixXd &J6,
                                   const Eigen::Matrix3d &frameRot_W,
                                   const Eigen::Vector3d &pointInFrame)
{
    const Eigen::Vector3d pointOffset_W = frameRot_W * pointInFrame;
    const Eigen::MatrixXd Jp_W = J6.topRows(3) - skewSymmetric(pointOffset_W) * J6.bottomRows(3);
    return yawRotationFromRotation(frameRot_W).transpose() * Jp_W;
}

Eigen::Vector3d pointDjdqLocal(const Eigen::MatrixXd &J6,
                               const Eigen::MatrixXd &dJ6,
                               const Eigen::Matrix3d &frameRot_W,
                               const Eigen::Vector3d &pointInFrame,
                               const Eigen::VectorXd &dq)
{
    const Eigen::Vector3d pointOffset_W = frameRot_W * pointInFrame;
    const Eigen::Vector3d omega_W = J6.bottomRows(3) * dq;
    const Eigen::Vector3d pointOffsetDot_W = omega_W.cross(pointOffset_W);
    const Eigen::Vector3d dJpDq_W =
        dJ6.topRows(3) * dq -
        skewSymmetric(pointOffsetDot_W) * (J6.bottomRows(3) * dq) -
        skewSymmetric(pointOffset_W) * (dJ6.bottomRows(3) * dq);
    return yawRotationFromRotation(frameRot_W).transpose() * dJpDq_W;
}

Kin1D frameYawKinematics(const Eigen::Matrix3d &rot_W,
                         const Eigen::MatrixXd &J6,
                         const Eigen::MatrixXd &dJ6,
                         const Eigen::VectorXd &dq)
{
    Kin1D out;
    out.position = std::atan2(rot_W(1, 0), rot_W(0, 0));
    out.velocity = (J6.row(5) * dq)(0);
    out.dJdq = (dJ6.row(5) * dq)(0);
    out.jacobian = J6.row(5);
    return out;
}

Kin1D footDeltaPitchKinematics(const Eigen::MatrixXd &J6,
                               const Eigen::MatrixXd &dJ6,
                               const Eigen::Matrix3d &footRot_W,
                               const Eigen::VectorXd &dq)
{
    Kin1D out;
    const Eigen::MatrixXd J_lf = pointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactFrontX, kFootContactHalfWidth, 0.0));
    const Eigen::MatrixXd J_rf = pointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactFrontX, -kFootContactHalfWidth, 0.0));
    const Eigen::MatrixXd J_lb = pointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactBackX, kFootContactHalfWidth, 0.0));
    const Eigen::MatrixXd J_rb = pointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactBackX, -kFootContactHalfWidth, 0.0));
    const Eigen::Vector3d dJdq_lf = pointDjdqLocal(J6, dJ6, footRot_W, Eigen::Vector3d(kFootContactFrontX, kFootContactHalfWidth, 0.0), dq);
    const Eigen::Vector3d dJdq_rf = pointDjdqLocal(J6, dJ6, footRot_W, Eigen::Vector3d(kFootContactFrontX, -kFootContactHalfWidth, 0.0), dq);
    const Eigen::Vector3d dJdq_lb = pointDjdqLocal(J6, dJ6, footRot_W, Eigen::Vector3d(kFootContactBackX, kFootContactHalfWidth, 0.0), dq);
    const Eigen::Vector3d dJdq_rb = pointDjdqLocal(J6, dJ6, footRot_W, Eigen::Vector3d(kFootContactBackX, -kFootContactHalfWidth, 0.0), dq);
    const Eigen::Vector3d p_lf = footRot_W * Eigen::Vector3d(kFootContactFrontX, kFootContactHalfWidth, 0.0);
    const Eigen::Vector3d p_rf = footRot_W * Eigen::Vector3d(kFootContactFrontX, -kFootContactHalfWidth, 0.0);
    const Eigen::Vector3d p_lb = footRot_W * Eigen::Vector3d(kFootContactBackX, kFootContactHalfWidth, 0.0);
    const Eigen::Vector3d p_rb = footRot_W * Eigen::Vector3d(kFootContactBackX, -kFootContactHalfWidth, 0.0);
    const double length = kFootContactFrontX - kFootContactBackX;
    out.jacobian = ((J_lb.row(2) + J_rb.row(2)) * 0.5 - (J_lf.row(2) + J_rf.row(2)) * 0.5) / length;
    out.position = ((p_lb.z() + p_rb.z()) * 0.5 - (p_lf.z() + p_rf.z()) * 0.5) / length;
    out.velocity = (out.jacobian * dq)(0);
    out.dJdq = (((dJdq_lb.z() + dJdq_rb.z()) * 0.5 - (dJdq_lf.z() + dJdq_rf.z()) * 0.5) / length);
    return out;
}

Kin1D footDeltaRollKinematics(const Eigen::MatrixXd &J6,
                              const Eigen::MatrixXd &dJ6,
                              const Eigen::Matrix3d &footRot_W,
                              const Eigen::VectorXd &dq)
{
    Kin1D out;
    const Eigen::MatrixXd J_lf = pointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactFrontX, kFootContactHalfWidth, 0.0));
    const Eigen::MatrixXd J_rf = pointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactFrontX, -kFootContactHalfWidth, 0.0));
    const Eigen::MatrixXd J_lb = pointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactBackX, kFootContactHalfWidth, 0.0));
    const Eigen::MatrixXd J_rb = pointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactBackX, -kFootContactHalfWidth, 0.0));
    const Eigen::Vector3d dJdq_lf = pointDjdqLocal(J6, dJ6, footRot_W, Eigen::Vector3d(kFootContactFrontX, kFootContactHalfWidth, 0.0), dq);
    const Eigen::Vector3d dJdq_rf = pointDjdqLocal(J6, dJ6, footRot_W, Eigen::Vector3d(kFootContactFrontX, -kFootContactHalfWidth, 0.0), dq);
    const Eigen::Vector3d dJdq_lb = pointDjdqLocal(J6, dJ6, footRot_W, Eigen::Vector3d(kFootContactBackX, kFootContactHalfWidth, 0.0), dq);
    const Eigen::Vector3d dJdq_rb = pointDjdqLocal(J6, dJ6, footRot_W, Eigen::Vector3d(kFootContactBackX, -kFootContactHalfWidth, 0.0), dq);
    const Eigen::Vector3d p_lf = footRot_W * Eigen::Vector3d(kFootContactFrontX, kFootContactHalfWidth, 0.0);
    const Eigen::Vector3d p_rf = footRot_W * Eigen::Vector3d(kFootContactFrontX, -kFootContactHalfWidth, 0.0);
    const Eigen::Vector3d p_lb = footRot_W * Eigen::Vector3d(kFootContactBackX, kFootContactHalfWidth, 0.0);
    const Eigen::Vector3d p_rb = footRot_W * Eigen::Vector3d(kFootContactBackX, -kFootContactHalfWidth, 0.0);
    const double width = 2.0 * kFootContactHalfWidth;
    out.jacobian = ((J_lf.row(2) + J_lb.row(2)) * 0.5 - (J_rf.row(2) + J_rb.row(2)) * 0.5) / width;
    out.position = ((p_lf.z() + p_lb.z()) * 0.5 - (p_rf.z() + p_rb.z()) * 0.5) / width;
    out.velocity = (out.jacobian * dq)(0);
    out.dJdq = (((dJdq_lf.z() + dJdq_lb.z()) * 0.5 - (dJdq_rf.z() + dJdq_rb.z()) * 0.5) / width);
    return out;
}

Kin1D baseDeltaPitchKinematics(const Eigen::MatrixXd &Jbase,
                               const Eigen::MatrixXd &dJbase,
                               const Eigen::Matrix3d &baseRot_W,
                               const Eigen::VectorXd &dq)
{
    Kin1D out;
    const Eigen::MatrixXd J_f = pointJacobianLocal(Jbase, baseRot_W, Eigen::Vector3d(kBaseFrontX, 0.0, 0.0));
    const Eigen::MatrixXd J_b = pointJacobianLocal(Jbase, baseRot_W, Eigen::Vector3d(kBaseBackX, 0.0, 0.0));
    const Eigen::Vector3d dJdq_f = pointDjdqLocal(Jbase, dJbase, baseRot_W, Eigen::Vector3d(kBaseFrontX, 0.0, 0.0), dq);
    const Eigen::Vector3d dJdq_b = pointDjdqLocal(Jbase, dJbase, baseRot_W, Eigen::Vector3d(kBaseBackX, 0.0, 0.0), dq);
    const Eigen::Vector3d p_f = baseRot_W * Eigen::Vector3d(kBaseFrontX, 0.0, 0.0);
    const Eigen::Vector3d p_b = baseRot_W * Eigen::Vector3d(kBaseBackX, 0.0, 0.0);
    out.jacobian = (J_b.row(2) - J_f.row(2)) / (kBaseFrontX - kBaseBackX);
    out.position = (p_b.z() - p_f.z()) / (kBaseFrontX - kBaseBackX);
    out.velocity = (out.jacobian * dq)(0);
    out.dJdq = (dJdq_b.z() - dJdq_f.z()) / (kBaseFrontX - kBaseBackX);
    return out;
}

Kin1D baseDeltaRollKinematics(const Eigen::MatrixXd &Jbase,
                              const Eigen::MatrixXd &dJbase,
                              const Eigen::Matrix3d &baseRot_W,
                              const Eigen::VectorXd &dq)
{
    Kin1D out;
    const Eigen::MatrixXd J_l = pointJacobianLocal(Jbase, baseRot_W, Eigen::Vector3d(0.0, kBaseLeftY, 0.0));
    const Eigen::MatrixXd J_r = pointJacobianLocal(Jbase, baseRot_W, Eigen::Vector3d(0.0, kBaseRightY, 0.0));
    const Eigen::Vector3d dJdq_l = pointDjdqLocal(Jbase, dJbase, baseRot_W, Eigen::Vector3d(0.0, kBaseLeftY, 0.0), dq);
    const Eigen::Vector3d dJdq_r = pointDjdqLocal(Jbase, dJbase, baseRot_W, Eigen::Vector3d(0.0, kBaseRightY, 0.0), dq);
    const Eigen::Vector3d p_l = baseRot_W * Eigen::Vector3d(0.0, kBaseLeftY, 0.0);
    const Eigen::Vector3d p_r = baseRot_W * Eigen::Vector3d(0.0, kBaseRightY, 0.0);
    out.jacobian = (J_l.row(2) - J_r.row(2)) / (kBaseLeftY - kBaseRightY);
    out.position = (p_l.z() - p_r.z()) / (kBaseLeftY - kBaseRightY);
    out.velocity = (out.jacobian * dq)(0);
    out.dJdq = (dJdq_l.z() - dJdq_r.z()) / (kBaseLeftY - kBaseRightY);
    return out;
}

Eigen::Matrix<double, 6, 6> planeForceToWrenchMap()
{
    Eigen::Matrix<double, 6, 6> H;
    H << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 1.0, 1.0,
        0.0, 0.0, kFootContactHalfWidth, 0.0, -kFootContactHalfWidth, 0.0,
        0.0, 0.0, -kFootContactFrontX, 0.0, -kFootContactFrontX, -kFootContactBackX,
        -kFootContactHalfWidth, kFootContactFrontX, 0.0, kFootContactHalfWidth, 0.0, 0.0;
    return H;
}

Eigen::Matrix<double, 6, 1> distributeVerticalFootLoad(double fz)
{
    Eigen::Matrix<double, 6, 1> f;
    const double frontTotal = -kFootContactBackX / (kFootContactFrontX - kFootContactBackX) * fz;
    const double back = fz - frontTotal;
    f << 0.0, 0.0, 0.5 * frontTotal, 0.0, 0.5 * frontTotal, back;
    return f;
}

Eigen::Matrix<double, 11, 6> roMoCoPlaneFootCone(double mu,
                                                 double footHalfWidth,
                                                 double footFront,
                                                 double footBack,
                                                 double yawFriction)
{
    Eigen::Matrix<double, 11, 6> coneRaw;
    coneRaw << 0, 0, -1, 0, 0, 0,
        1, 0, -mu / std::sqrt(2.0), 0, 0, 0,
        -1, 0, -mu / std::sqrt(2.0), 0, 0, 0,
        0, 1, -mu / std::sqrt(2.0), 0, 0, 0,
        0, -1, -mu / std::sqrt(2.0), 0, 0, 0,
        0, 0, -footHalfWidth, 1, 0, 0,
        0, 0, -footHalfWidth, -1, 0, 0,
        0, 0, -footFront, 0, 1, 0,
        0, 0, footBack, 0, -1, 0,
        0, 0, -yawFriction, 0, 0, 1,
        0, 0, -yawFriction, 0, 0, -1;
    return coneRaw * planeForceToWrenchMap();
}

double motorTorqueLimit(int motorId)
{
    static const double limits[] = {
        88.0, 139.0, 88.0, 139.0, 50.0, 50.0,
        88.0, 139.0, 88.0, 139.0, 50.0, 50.0,
        88.0, 50.0, 50.0,
        25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0,
        25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0};
    constexpr int nLimits = sizeof(limits) / sizeof(limits[0]);
    if (motorId >= 0 && motorId < nLimits)
    {
        return limits[motorId];
    }
    return 50.0;
}
} // namespace g1_kin_dyn

namespace
{
Eigen::Matrix3d ParallelAxisShift(double mass, const Eigen::Vector3d &r)
{
    return mass * (r.squaredNorm() * Eigen::Matrix3d::Identity() - r * r.transpose());
}

void AddG1FootFrames(pinocchio::Model &model)
{
    const auto left_ankle_id = model.getJointId("left_ankle_roll_joint");
    const auto right_ankle_id = model.getJointId("right_ankle_roll_joint");

    auto addFrameIfMissing = [&](const std::string &name,
                                 const pinocchio::JointIndex jointId,
                                 const Eigen::Vector3d &offset)
    {
        if (!model.existFrame(name))
        {
            model.addFrame(pinocchio::Frame(name, jointId, 0,
                                            pinocchio::SE3(Eigen::Matrix3d::Identity(), offset),
                                            pinocchio::OP_FRAME));
        }
    };

    addFrameIfMissing("left_foot_LF", left_ankle_id, Eigen::Vector3d(0.12, 0.025, -0.03));
    addFrameIfMissing("left_foot_RF", left_ankle_id, Eigen::Vector3d(0.12, -0.025, -0.03));
    addFrameIfMissing("left_foot_LB", left_ankle_id, Eigen::Vector3d(-0.05, 0.025, -0.03));
    addFrameIfMissing("left_foot_RB", left_ankle_id, Eigen::Vector3d(-0.05, -0.025, -0.03));
    addFrameIfMissing("right_foot_LF", right_ankle_id, Eigen::Vector3d(0.12, 0.025, -0.03));
    addFrameIfMissing("right_foot_RF", right_ankle_id, Eigen::Vector3d(0.12, -0.025, -0.03));
    addFrameIfMissing("right_foot_LB", right_ankle_id, Eigen::Vector3d(-0.05, 0.025, -0.03));
    addFrameIfMissing("right_foot_RB", right_ankle_id, Eigen::Vector3d(-0.05, -0.025, -0.03));
    addFrameIfMissing("left_below_ankle", left_ankle_id, Eigen::Vector3d(0.0, 0.0, -0.03));
    addFrameIfMissing("right_below_ankle", right_ankle_id, Eigen::Vector3d(0.0, 0.0, -0.03));
}

Eigen::Matrix3d YawRotationFromFrame(const pinocchio::SE3 &framePose)
{
    const Eigen::Matrix3d &rot = framePose.rotation();
    return Rz3(std::atan2(rot(1, 0), rot(0, 0)));
}

void BuildRoMoCoFootConstraint(const pinocchio::Model &model,
                               pinocchio::Data &data,
                               pinocchio::FrameIndex belowAnkleFrame,
                               pinocchio::FrameIndex lfFrame,
                               pinocchio::FrameIndex rfFrame,
                               pinocchio::FrameIndex lbFrame,
                               pinocchio::FrameIndex rbFrame,
                               Eigen::MatrixXd &Jh,
                               Eigen::VectorXd &dJhdq)
{
    const Eigen::Matrix3d RyawT = YawRotationFromFrame(data.oMf[belowAnkleFrame]).transpose();
    Eigen::MatrixXd Jlf = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd Jrf = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd Jlb = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd Jrb = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, lfFrame, pinocchio::LOCAL_WORLD_ALIGNED, Jlf);
    pinocchio::getFrameJacobian(model, data, rfFrame, pinocchio::LOCAL_WORLD_ALIGNED, Jrf);
    pinocchio::getFrameJacobian(model, data, lbFrame, pinocchio::LOCAL_WORLD_ALIGNED, Jlb);
    pinocchio::getFrameJacobian(model, data, rbFrame, pinocchio::LOCAL_WORLD_ALIGNED, Jrb);

    const Eigen::Vector3d alf = RyawT * pinocchio::getFrameClassicalAcceleration(model, data, lfFrame, pinocchio::LOCAL_WORLD_ALIGNED).linear();
    const Eigen::Vector3d arf = RyawT * pinocchio::getFrameClassicalAcceleration(model, data, rfFrame, pinocchio::LOCAL_WORLD_ALIGNED).linear();
    const Eigen::Vector3d alb = RyawT * pinocchio::getFrameClassicalAcceleration(model, data, lbFrame, pinocchio::LOCAL_WORLD_ALIGNED).linear();
    const Eigen::Vector3d arb = RyawT * pinocchio::getFrameClassicalAcceleration(model, data, rbFrame, pinocchio::LOCAL_WORLD_ALIGNED).linear();

    Jlf.topRows(3) = RyawT * Jlf.topRows(3);
    Jrf.topRows(3) = RyawT * Jrf.topRows(3);
    Jlb.topRows(3) = RyawT * Jlb.topRows(3);
    Jrb.topRows(3) = RyawT * Jrb.topRows(3);

    Jh.resize(6, model.nv);
    Jh.topRows(3) = Jlf.topRows(3);
    Jh.row(3) = Jrf.row(0);
    Jh.row(4) = Jrf.row(2);
    Jh.row(5) = 0.5 * (Jlb.row(2) + Jrb.row(2));

    dJhdq.resize(6);
    dJhdq.head<3>() = alf;
    dJhdq(3) = arf.x();
    dJhdq(4) = arf.z();
    dJhdq(5) = 0.5 * (alb.z() + arb.z());
}
} // namespace

Pin_KinDyn::Pin_KinDyn(std::string urdf_pathIn)
{
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(urdf_pathIn, root_joint, model_biped);
    pinocchio::urdf::buildModel(urdf_pathIn, model_biped_fixed);
    AddG1FootFrames(model_biped);
    AddG1FootFrames(model_biped_fixed);
    data_biped = pinocchio::Data(model_biped);
    data_biped_fixed = pinocchio::Data(model_biped_fixed);
    model_nv = model_biped.nv;
    J_l = Eigen::MatrixXd::Zero(6, model_nv);
    J_r = Eigen::MatrixXd::Zero(6, model_nv);
    J_l_body = Eigen::MatrixXd::Zero(6, model_biped_fixed.nv);
    J_r_body = Eigen::MatrixXd::Zero(6, model_biped_fixed.nv);
    J_hd_l = Eigen::MatrixXd::Zero(6, model_nv);
    J_hd_r = Eigen::MatrixXd::Zero(6, model_nv);
    J_base = Eigen::MatrixXd::Zero(6, model_nv);
    J_hip_link = Eigen::MatrixXd::Zero(6, model_nv);
    J_l_foot = Eigen::MatrixXd::Zero(6, model_nv);
    J_r_foot = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_l = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_r = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_l_foot = Eigen::VectorXd::Zero(6);
    dJ_r_foot = Eigen::VectorXd::Zero(6);
    dJ_hd_l = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_hd_r = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_base = Eigen::MatrixXd::Zero(6, model_nv);
    q.setZero();
    dq.setZero();
    ddq.setZero();
    Rcur.setIdentity();
    dyn_M = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_M_inv = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_C = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_G = Eigen::MatrixXd::Zero(model_nv, 1);

    // G1 joint and frame ids.
    r_ankle_joint = model_biped.getJointId("right_ankle_roll_joint");
    l_ankle_joint = model_biped.getJointId("left_ankle_roll_joint");
    r_foot_frame = model_biped.getFrameId("right_below_ankle");
    l_foot_frame = model_biped.getFrameId("left_below_ankle");
    l_foot_lf_frame = model_biped.getFrameId("left_foot_LF");
    l_foot_rf_frame = model_biped.getFrameId("left_foot_RF");
    l_foot_lb_frame = model_biped.getFrameId("left_foot_LB");
    l_foot_rb_frame = model_biped.getFrameId("left_foot_RB");
    r_foot_lf_frame = model_biped.getFrameId("right_foot_LF");
    r_foot_rf_frame = model_biped.getFrameId("right_foot_RF");
    r_foot_lb_frame = model_biped.getFrameId("right_foot_LB");
    r_foot_rb_frame = model_biped.getFrameId("right_foot_RB");
    r_hand_joint = model_biped.getJointId("right_wrist_yaw_joint");
    l_hand_joint = model_biped.getJointId("left_wrist_yaw_joint");
    r_hand_joint_fixed = model_biped_fixed.getJointId("right_wrist_yaw_joint");
    l_hand_joint_fixed = model_biped_fixed.getJointId("left_wrist_yaw_joint");
    r_hip_joint = model_biped.getJointId("right_hip_yaw_joint");
    l_hip_joint = model_biped.getJointId("left_hip_yaw_joint");
    r_hip_roll_joint = model_biped.getJointId("right_hip_roll_joint");
    l_hip_roll_joint = model_biped.getJointId("left_hip_roll_joint");
    r_ankle_joint_fixed = model_biped_fixed.getJointId("right_ankle_roll_joint");
    l_ankle_joint_fixed = model_biped_fixed.getJointId("left_ankle_roll_joint");
    r_foot_frame_fixed = model_biped_fixed.getFrameId("right_below_ankle");
    l_foot_frame_fixed = model_biped_fixed.getFrameId("left_below_ankle");
    r_hip_joint_fixed = model_biped_fixed.getJointId("right_hip_yaw_joint");
    l_hip_joint_fixed = model_biped_fixed.getJointId("left_hip_yaw_joint");
    base_joint = model_biped.getJointId("root_joint");
    waist_yaw_joint = model_biped.getJointId("waist_yaw_joint");

    // read joint pvt parameters
    Json::Reader reader;
    Json::Value root_read;
    std::ifstream in("../common/joint_ctrl_config.json", std::ios::binary);
    if (!in.is_open())
    {
        in.open("common/joint_ctrl_config.json", std::ios::binary);
    }

    motorMaxTorque = Eigen::VectorXd::Zero(motorName.size());
    motorMaxPos = Eigen::VectorXd::Zero(motorName.size());
    motorMinPos = Eigen::VectorXd::Zero(motorName.size());
    reader.parse(in, root_read);
    for (int i = 0; i < motorName.size(); i++)
    {
        motorMaxTorque(i) = (root_read[motorName[i]]["maxTorque"].asDouble());
        motorMaxPos(i) = (root_read[motorName[i]]["maxPos"].asDouble());
        motorMinPos(i) = (root_read[motorName[i]]["minPos"].asDouble());
    }
    motorReachLimit.assign(motorName.size(), false);
    tauJointOld = Eigen::VectorXd::Zero(motorName.size());
}

void Pin_KinDyn::dataBusRead(const DataBus &robotState)
{
    //  For Pinocchio: The base translation part is expressed in the parent frame (here the world coordinate system)
    //  while its velocity is expressed in the body coordinate system.
    //  https://github.com/stack-of-tasks/pinocchio/issues/1137
    //  q = [global_base_position, global_base_quaternion, joint_positions]
    //  v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]
    q = robotState.q;
    dq = robotState.dq;
    dq.block(0, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(0, 0, 3, 1);
    dq.block(3, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(3, 0, 3, 1);
    ddq = robotState.ddq;
}

void Pin_KinDyn::dataBusWrite(DataBus &robotState)
{
    robotState.J_l = J_l;
    robotState.J_r = J_r;
    robotState.J_base = J_base;
    robotState.dJ_l = dJ_l;
    robotState.dJ_r = dJ_r;
    robotState.J_l_foot = J_l_foot;
    robotState.J_r_foot = J_r_foot;
    robotState.dJ_l_foot = dJ_l_foot;
    robotState.dJ_r_foot = dJ_r_foot;
    robotState.J_hd_l = J_hd_l;
    robotState.J_hd_r = J_hd_r;
    robotState.dJ_hd_l = dJ_hd_l;
    robotState.dJ_hd_r = dJ_hd_r;
    robotState.dJ_base = dJ_base;
    robotState.J_hip_link = J_hip_link;
    robotState.fe_l_pos_W = fe_l_pos;
    robotState.fe_r_pos_W = fe_r_pos;
    robotState.fe_l_pos_L = fe_l_pos_body;
    robotState.fe_r_pos_L = fe_r_pos_body;
    robotState.fe_l_rot_W = fe_l_rot;
    robotState.fe_r_rot_W = fe_r_rot;
    robotState.fe_l_rot_L = fe_l_rot_body;
    robotState.fe_r_rot_L = fe_r_rot_body;
    robotState.fe_l_vel_L = fe_l_vel_body;
    robotState.fe_r_vel_L = fe_r_vel_body;
    robotState.hip_r_pos_L = hip_r_pos_body;
    robotState.hip_l_pos_L = hip_l_pos_body;
    robotState.hip_r_pos_W = hip_r_pos;
    robotState.hip_l_pos_W = hip_l_pos;
    robotState.hd_l_pos_L = hd_l_pos_body;
    robotState.hd_l_rot_L = hd_l_rot_body;
    robotState.hd_l_pos_W = hd_l_pos;
    robotState.hd_l_rot_W = hd_l_rot;
    robotState.hd_r_pos_L = hd_r_pos_body;
    robotState.hd_r_rot_L = hd_r_rot_body;
    robotState.hd_r_pos_W = hd_r_pos;
    robotState.hd_r_rot_W = hd_r_rot;
    robotState.hip_link_pos = hip_link_pos;
    robotState.hip_link_rot = hip_link_rot;

    robotState.dyn_M = dyn_M;
    robotState.dyn_M_inv = dyn_M_inv;
    robotState.dyn_C = dyn_C;
    robotState.dyn_G = dyn_G;
    robotState.dyn_Ag = dyn_Ag;
    robotState.dyn_dAg = dyn_dAg;
    robotState.dyn_Non = dyn_Non;

    robotState.pCoM_W = CoM_pos;
    robotState.Jcom_W = Jcom;
    robotState.dJcom_W = dJcom;

    robotState.inertia = inertia; // w.r.t body frame
}

// update jacobians and joint positions
void Pin_KinDyn::computeJ_dJ()
{
    pinocchio::forwardKinematics(model_biped, data_biped, q, dq, Eigen::VectorXd::Zero(model_nv));
    pinocchio::jacobianCenterOfMass(model_biped, data_biped, q, true);
    pinocchio::centerOfMass(model_biped, data_biped, pinocchio::KinematicLevel::ACCELERATION, false);
    //    pinocchio::computeJointJacobians(model_biped,data_biped,q);
    pinocchio::computeJointJacobiansTimeVariation(model_biped, data_biped, q, dq);
    pinocchio::updateGlobalPlacements(model_biped, data_biped);
    pinocchio::updateFramePlacements(model_biped, data_biped);
    pinocchio::getFrameJacobian(model_biped, data_biped, r_foot_frame, pinocchio::LOCAL_WORLD_ALIGNED, J_r);
    pinocchio::getFrameJacobian(model_biped, data_biped, l_foot_frame, pinocchio::LOCAL_WORLD_ALIGNED, J_l);
    BuildRoMoCoFootConstraint(model_biped, data_biped, l_foot_frame,
                              l_foot_lf_frame, l_foot_rf_frame, l_foot_lb_frame, l_foot_rb_frame,
                              J_l_foot, dJ_l_foot);
    BuildRoMoCoFootConstraint(model_biped, data_biped, r_foot_frame,
                              r_foot_lf_frame, r_foot_rf_frame, r_foot_lb_frame, r_foot_rb_frame,
                              J_r_foot, dJ_r_foot);
    pinocchio::getJointJacobian(model_biped, data_biped, r_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hd_r);
    pinocchio::getJointJacobian(model_biped, data_biped, l_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hd_l);
    pinocchio::getJointJacobian(model_biped, data_biped, base_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_base);

    Eigen::Matrix<double, 6, -1> J_hip_roll_l, J_hip_roll_r;
    J_hip_roll_l = Eigen::MatrixXd::Zero(6, model_nv);
    J_hip_roll_r = Eigen::MatrixXd::Zero(6, model_nv);
    pinocchio::getJointJacobian(model_biped, data_biped, l_hip_roll_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_roll_l);
    pinocchio::getJointJacobian(model_biped, data_biped, r_hip_roll_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_roll_r);
    //    J_hip_link=J_hip_roll_l;
    //    std::cout<<"J_hip_roll_l"<<std::endl<<J_hip_roll_l<<std::endl;
    //    std::cout<<"J_hip_roll_r"<<std::endl<<J_hip_roll_r<<std::endl;
    pinocchio::getJointJacobian(model_biped, data_biped, waist_yaw_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_link);

    pinocchio::getFrameJacobianTimeVariation(model_biped, data_biped, r_foot_frame, pinocchio::LOCAL_WORLD_ALIGNED, dJ_r);
    pinocchio::getFrameJacobianTimeVariation(model_biped, data_biped, l_foot_frame, pinocchio::LOCAL_WORLD_ALIGNED, dJ_l);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, r_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_hd_r);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, l_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_hd_l);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, base_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_base);
    fe_l_pos = data_biped.oMf[l_foot_frame].translation();
    fe_l_rot = data_biped.oMf[l_foot_frame].rotation();
    hip_l_pos = data_biped.oMi[l_hip_joint].translation();
    fe_r_pos = data_biped.oMf[r_foot_frame].translation();
    fe_r_rot = data_biped.oMf[r_foot_frame].rotation();
    hip_r_pos = data_biped.oMi[r_hip_joint].translation();
    base_pos = data_biped.oMi[base_joint].translation();
    base_rot = data_biped.oMi[base_joint].rotation();
    hd_l_pos = data_biped.oMi[l_hand_joint].translation();
    hd_l_rot = data_biped.oMi[l_hand_joint].rotation();
    hd_r_pos = data_biped.oMi[r_hand_joint].translation();
    hd_r_rot = data_biped.oMi[r_hand_joint].rotation();
    //    hip_link_pos=(data_biped.oMi[l_hip_roll_joint].translation()+data_biped.oMi[r_hip_roll_joint].translation())*0.5;
    //    hip_link_rot=data_biped.oMi[l_hip_roll_joint].rotation();
    hip_link_pos = data_biped.oMi[waist_yaw_joint].translation();
    hip_link_rot = data_biped.oMi[waist_yaw_joint].rotation();
    Jcom = data_biped.Jcom;

    Eigen::MatrixXd Mpj; // transform into world frame, and accept dq that in world frame
    Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj.block(0, 0, 3, 3) = base_rot.transpose();
    Mpj.block(3, 3, 3, 3) = base_rot.transpose();
    J_l = J_l * Mpj;
    J_r = J_r * Mpj;
    J_l_foot = J_l_foot * Mpj;
    J_r_foot = J_r_foot * Mpj;
    J_base = J_base * Mpj;
    dJ_l = dJ_l * Mpj;
    dJ_r = dJ_r * Mpj;
    J_hd_l = J_hd_l * Mpj;
    J_hd_r = J_hd_r * Mpj;
    dJ_hd_l = dJ_hd_l * Mpj;
    dJ_hd_r = dJ_hd_r * Mpj;
    dJ_base = dJ_base * Mpj;
    J_hip_link = J_hip_link * Mpj;
    Jcom = Jcom * Mpj;
    dJcom = data_biped.acom[0];

    Eigen::VectorXd q_fixed, dq_fixed;
    q_fixed = q.block(7, 0, model_biped_fixed.nv, 1);
    dq_fixed = dq.block(6, 0, model_biped_fixed.nv, 1);
    pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, q_fixed);
    pinocchio::computeJointJacobians(model_biped_fixed, data_biped_fixed, q_fixed);
    pinocchio::updateGlobalPlacements(model_biped_fixed, data_biped_fixed);
    pinocchio::updateFramePlacements(model_biped_fixed, data_biped_fixed);
    pinocchio::getFrameJacobian(model_biped_fixed, data_biped_fixed, r_foot_frame_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_r_body);
    pinocchio::getFrameJacobian(model_biped_fixed, data_biped_fixed, l_foot_frame_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_l_body);
    fe_l_pos_body = data_biped_fixed.oMf[l_foot_frame_fixed].translation();
    fe_r_pos_body = data_biped_fixed.oMf[r_foot_frame_fixed].translation();
    fe_l_rot_body = data_biped_fixed.oMf[l_foot_frame_fixed].rotation();
    fe_r_rot_body = data_biped_fixed.oMf[r_foot_frame_fixed].rotation();
    hip_l_pos_body = data_biped_fixed.oMi[l_hip_joint_fixed].translation();
    hip_r_pos_body = data_biped_fixed.oMi[r_hip_joint_fixed].translation();
    hd_l_pos_body = data_biped_fixed.oMi[l_hand_joint_fixed].translation();
    hd_l_rot_body = data_biped_fixed.oMi[l_hand_joint_fixed].rotation();
    hd_r_pos_body = data_biped_fixed.oMi[r_hand_joint_fixed].translation();
    hd_r_rot_body = data_biped_fixed.oMi[r_hand_joint_fixed].rotation();
    fe_l_vel_body = (J_l_body * dq_fixed).block(0, 0, 3, 1);
    fe_r_vel_body = (J_r_body * dq_fixed).block(0, 0, 3, 1);
}

Eigen::Quaterniond Pin_KinDyn::intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w)
{
    Eigen::Matrix3d Rcur = quat.normalized().toRotationMatrix();
    Eigen::Matrix3d Rinc = Eigen::Matrix3d::Identity();
    double theta = w.norm();
    if (theta > 1e-8)
    {
        Eigen::Vector3d w_norm;
        w_norm = w / theta;
        Eigen::Matrix3d a;
        a << 0, -w_norm(2), w_norm(1),
            w_norm(0), 0, -w_norm(0),
            -w_norm(1), w_norm(0), 0;
        Rinc = Eigen::Matrix3d::Identity() + a * sin(theta) + a * a * (1 - cos(theta));
    }
    Eigen::Matrix3d Rend = Rcur * Rinc;
    Eigen::Quaterniond quatRes;
    quatRes = Rend;
    return quatRes;
}

// intergrate the q with dq, for floating base dynamics
Eigen::VectorXd Pin_KinDyn::integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI)
{
    Eigen::VectorXd qRes = Eigen::VectorXd::Zero(model_nv + 1);
    Eigen::Vector3d wDes;
    wDes << dqI(3), dqI(4), dqI(5);
    Eigen::Quaterniond quatNew, quatNow;
    quatNow.x() = qI(3);
    quatNow.y() = qI(4);
    quatNow.z() = qI(5);
    quatNow.w() = qI(6);
    quatNew = intQuat(quatNow, wDes);
    qRes = qI;
    qRes(0) += dqI(0);
    qRes(1) += dqI(1);
    qRes(2) += dqI(2);
    qRes(3) = quatNew.x();
    qRes(4) = quatNew.y();
    qRes(5) = quatNew.z();
    qRes(6) = quatNew.w();
    for (int i = 0; i < model_nv - 6; i++)
        qRes(7 + i) += dqI(6 + i);
    return qRes;
}

// update dynamic parameters, M*ddq+C*dq+G=tau
void Pin_KinDyn::computeDyn()
{
    // cal M
    pinocchio::crba(model_biped, data_biped, q);
    // Pinocchio only gives half of the M, needs to restore it here
    data_biped.M.triangularView<Eigen::Lower>() = data_biped.M.transpose().triangularView<Eigen::Lower>();
    dyn_M = data_biped.M;

    // cal Minv
    pinocchio::computeMinverse(model_biped, data_biped, q);
    data_biped.Minv.triangularView<Eigen::Lower>() = data_biped.Minv.transpose().triangularView<Eigen::Lower>();
    dyn_M_inv = data_biped.Minv;

    // cal C
    pinocchio::computeCoriolisMatrix(model_biped, data_biped, q, dq);
    dyn_C = data_biped.C;

    // cal G
    pinocchio::computeGeneralizedGravity(model_biped, data_biped, q);
    dyn_G = data_biped.g;

    // cal Ag, Centroidal Momentum Matrix. First three rows: linear, other three rows: angular
    pinocchio::dccrba(model_biped, data_biped, q, dq);
    pinocchio::computeCentroidalMomentum(model_biped, data_biped, q, dq);
    dyn_Ag = data_biped.Ag;
    dyn_dAg = data_biped.dAg;

    // cal nonlinear item
    dyn_Non = dyn_C * dq + dyn_G;

    // cal I
    pinocchio::ccrba(model_biped, data_biped, q, dq);
    inertia = data_biped.Ig.inertia().matrix();

    // cal CoM
    CoM_pos = data_biped.com[0];
    //    std::cout<<"CoM_W"<<std::endl;
    //    std::cout<<CoM_pos.transpose()<<std::endl;

    Eigen::MatrixXd Mpj, Mpj_inv; // transform into world frame
    Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj_inv = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj.block(0, 0, 3, 3) = base_rot.transpose();
    Mpj.block(3, 3, 3, 3) = base_rot.transpose();
    Mpj_inv.block(0, 0, 3, 3) = base_rot;
    Mpj_inv.block(3, 3, 3, 3) = base_rot;
    dyn_M = Mpj_inv * dyn_M * Mpj;
    dyn_M_inv = Mpj_inv * dyn_M_inv * Mpj;
    dyn_C = Mpj_inv * dyn_C * Mpj;
    dyn_G = Mpj_inv * dyn_G;
    dyn_Non = Mpj_inv * dyn_Non;
    dyn_Ag = dyn_Ag * Mpj;
    dyn_dAg = dyn_dAg * Mpj;
}

Pin_KinDyn::CompositeInertia Pin_KinDyn::computeLowerBodyInertia(const Eigen::Vector3d &referenceComWorld)
{
    CompositeInertia out;
    pinocchio::forwardKinematics(model_biped, data_biped, q);
    pinocchio::updateGlobalPlacements(model_biped, data_biped);

    static const std::vector<std::string> jointNames = {
        "root_joint",
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};

    struct BodyContribution
    {
        double mass{0.0};
        Eigen::Vector3d comWorld{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d inertiaAboutBodyComWorld{Eigen::Matrix3d::Zero()};
    };
    std::vector<BodyContribution> bodies;
    bodies.reserve(jointNames.size());

    for (const std::string &name : jointNames)
    {
        if (!model_biped.existJointName(name))
        {
            continue;
        }
        const pinocchio::JointIndex jointId = model_biped.getJointId(name);
        if (jointId >= model_biped.inertias.size())
        {
            continue;
        }
        const pinocchio::Inertia &bodyInertia = model_biped.inertias[jointId];
        const double mass = bodyInertia.mass();
        if (mass <= 0.0)
        {
            continue;
        }

        const Eigen::Matrix3d &rotWorld = data_biped.oMi[jointId].rotation();
        const Eigen::Vector3d comWorld = data_biped.oMi[jointId].act(bodyInertia.lever());
        const Eigen::Matrix3d inertiaWorld = rotWorld * bodyInertia.inertia().matrix() * rotWorld.transpose();

        bodies.push_back({mass, comWorld, inertiaWorld});
        out.mass += mass;
    }

    if (out.mass <= 1e-9)
    {
        return out;
    }

    for (const BodyContribution &body : bodies)
    {
        out.inertiaAboutReferenceWorld.noalias() +=
            body.inertiaAboutBodyComWorld + ParallelAxisShift(body.mass, body.comWorld - referenceComWorld);
    }
    return out;
}

// Inverse kinematics for leg posture. Note: the Rdes and Pdes are both w.r.t the baselink coordinate in body frame!
Pin_KinDyn::IkRes
Pin_KinDyn::computeInK_Leg(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R,
                           const Eigen::Vector3d &Pdes_R)
{
    const pinocchio::SE3 oMdesL(Rdes_L, Pdes_L);
    const pinocchio::SE3 oMdesR(Rdes_R, Pdes_R);
    // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
    Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_biped_fixed.nv); // initial guess
    if (q.size() >= 7 + model_biped_fixed.nv)
    {
        qIk = q.segment(7, model_biped_fixed.nv);
    }
    else
    {
        qIk[22] = -0.1;
        qIk[28] = -0.1;
    }

    const double eps = 1e-4;
    const int IT_MAX = 100;
    const double DT = 7e-1;
    const double damp = 5e-3;
    Eigen::MatrixXd JL(6, model_biped_fixed.nv);
    Eigen::MatrixXd JR(6, model_biped_fixed.nv);
    Eigen::MatrixXd JCompact(12, model_biped_fixed.nv);
    JL.setZero();
    JR.setZero();
    JCompact.setZero();

    bool success = false;
    Eigen::Matrix<double, 6, 1> errL, errR;
    Eigen::Matrix<double, 12, 1> errCompact;
    Eigen::VectorXd v(model_biped_fixed.nv);

    int itr_count{0};
    for (itr_count = 0;; itr_count++)
    {
        pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
        pinocchio::computeJointJacobians(model_biped_fixed, data_biped_fixed, qIk);
        pinocchio::updateFramePlacements(model_biped_fixed, data_biped_fixed);
        const pinocchio::SE3 iMdL = data_biped_fixed.oMf[l_foot_frame_fixed].actInv(oMdesL);
        const pinocchio::SE3 iMdR = data_biped_fixed.oMf[r_foot_frame_fixed].actInv(oMdesR);
        errL = pinocchio::log6(iMdL).toVector(); // in joint frame
        errR = pinocchio::log6(iMdR).toVector(); // in joint frame
        errCompact.block<6, 1>(0, 0) = errL;
        errCompact.block<6, 1>(6, 0) = errR;
        if (errCompact.norm() < eps)
        {
            success = true;
            break;
        }
        if (itr_count >= IT_MAX)
        {
            success = false;
            break;
        }

        pinocchio::getFrameJacobian(model_biped_fixed, data_biped_fixed, l_foot_frame_fixed, pinocchio::LOCAL, JL);
        pinocchio::getFrameJacobian(model_biped_fixed, data_biped_fixed, r_foot_frame_fixed, pinocchio::LOCAL, JR);
        Eigen::MatrixXd W;
        W = Eigen::MatrixXd::Identity(model_biped_fixed.nv, model_biped_fixed.nv); // weighted matrix
        // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
        //        W(16,16)=0.001;  // use a smaller value to make the solver try not to use waist joint
        //        W(17,17)=0.001;
        //        W(18,18)=0.001;
        JL.block(0, 16, 6, 3).setZero();
        JR.block(0, 16, 6, 3).setZero();
        pinocchio::Data::Matrix6 JlogL;
        pinocchio::Data::Matrix6 JlogR;
        pinocchio::Jlog6(iMdL.inverse(), JlogL);
        pinocchio::Jlog6(iMdR.inverse(), JlogR);
        JL = -JlogL * JL;
        JR = -JlogR * JR;
        JCompact.block(0, 0, 6, model_biped_fixed.nv) = JL;
        JCompact.block(6, 0, 6, model_biped_fixed.nv) = JR;
        // pinocchio::Data::Matrix6 JJt;
        Eigen::Matrix<double, 12, 12> JJt;
        JJt.noalias() = JCompact * W * JCompact.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -W * JCompact.transpose() * JJt.ldlt().solve(errCompact);
        qIk = pinocchio::integrate(model_biped_fixed, qIk, v * DT);
    }

    IkRes res;
    res.err = errCompact;
    res.itr = itr_count;

    if (success)
    {
        res.status = 0;
    }
    else
    {
        res.status = -1;
    }
    res.jointPosRes = qIk;
    return res;
}

// Inverse Kinematics for hand posture. Note: the Rdes and Pdes are both w.r.t the baselink coordinate in body frame!
Pin_KinDyn::IkRes
Pin_KinDyn::computeInK_Hand(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R,
                            const Eigen::Vector3d &Pdes_R)
{
    const pinocchio::SE3 oMdesL(Rdes_L, Pdes_L);
    const pinocchio::SE3 oMdesR(Rdes_R, Pdes_R);
    Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_biped_fixed.nv); // initial guess
    // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
    qIk.block<7, 1>(0, 0) << 0.433153883479341, 1.11739345867607, 1.88491913406236,
        0.802378252758275, 1.22726400279662, 0.0249797771339966, -0.0875282610654057;

    qIk.block<7, 1>(7, 0) << -0.433152540054138, -1.11739347975224, -1.88492038240761,
        0.802375980602373, -1.22726323451626, 0.0249795712262396, 0.0875271396314979;

    const double eps = 1e-4;
    const int IT_MAX = 100;
    const double DT = 6e-1;
    const double damp = 1e-2;
    Eigen::MatrixXd JL(6, model_biped_fixed.nv);
    Eigen::MatrixXd JR(6, model_biped_fixed.nv);
    Eigen::MatrixXd JCompact(12, model_biped_fixed.nv);
    JL.setZero();
    JR.setZero();
    JCompact.setZero();

    bool success = false;
    Eigen::Matrix<double, 6, 1> errL, errR;
    Eigen::Matrix<double, 12, 1> errCompact;
    Eigen::VectorXd v(model_biped_fixed.nv);

    pinocchio::JointIndex J_Idx_l, J_Idx_r;
    J_Idx_l = l_hand_joint_fixed;
    J_Idx_r = r_hand_joint_fixed;
    int itr_count{0};
    for (itr_count = 0;; itr_count++)
    {
        pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
        const pinocchio::SE3 iMdL = data_biped_fixed.oMi[J_Idx_l].actInv(oMdesL);
        const pinocchio::SE3 iMdR = data_biped_fixed.oMi[J_Idx_r].actInv(oMdesR);
        errL = pinocchio::log6(iMdL).toVector(); // in joint frame
        errR = pinocchio::log6(iMdR).toVector(); // in joint frame
        errCompact.block<6, 1>(0, 0) = errL;
        errCompact.block<6, 1>(6, 0) = errR;
        if (errCompact.norm() < eps)
        {
            success = true;
            break;
        }
        if (itr_count >= IT_MAX)
        {
            success = false;
            break;
        }

        pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_l, JL); // JL in joint frame
        pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_r, JR); // JR in joint frame
        pinocchio::Data::Matrix6 JlogL;
        pinocchio::Data::Matrix6 JlogR;
        pinocchio::Jlog6(iMdL.inverse(), JlogL);
        pinocchio::Jlog6(iMdR.inverse(), JlogR);
        JL = -JlogL * JL;
        JR = -JlogR * JR;
        JCompact.block(0, 0, 6, model_biped_fixed.nv) = JL;
        JCompact.block(6, 0, 6, model_biped_fixed.nv) = JR;
        // pinocchio::Data::Matrix6 JJt;
        Eigen::Matrix<double, 12, 12> JJt;
        JJt.noalias() = JCompact * JCompact.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -JCompact.transpose() * JJt.ldlt().solve(errCompact);
        qIk = pinocchio::integrate(model_biped_fixed, qIk, v * DT);
    }

    IkRes res;
    res.err = errCompact;
    res.itr = itr_count;

    if (success)
    {
        res.status = 0;
    }
    else
    {
        res.status = -1;
    }

    res.jointPosRes = qIk;
    return res;
}

// must call computeDyn() first!
void Pin_KinDyn::workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT)
{
    for (int i = 0; i < motorName.size(); i++)
        if (qFT(i + 7) > motorMaxPos(i))
        {
            qFT(i + 7) = motorMaxPos(i);
            motorReachLimit[i] = true;
            tauJointFT(i) = tauJointOld(i);
        }
        else if (qFT(i + 7) < motorMinPos(i))
        {
            qFT(i + 7) = motorMinPos(i);
            motorReachLimit[i] = true;
            tauJointFT(i) = tauJointOld(i);
        }
        else
            motorReachLimit[i] = false;

    tauJointOld = tauJointFT;
}

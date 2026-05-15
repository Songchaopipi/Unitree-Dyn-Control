/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "wbc_priority.h"
#include <cmath>
#include "iostream"

namespace
{
constexpr int kBaseVelDof = 6;
constexpr int kLeftHipPitchQ = 7;
constexpr int kLeftHipRollQ = 8;
constexpr int kLeftHipYawQ = 9;
constexpr int kLeftKneeQ = 10;
constexpr int kLeftAnklePitchQ = 11;
constexpr int kLeftAnkleRollQ = 12;
constexpr int kRightHipPitchQ = 13;
constexpr int kRightHipRollQ = 14;
constexpr int kRightHipYawQ = 15;
constexpr int kRightKneeQ = 16;
constexpr int kRightAnklePitchQ = 17;
constexpr int kRightAnkleRollQ = 18;
constexpr int kWaistYawQ = 19;
constexpr int kWaistRollQ = 20;
constexpr int kWaistPitchQ = 21;
constexpr int kUpperBodyQStart = 19;
constexpr int kUpperBodyDof = 17;
constexpr double kFootContactFrontX = 0.12;
constexpr double kFootContactBackX = -0.05;
constexpr double kFootContactHalfWidth = 0.025;

double WrapToPi(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double ReferenceValue(const Eigen::VectorXd &values, int index, double fallback = 0.0)
{
    return (index >= 0 && index < values.size()) ? values(index) : fallback;
}

void SetJointSelector(Eigen::MatrixXd &J, int row, int q_index)
{
    J(row, q_index - 1) = 1.0;
}

Eigen::Matrix<double, 3, 3> Skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix<double, 3, 3> m;
    m << 0.0, -v.z(), v.y(),
        v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
}

Eigen::Matrix3d YawRotationFromFoot(const Eigen::Matrix3d &footRot_W)
{
    const double footYaw = std::atan2(footRot_W(1, 0), footRot_W(0, 0));
    Eigen::Matrix3d yawRot_W;
    yawRot_W << std::cos(footYaw), -std::sin(footYaw), 0.0,
        std::sin(footYaw), std::cos(footYaw), 0.0,
        0.0, 0.0, 1.0;
    return yawRot_W;
}

Eigen::MatrixXd PointJacobianLocal(const Eigen::MatrixXd &J6,
                                   const Eigen::Matrix3d &footRot_W,
                                   const Eigen::Vector3d &pointInFoot)
{
    const Eigen::Vector3d pointOffset_W = footRot_W * pointInFoot;
    Eigen::MatrixXd Jp_W = J6.topRows(3) - Skew(pointOffset_W) * J6.bottomRows(3);
    return YawRotationFromFoot(footRot_W).transpose() * Jp_W;
}

Eigen::MatrixXd BuildPlaneFootContactJacobianFromPointJacobian(const Eigen::MatrixXd &J6,
                                                               const Eigen::Matrix3d &footRot_W)
{
    const Eigen::MatrixXd J_lf = PointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactFrontX, kFootContactHalfWidth, 0.0));
    const Eigen::MatrixXd J_rf = PointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactFrontX, -kFootContactHalfWidth, 0.0));
    const Eigen::MatrixXd J_mb = PointJacobianLocal(J6, footRot_W, Eigen::Vector3d(kFootContactBackX, 0.0, 0.0));

    Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(6, J6.cols());
    Jc.topRows(3) = J_lf;
    Jc.row(3) = J_rf.row(0);
    Jc.row(4) = J_rf.row(2);
    Jc.row(5) = J_mb.row(2);
    return Jc;
}

Eigen::MatrixXd BuildPlaneFootContactJacobian(const Eigen::MatrixXd &J6,
                                              const Eigen::Matrix3d &footRot_W)
{
    return BuildPlaneFootContactJacobianFromPointJacobian(J6, footRot_W);
}

Eigen::MatrixXd BuildPlaneFootContactTimeVariation(const Eigen::MatrixXd &dJ6,
                                                   const Eigen::Matrix3d &footRot_W)
{
    return BuildPlaneFootContactJacobianFromPointJacobian(dJ6, footRot_W);
}

Eigen::Matrix<double, 6, 6> PlaneContactForceToWrenchMap()
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

Eigen::Matrix<double, 11, 6> RoMoCoPlaneFootCone(double mu,
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
    return coneRaw * PlaneContactForceToWrenchMap();
}

Eigen::Matrix<double, 11, 6> RoMoCoWrenchCone(double mu,
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
    return coneRaw;
}

} // namespace

// QP_nvIn=18, QP_ncIn=34
WBC_priority::WBC_priority(int model_nv_In, int QP_nvIn, int QP_ncIn, double miu_In, double dt) : QP_prob(QP_nvIn,
                                                                                                          QP_nc_des)
{
    (void)QP_ncIn;
    timeStep = dt;
    model_nv = model_nv_In;
    miu = miu_In;
    QP_nc = QP_nc_des;
    QP_nv = QP_nvIn;
    Sf = Eigen::MatrixXd::Zero(6, model_nv);
    Sf.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    St_qpV2 = Eigen::MatrixXd::Zero(model_nv, model_nv - 6); // 6 means the dims of floating base
    St_qpV2.block(6, 0, model_nv - 6, model_nv - 6) = Eigen::MatrixXd::Identity(model_nv - 6, model_nv - 6);

    St_qpV1 = Eigen::MatrixXd::Zero(model_nv, 6); // 6 means the dims of delta_b
    St_qpV1.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);

    // defined in body frame
    f_z_low = 100;
    f_z_upp = 1400;

    tau_upp_stand_L << 15, 30, 40; // foot end contact torque limit for stand state, in body frame
    tau_low_stand_L << -15, -30, -40;

    tau_upp_walk_L << 15, 40, 40; // foot end contact torque limit for walk state, in body frame
    tau_low_walk_L << -15, -40, -40;

    qpOASES::Options options;
    options.setToMPC();
    // options.setToReliable();
    options.printLevel = qpOASES::PL_LOW;
    QP_prob.setOptions(options);

    eigen_xOpt = Eigen::VectorXd::Zero(QP_nv);
    eigen_ddq_Opt = Eigen::VectorXd::Zero(model_nv);
    eigen_fr_Opt = Eigen::VectorXd::Zero(12);
    eigen_tau_Opt = Eigen::VectorXd::Zero(model_nv - 6);

    delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
    dq_final_kin = Eigen::VectorXd::Zero(model_nv);
    ;
    ddq_final_kin = Eigen::VectorXd::Zero(model_nv);

    base_rpy_cur = Eigen::VectorXd::Zero(3);

    //  WBC task defined and order build
    ///------------ walk --------------
    kin_tasks_walk.addTask("static_Contact");
    kin_tasks_walk.addTask("Roll_Pitch_Yaw_Pz");
    kin_tasks_walk.addTask("RoMoCoWalkingOutputs");
    kin_tasks_walk.addTask("RoMoCoSupportOutputs");
    kin_tasks_walk.addTask("RoMoCoSwingPosition");
    kin_tasks_walk.addTask("RoMoCoSwingOrientation");
    kin_tasks_walk.addTask("BaseYaw");
    kin_tasks_walk.addTask("WaistTrack");
    kin_tasks_walk.addTask("PxPy");
    kin_tasks_walk.addTask("SwingLeg");
    kin_tasks_walk.addTask("HandTrackJoints");
    kin_tasks_walk.addTask("PosRot");

    std::vector<std::string> taskOrder_walk;

    taskOrder_walk.emplace_back("static_Contact");
    taskOrder_walk.emplace_back("BaseYaw");
    taskOrder_walk.emplace_back("RoMoCoSupportOutputs");
    taskOrder_walk.emplace_back("RoMoCoSwingPosition");
    taskOrder_walk.emplace_back("RoMoCoSwingOrientation");
    taskOrder_walk.emplace_back("WaistTrack");
    taskOrder_walk.emplace_back("HandTrackJoints");

    kin_tasks_walk.buildPriority(taskOrder_walk);

    ///-------- stand ------------
    kin_tasks_stand.addTask("static_Contact");
    kin_tasks_stand.addTask("CoMTrack");
    kin_tasks_stand.addTask("HandTrackJoints");
    kin_tasks_stand.addTask("HipRPY");
    kin_tasks_stand.addTask("HeadRP");
    kin_tasks_stand.addTask("Pz");
    kin_tasks_stand.addTask("CoMXY_HipRPY");
    kin_tasks_stand.addTask("Roll_Pitch_Yaw");
    kin_tasks_stand.addTask("WaistTrack");

    std::vector<std::string> taskOrder_stand;

    taskOrder_stand.emplace_back("static_Contact");
    taskOrder_stand.emplace_back("CoMXY_HipRPY");
    taskOrder_stand.emplace_back("Pz");
    taskOrder_stand.emplace_back("HandTrackJoints");
    taskOrder_stand.emplace_back("HeadRP");

    kin_tasks_stand.buildPriority(taskOrder_stand);
}

void WBC_priority::dataBusRead(const DataBus &robotState)
{
    // foot-end offset posture
    fe_L_rot_L_off = robotState.fe_L_rot_L_off;
    fe_R_rot_L_off = robotState.fe_R_rot_L_off;

    // deisred values
    base_rpy_des = robotState.base_rpy_des;
    base_rpy_cur << robotState.rpy[0], robotState.rpy[1], robotState.rpy[2];
    base_pos_des = robotState.base_pos_des;
    swing_fe_pos_des_W = robotState.swing_fe_pos_des_W;
    swing_fe_rpy_des_W = robotState.swing_fe_rpy_des_W;
    stance_fe_pos_cur_W = robotState.stance_fe_pos_cur_W;
    stance_fe_rot_cur_W = robotState.stance_fe_rot_cur_W;
    stanceDesPos_W = robotState.stanceDesPos_W;
    hd_l_pos_cur_W = robotState.hd_l_pos_W;
    hd_r_pos_cur_W = robotState.hd_r_pos_W;
    hd_l_rot_cur_W = robotState.hd_l_rot_W;
    hd_r_rot_cur_W = robotState.hd_r_rot_W;
    fe_l_pos_cur_W = robotState.fe_l_pos_W;
    fe_r_pos_cur_W = robotState.fe_r_pos_W;
    fe_l_rot_cur_W = robotState.fe_l_rot_W;
    fe_r_rot_cur_W = robotState.fe_r_rot_W;
    des_ddq = robotState.des_ddq;
    des_dq = robotState.des_dq;
    des_delta_q = robotState.des_delta_q;
    des_q = robotState.des_q;

    // state update
    J_base = robotState.J_base;
    dJ_base = robotState.dJ_base;
    base_rot = robotState.base_rot;
    base_pos = robotState.base_pos;
    hip_link_pos = robotState.hip_link_pos;
    hip_link_rot = robotState.hip_link_rot;
    J_hip_link = robotState.J_hip_link;

    Jfe = Eigen::MatrixXd::Zero(12, model_nv);
    Jfe.block(0, 0, 6, model_nv) = robotState.J_l;
    Jfe.block(6, 0, 6, model_nv) = robotState.J_r;
    Jfe_foot = Eigen::MatrixXd::Zero(12, model_nv);
    Jfe_foot.topRows(6) = BuildPlaneFootContactJacobian(robotState.J_l, robotState.fe_l_rot_W);
    Jfe_foot.bottomRows(6) = BuildPlaneFootContactJacobian(robotState.J_r, robotState.fe_r_rot_W);
    dJfe = Eigen::MatrixXd::Zero(12, model_nv);
    dJfe.block(0, 0, 6, model_nv) = robotState.dJ_l;
    dJfe.block(6, 0, 6, model_nv) = robotState.dJ_r;
    dJfe_foot = Eigen::MatrixXd::Zero(12, model_nv);
    dJfe_foot.topRows(6) = BuildPlaneFootContactTimeVariation(robotState.dJ_l, robotState.fe_l_rot_W);
    dJfe_foot.bottomRows(6) = BuildPlaneFootContactTimeVariation(robotState.dJ_r, robotState.fe_r_rot_W);
    J_hd_l = robotState.J_hd_l;
    J_hd_r = robotState.J_hd_r;
    dJ_hd_l = robotState.J_hd_l;
    dJ_hd_r = robotState.J_hd_r;
    Fr_ff = robotState.Fr_ff;
    dyn_M = robotState.dyn_M;
    dyn_M_inv = robotState.dyn_M_inv;
    dyn_Ag = robotState.dyn_Ag;
    dyn_dAg = robotState.dyn_dAg;
    dyn_Non = robotState.dyn_Non;
    dq = robotState.dq;
    q = robotState.q;
    walk_yd = robotState.walk_yd;
    walk_dyd = robotState.walk_dyd;
    walk_d2yd = robotState.walk_d2yd;
    walk_target_yaw = robotState.walk_target_yaw;
    walk_target_yaw_rate = robotState.walk_target_yaw_rate;
    legStateCur = robotState.legState;
    motionStateCur = robotState.motionState;

    const DataBus::LegState activeStance =
        (legStateCur == DataBus::DSt) ? robotState.walk_stance_leg : legStateCur;

    if (activeStance == DataBus::LSt)
    {
        Jc = robotState.J_l;
        Jc_foot = Jfe_foot.topRows(6);
        dJc = robotState.dJ_l;
        dJc_foot = dJfe_foot.topRows(6);
        Jsw = robotState.J_r;
        dJsw = robotState.dJ_r;
        fe_pos_sw_W = robotState.fe_r_pos_W;
        fe_rot_sw_W = robotState.fe_r_rot_W;
    }
    else
    {
        Jc = robotState.J_r;
        Jc_foot = Jfe_foot.bottomRows(6);
        dJc = robotState.dJ_r;
        dJc_foot = dJfe_foot.bottomRows(6);
        Jsw = robotState.J_l;
        dJsw = robotState.dJ_l;
        fe_pos_sw_W = robotState.fe_l_pos_W;
        fe_rot_sw_W = robotState.fe_l_rot_W;
    }

    Jcom = robotState.Jcom_W;
    dJcom = robotState.dJcom_W;
    pCoMCur = robotState.pCoM_W;
    if (!pCoMDesInitialized)
    {
        pCoMDes = pCoMCur;
        pCoMDesInitialized = true;
    }
}

void WBC_priority::dataBusWrite(DataBus &robotState)
{
    robotState.wbc_ddq_final = eigen_ddq_Opt;
    robotState.wbc_tauJointRes = tauJointRes;
    robotState.wbc_FrRes = eigen_fr_Opt;
    robotState.qp_cpuTime = cpu_time;
    robotState.qp_nWSR = nWSR;
    robotState.qp_status = qpStatus;

    robotState.wbc_delta_q_final = delta_q_final_kin;
    robotState.wbc_dq_final = dq_final_kin;
    robotState.wbc_ddq_final = ddq_final_kin;

    robotState.qp_status = qpStatus;
    robotState.qp_nWSR = nWSR;
    robotState.qp_cpuTime = cpu_time;
}

// QP problem contains joint torque, QP_nv=6+12, QP_nc=22;
void WBC_priority::computeTau()
{
    Eigen::MatrixXd Jfe_qp = Jfe;
    Eigen::MatrixXd dJfe_qp = dJfe;
    Eigen::VectorXd Fr_ff_qp = Fr_ff;
    if (motionStateCur == DataBus::Walk || motionStateCur == DataBus::Walk2Stand)
    {
        Jfe_qp = Jfe_foot;
        dJfe_qp = dJfe_foot;
        Fr_ff_qp = Eigen::VectorXd::Zero(12);
        auto distributeVerticalLoad = [](double fz)
        {
            Eigen::Matrix<double, 6, 1> f;
            const double frontTotal = -kFootContactBackX / (kFootContactFrontX - kFootContactBackX) * fz;
            const double back = fz - frontTotal;
            f << 0.0, 0.0, 0.5 * frontTotal, 0.0, 0.5 * frontTotal, back;
            return f;
        };
        Fr_ff_qp.segment<6>(0) = distributeVerticalLoad(Fr_ff(2));
        Fr_ff_qp.segment<6>(6) = distributeVerticalLoad(Fr_ff(8));
    }

    if (motionStateCur == DataBus::Walk || motionStateCur == DataBus::Walk2Stand)
    {
        const bool doubleSupport = (motionStateCur == DataBus::Stand || legStateCur == DataBus::DSt);
        const bool leftStance = (legStateCur == DataBus::LSt);
        const int nu = model_nv - 6;
        const int nf = doubleSupport ? 12 : 6;
        const int nvar = model_nv + nu + nf;
        const int neq = model_nv + nf;
        const int nfric = doubleSupport ? 22 : 11;
        const int nc = neq + nfric;

        Eigen::MatrixXd Jact = Eigen::MatrixXd::Zero(nf, model_nv);
        Eigen::MatrixXd dJact = Eigen::MatrixXd::Zero(nf, model_nv);
        Eigen::VectorXd fRef = Eigen::VectorXd::Zero(nf);
        if (doubleSupport)
        {
            Jact = Jfe_qp;
            dJact = dJfe_qp;
            fRef = Fr_ff_qp;
        }
        else if (leftStance)
        {
            Jact = Jfe_qp.topRows(6);
            dJact = dJfe_qp.topRows(6);
            fRef = Fr_ff_qp.segment(0, 6);
        }
        else
        {
            Jact = Jfe_qp.bottomRows(6);
            dJact = dJfe_qp.bottomRows(6);
            fRef = Fr_ff_qp.segment(6, 6);
        }

        const Eigen::Matrix<double, 11, 6> footCone =
            RoMoCoPlaneFootCone(miu, foot_half_width, foot_l_front, foot_l_back, foot_yaw_friction);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nvar, nvar);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(nvar);

        auto addAccelerationTaskCost = [&](const Task &task, const Eigen::VectorXd &rowWeights)
        {
            if (task.J.rows() == 0 || rowWeights.size() != task.J.rows())
            {
                return;
            }
            const Eigen::VectorXd ddxCmd = task.ddxDes + task.kp * task.errX + task.kd * task.derrX;
            const Eigen::VectorXd residualBias = task.dJ * dq - ddxCmd;
            const Eigen::MatrixXd W = rowWeights.asDiagonal();
            H.block(0, 0, model_nv, model_nv).noalias() += 2.0 * task.J.transpose() * W * task.J;
            g.head(model_nv).noalias() += 2.0 * task.J.transpose() * W * residualBias;
        };

        const int supportOutputId = kin_tasks_walk.getId("RoMoCoSupportOutputs");
        if (supportOutputId >= 0)
        {
            const Eigen::VectorXd taskWeights =
                (Eigen::VectorXd(4) << 30.0, 5.0, 20.0, 20.0).finished();
            addAccelerationTaskCost(kin_tasks_walk.taskLib[supportOutputId], taskWeights);
        }
        const int baseYawId = kin_tasks_walk.getId("BaseYaw");
        if (baseYawId >= 0)
        {
            addAccelerationTaskCost(kin_tasks_walk.taskLib[baseYawId], Eigen::VectorXd::Constant(1, 10.0));
        }
        const int swingPositionId = kin_tasks_walk.getId("RoMoCoSwingPosition");
        if (swingPositionId >= 0)
        {
            const Eigen::VectorXd taskWeights =
                (Eigen::VectorXd(3) << 20.0, 20.0, 30.0).finished();
            addAccelerationTaskCost(kin_tasks_walk.taskLib[swingPositionId], taskWeights);
        }
        const int swingOrientationId = kin_tasks_walk.getId("RoMoCoSwingOrientation");
        if (swingOrientationId >= 0)
        {
            const Eigen::VectorXd taskWeights =
                (Eigen::VectorXd(3) << 5.0, 8.0, 8.0).finished();
            addAccelerationTaskCost(kin_tasks_walk.taskLib[swingOrientationId], taskWeights);
        }

        H.diagonal().head(model_nv).array() += 2.0e-6;
        H.diagonal().segment(model_nv, nu).array() += 2.0e-6;
        H.diagonal().tail(nf).array() += 2.0e-3;
        g.tail(nf).array() -= 2.0e-3 * fRef.array();

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nc, nvar);
        Eigen::VectorXd lbA = Eigen::VectorXd::Constant(nc, -1e10);
        Eigen::VectorXd ubA = Eigen::VectorXd::Constant(nc, 1e10);

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(model_nv, nu);
        B.block(6, 0, nu, nu) = Eigen::MatrixXd::Identity(nu, nu);
        A.block(0, 0, model_nv, model_nv) = dyn_M;
        A.block(0, model_nv, model_nv, nu) = -B;
        A.block(0, model_nv + nu, model_nv, nf) = -Jact.transpose();
        lbA.segment(0, model_nv) = -dyn_Non;
        ubA.segment(0, model_nv) = -dyn_Non;

        A.block(model_nv, 0, nf, model_nv) = Jact;
        const Eigen::VectorXd contactAccRhs = -dJact * dq;
        lbA.segment(model_nv, nf) = contactAccRhs;
        ubA.segment(model_nv, nf) = contactAccRhs;

        if (doubleSupport)
        {
            A.block(neq, model_nv + nu, 11, 6) = footCone;
            A.block(neq + 11, model_nv + nu + 6, 11, 6) = footCone;
            ubA.segment(neq, 22).setZero();
            ubA(neq) = -f_z_low;
            ubA(neq + 11) = -f_z_low;
        }
        else
        {
            A.block(neq, model_nv + nu, 11, 6) = footCone;
            ubA.segment(neq, 11).setZero();
            ubA(neq) = -f_z_low;
        }

        std::vector<qpOASES::real_t> qpH(nvar * nvar), qpg(nvar), qpA(nc * nvar), qplbA(nc), qpubA(nc);
        for (int i = 0; i < nvar; ++i)
        {
            qpg[i] = g(i);
            for (int j = 0; j < nvar; ++j)
            {
                qpH[i * nvar + j] = H(i, j);
            }
        }
        for (int i = 0; i < nc; ++i)
        {
            qplbA[i] = lbA(i);
            qpubA[i] = ubA(i);
            for (int j = 0; j < nvar; ++j)
            {
                qpA[i * nvar + j] = A(i, j);
            }
        }

        qpOASES::QProblem idProb(nvar, nc);
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_NONE;
        idProb.setOptions(options);
        qpOASES::int_t nwsr = 200;
        qpOASES::real_t cput = timeStep;
        qpOASES::returnValue res = idProb.init(qpH.data(), qpg.data(), qpA.data(), nullptr, nullptr,
                                               qplbA.data(), qpubA.data(), nwsr, &cput);
        qpStatus = qpOASES::getSimpleStatus(res);
        nWSR = nwsr;
        cpu_time = cput;

        std::vector<qpOASES::real_t> sol(nvar, 0.0);
        idProb.getPrimalSolution(sol.data());
        if (res == qpOASES::SUCCESSFUL_RETURN)
        {
            eigen_ddq_Opt = Eigen::VectorXd::Zero(model_nv);
            tauJointRes = Eigen::VectorXd::Zero(nu);
            for (int i = 0; i < model_nv; ++i)
            {
                eigen_ddq_Opt(i) = sol[i];
            }
            for (int i = 0; i < nu; ++i)
            {
                tauJointRes(i) = sol[model_nv + i];
            }
            eigen_fr_Opt = Eigen::VectorXd::Zero(12);
            if (doubleSupport)
            {
                for (int i = 0; i < 12; ++i)
                {
                    eigen_fr_Opt(i) = sol[model_nv + nu + i];
                }
            }
            else if (leftStance)
            {
                for (int i = 0; i < 6; ++i)
                {
                    eigen_fr_Opt(i) = sol[model_nv + nu + i];
                }
            }
            else
            {
                for (int i = 0; i < 6; ++i)
                {
                    eigen_fr_Opt(6 + i) = sol[model_nv + nu + i];
                }
            }
        }
        else
        {
            eigen_ddq_Opt = ddq_final_kin;
            eigen_fr_Opt = Fr_ff_qp;
            Eigen::VectorXd tauRes = dyn_M * eigen_ddq_Opt + dyn_Non - Jfe_qp.transpose() * eigen_fr_Opt;
            tauJointRes = tauRes.block(6, 0, model_nv - 6, 1);
        }

        const Eigen::Matrix<double, 6, 6> H_contact = PlaneContactForceToWrenchMap();
        Eigen::VectorXd wrenchForLog = Eigen::VectorXd::Zero(12);
        wrenchForLog.segment<6>(0) = H_contact * eigen_fr_Opt.segment<6>(0);
        wrenchForLog.segment<6>(6) = H_contact * eigen_fr_Opt.segment<6>(6);
        eigen_fr_Opt = wrenchForLog;

        last_nWSR = nWSR;
        last_cpu_time = cpu_time;
        return;
    }

    // constust the QP problem, refer to the md file for more details
    Eigen::MatrixXd eigen_qp_A1 = Eigen::MatrixXd::Zero(6, QP_nv); // 18 means the sum of dims of delta_r and delta_Fr
    eigen_qp_A1.block<6, 6>(0, 0) = Sf * dyn_M * St_qpV1;

    eigen_qp_A1.block<6, 12>(0, 6) = -Sf * Jfe_qp.transpose();

    Eigen::VectorXd eqRes = Eigen::VectorXd::Zero(6);
    eqRes = -Sf * dyn_M * ddq_final_kin - Sf * dyn_Non + Sf * Jfe_qp.transpose() * Fr_ff_qp;

    const Eigen::Matrix<double, 11, 6> footConeLocal =
        RoMoCoWrenchCone(miu, foot_half_width, foot_l_front, foot_l_back, foot_yaw_friction);

    const bool leftContact = (motionStateCur == DataBus::Stand) ||
                             (motionStateCur == DataBus::Walk && legStateCur == DataBus::LSt) ||
                             (motionStateCur == DataBus::Walk2Stand && legStateCur != DataBus::RSt);
    const bool rightContact = (motionStateCur == DataBus::Stand) ||
                              (motionStateCur == DataBus::Walk && legStateCur == DataBus::RSt) ||
                              (motionStateCur == DataBus::Walk2Stand && legStateCur != DataBus::LSt);

    Eigen::MatrixXd frictionA = Eigen::MatrixXd::Zero(22, QP_nv);
    Eigen::VectorXd frictionLow = Eigen::VectorXd::Constant(22, -1e10);
    Eigen::VectorXd frictionUpp = Eigen::VectorXd::Constant(22, 1e10);

    auto addFootCone = [&](int footOffset, int rowOffset)
    {
        Eigen::Matrix<double, 11, 12> coneWorld = Eigen::Matrix<double, 11, 12>::Zero();
        coneWorld.block<11, 6>(0, footOffset) = footConeLocal;

        Eigen::VectorXd coneUpper = Eigen::VectorXd::Zero(11);
        coneUpper(0) = -f_z_low;

        frictionA.block(rowOffset, 6, 11, 12) = coneWorld;
        frictionUpp.segment(rowOffset, 11) = coneUpper - coneWorld * Fr_ff_qp;
    };

    if (leftContact)
    {
        addFootCone(0, 0);
    }
    if (rightContact)
    {
        addFootCone(6, 11);
    }

    Eigen::MatrixXd swingZeroA = Eigen::MatrixXd::Zero(6, QP_nv);
    Eigen::VectorXd swingZeroLow = Eigen::VectorXd::Constant(6, -1e10);
    Eigen::VectorXd swingZeroUpp = Eigen::VectorXd::Constant(6, 1e10);
    if (motionStateCur == DataBus::Walk || motionStateCur == DataBus::Walk2Stand)
    {
        if (legStateCur == DataBus::LSt)
        {
            swingZeroA.block<6, 6>(0, 12) = Eigen::Matrix<double, 6, 6>::Identity();
            swingZeroLow = -Fr_ff_qp.segment<6>(6);
            swingZeroUpp = swingZeroLow;
        }
        else if (legStateCur == DataBus::RSt)
        {
            swingZeroA.block<6, 6>(0, 6) = Eigen::Matrix<double, 6, 6>::Identity();
            swingZeroLow = -Fr_ff_qp.segment<6>(0);
            swingZeroUpp = swingZeroLow;
        }
    }

    Eigen::MatrixXd eigen_qp_A_final = Eigen::MatrixXd::Zero(QP_nc, QP_nv);
    eigen_qp_A_final.block(0, 0, 6, QP_nv) = eigen_qp_A1;
    eigen_qp_A_final.block(6, 0, 22, QP_nv) = frictionA;
    eigen_qp_A_final.block(28, 0, 6, QP_nv) = swingZeroA;

    Eigen::VectorXd eigen_qp_lbA = Eigen::VectorXd::Constant(QP_nc, -1e10);
    Eigen::VectorXd eigen_qp_ubA = Eigen::VectorXd::Constant(QP_nc, 1e10);

    eigen_qp_lbA.segment(0, 6) = eqRes;
    eigen_qp_lbA.segment(6, 22) = frictionLow;
    eigen_qp_lbA.segment(28, 6) = swingZeroLow;
    eigen_qp_ubA.segment(0, 6) = eqRes;
    eigen_qp_ubA.segment(6, 22) = frictionUpp;
    eigen_qp_ubA.segment(28, 6) = swingZeroUpp;

    Eigen::MatrixXd eigen_qp_H = Eigen::MatrixXd::Zero(QP_nv, QP_nv);
    Q2 = Eigen::MatrixXd::Identity(6, 6);
    Q1 = Eigen::MatrixXd::Identity(12, 12);
	if (motionStateCur == DataBus::Stand){
		eigen_qp_H.block<6, 6>(0, 0) = Q2 * 2.0 * 1e7;
		eigen_qp_H.block<12, 12>(6, 6) = Q1 * 2.0 * 1e1;
		eigen_qp_H(9,9) *= 100;
		eigen_qp_H(10,10) *= 100;
		eigen_qp_H(15,15) *= 100;
		eigen_qp_H(16,16) *= 100;
	}
	else{
		eigen_qp_H.block<6, 6>(0, 0) = Q2 * 2.0 * 1e7;
		eigen_qp_H.block<12, 12>(6, 6) = Q1 * 2.0 * 1e1;
	}

    // obj: (1/2)x'Hx+x'g
    // s.t. lbA<=Ax<=ubA
    //       lb<=x<=ub
    //    qpOASES::real_t qp_H[QP_nv*QP_nv];
    //    qpOASES::real_t qp_A[QP_nc*QP_nv];
    //    qpOASES::real_t qp_g[QP_nv];
    //    qpOASES::real_t qp_lbA[QP_nc];
    //    qpOASES::real_t qp_ubA[QP_nc];
    //    qpOASES::real_t xOpt_iniGuess[QP_nv];

    copy_Eigen_to_real_t(qp_H, eigen_qp_H, eigen_qp_H.rows(), eigen_qp_H.cols());
    copy_Eigen_to_real_t(qp_A, eigen_qp_A_final, eigen_qp_A_final.rows(), eigen_qp_A_final.cols());
    copy_Eigen_to_real_t(qp_lbA, eigen_qp_lbA, eigen_qp_lbA.rows(), eigen_qp_lbA.cols());
    copy_Eigen_to_real_t(qp_ubA, eigen_qp_ubA, eigen_qp_ubA.rows(), eigen_qp_ubA.cols());

    qpOASES::returnValue res;
    for (int i = 0; i < QP_nv; i++)
    {
        xOpt_iniGuess[i] = 0;
        //        xOpt_iniGuess[i] =eigen_xOpt(i);
        qp_g[i] = 0;
    }
    nWSR = 200;
    cpu_time = timeStep;
    //    QP_prob.reset();
    res = QP_prob.init(qp_H, qp_g, qp_A, NULL, NULL, qp_lbA, qp_ubA, nWSR, &cpu_time, xOpt_iniGuess);
    qpStatus = qpOASES::getSimpleStatus(res);
    //    if (res==qpOASES::SUCCESSFUL_RETURN)
    //        printf("WBC-QP: successful_return\n");
    //    else if (res==qpOASES::RET_MAX_NWSR_REACHED)
    //        printf("WBC-QP: max_nwsr\n");
    //    else if (res==qpOASES::RET_INIT_FAILED)
    //        printf("WBC-QP: init_failed\n");

    qpOASES::real_t xOpt[QP_nv];
    QP_prob.getPrimalSolution(xOpt);
    if (res == qpOASES::SUCCESSFUL_RETURN)
        for (int i = 0; i < QP_nv; i++)
            eigen_xOpt(i) = xOpt[i];

    eigen_ddq_Opt = ddq_final_kin;
    eigen_ddq_Opt.block<6, 1>(0, 0) += eigen_xOpt.block<6, 1>(0, 0);
    eigen_fr_Opt = Fr_ff_qp + eigen_xOpt.block<12, 1>(6, 0);

    if (qpStatus != 0)
    {
        Eigen::MatrixXd A_x;
        Eigen::VectorXd xOpt_iniGuess_m(QP_nv, 1);
        for (int i = 0; i < QP_nv; i++)
            xOpt_iniGuess_m(i) = xOpt_iniGuess[i];
    }

    Eigen::VectorXd tauRes;
    tauRes = dyn_M * eigen_ddq_Opt + dyn_Non - Jfe_qp.transpose() * eigen_fr_Opt;

    tauJointRes = tauRes.block(6, 0, model_nv - 6, 1);

    last_nWSR = nWSR;
    last_cpu_time = cpu_time;
}

void WBC_priority::computeDdq(Pin_KinDyn &pinKinDynIn)
{
    // task definition
    /// -------- walk -------------
    {
        int id = kin_tasks_walk.getId("static_Contact");
        const bool doubleSupportWalk = (legStateCur == DataBus::DSt);
        const int contactRows = doubleSupportWalk ? 12 : 6;
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(contactRows);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(contactRows);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(contactRows);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(contactRows);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(contactRows, contactRows) * 0;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(contactRows, contactRows) * 0;
        kin_tasks_walk.taskLib[id].J = doubleSupportWalk ? Jfe_foot : Jc_foot;
        kin_tasks_walk.taskLib[id].J.block(0, kWaistYawQ - 1, contactRows, 3).setZero();
        kin_tasks_walk.taskLib[id].dJ = doubleSupportWalk ? dJfe_foot : dJc_foot;
        kin_tasks_walk.taskLib[id].dJ.block(0, kWaistYawQ - 1, contactRows, 3).setZero();
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("RoMoCoWalkingOutputs");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(10);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(10);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(10);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(10);

        const bool leftStance = (legStateCur == DataBus::LSt);
        const Eigen::Vector3d swingToStance = fe_pos_sw_W - stance_fe_pos_cur_W;
        Eigen::Vector3d swingToStanceDes = swing_fe_pos_des_W - stanceDesPos_W;
        Eigen::Vector3d swingToStanceVelDes = Eigen::Vector3d::Zero();
        Eigen::Vector3d swingToStanceAccDes = Eigen::Vector3d::Zero();
        if (walk_yd.size() >= 7)
        {
            swingToStanceDes = walk_yd.segment<3>(4);
        }
        if (walk_dyd.size() >= 7)
        {
            swingToStanceVelDes = walk_dyd.segment<3>(4);
        }
        if (walk_d2yd.size() >= 7)
        {
            swingToStanceAccDes = walk_d2yd.segment<3>(4);
        }
        const Eigen::VectorXd comVel = Jcom * dq;
        Eigen::Matrix3d desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        const Eigen::Vector3d baseRotErr = diffRot(base_rot, desRot);
        const int stanceHipYawQ = leftStance ? kLeftHipYawQ : kRightHipYawQ;
        const int swingHipYawQ = leftStance ? kRightHipYawQ : kLeftHipYawQ;
        const Eigen::MatrixXd JcomToStance = Jcom.topRows(3) - Jc.topRows(3);
        const Eigen::MatrixXd dJcomToStance = -dJc.topRows(3);
        const Eigen::MatrixXd JswingToStance = Jsw.topRows(3) - Jc.topRows(3);
        const Eigen::MatrixXd dJswingToStance = dJsw.topRows(3) - dJc.topRows(3);

        kin_tasks_walk.taskLib[id].errX(0) =
            ReferenceValue(walk_yd, 0, pCoMDes(2) - stance_fe_pos_cur_W(2)) -
            (pCoMCur(2) - stance_fe_pos_cur_W(2));
        kin_tasks_walk.taskLib[id].errX(1) =
            ReferenceValue(walk_yd, 1, qIniDes(stanceHipYawQ)) - q(stanceHipYawQ);
        kin_tasks_walk.taskLib[id].errX(2) =
            ReferenceValue(walk_yd, 2, 0.0) - base_rpy_cur(1);
        kin_tasks_walk.taskLib[id].errX(3) =
            ReferenceValue(walk_yd, 3, 0.0) - base_rpy_cur(0);
        kin_tasks_walk.taskLib[id].errX.segment<3>(4) = swingToStanceDes - swingToStance;
        kin_tasks_walk.taskLib[id].errX(7) =
            ReferenceValue(walk_yd, 7, qIniDes(swingHipYawQ)) - q(swingHipYawQ);
        Eigen::Matrix3d swingDesRot = eul2Rot(0.0, 0.0, swing_fe_rpy_des_W(2));
        const Eigen::Vector3d swingRotErr = diffRot(fe_rot_sw_W, swingDesRot);
        kin_tasks_walk.taskLib[id].errX(8) = ReferenceValue(walk_yd, 8, 0.0) + swingRotErr(1);
        kin_tasks_walk.taskLib[id].errX(9) = ReferenceValue(walk_yd, 9, 0.0) + swingRotErr(0);

        kin_tasks_walk.taskLib[id].derrX(0) =
            ReferenceValue(walk_dyd, 0, 0.0) - (JcomToStance.row(2) * dq)(0);
        kin_tasks_walk.taskLib[id].derrX(1) =
            ReferenceValue(walk_dyd, 1, 0.0) - dq(stanceHipYawQ - 1);
        kin_tasks_walk.taskLib[id].derrX(2) =
            ReferenceValue(walk_dyd, 2, 0.0) - dq(4);
        kin_tasks_walk.taskLib[id].derrX(3) =
            ReferenceValue(walk_dyd, 3, 0.0) - dq(3);
        kin_tasks_walk.taskLib[id].derrX.segment<3>(4) =
            swingToStanceVelDes - JswingToStance * dq;
        kin_tasks_walk.taskLib[id].derrX(7) =
            ReferenceValue(walk_dyd, 7, 0.0) - dq(swingHipYawQ - 1);
        kin_tasks_walk.taskLib[id].derrX(8) =
            ReferenceValue(walk_dyd, 8, 0.0) - (Jsw.row(4) * dq)(0);
        kin_tasks_walk.taskLib[id].derrX(9) =
            ReferenceValue(walk_dyd, 9, 0.0) - (Jsw.row(3) * dq)(0);

        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Zero(10, 10);
        kin_tasks_walk.taskLib[id].kp.diagonal() << 450.0, 220.0, 300.0, 300.0, 450.0, 450.0, 450.0, 220.0, 220.0, 220.0;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Zero(10, 10);
        kin_tasks_walk.taskLib[id].kd.diagonal() << 35.0, 20.0, 25.0, 25.0, 30.0, 30.0, 30.0, 20.0, 20.0, 20.0;
        kin_tasks_walk.taskLib[id].ddxDes.setZero();
        kin_tasks_walk.taskLib[id].ddxDes(0) = ReferenceValue(walk_d2yd, 0, 0.0) - dJcom(2);
        kin_tasks_walk.taskLib[id].ddxDes(1) = ReferenceValue(walk_d2yd, 1, 0.0);
        kin_tasks_walk.taskLib[id].ddxDes(2) = ReferenceValue(walk_d2yd, 2, 0.0);
        kin_tasks_walk.taskLib[id].ddxDes(3) = ReferenceValue(walk_d2yd, 3, 0.0);
        kin_tasks_walk.taskLib[id].ddxDes.segment<3>(4) = swingToStanceAccDes;
        kin_tasks_walk.taskLib[id].ddxDes(7) = ReferenceValue(walk_d2yd, 7, 0.0);
        kin_tasks_walk.taskLib[id].ddxDes(8) = ReferenceValue(walk_d2yd, 8, 0.0);
        kin_tasks_walk.taskLib[id].ddxDes(9) = ReferenceValue(walk_d2yd, 9, 0.0);
        kin_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(10, model_nv);
        kin_tasks_walk.taskLib[id].J.row(0) = JcomToStance.row(2);
        SetJointSelector(kin_tasks_walk.taskLib[id].J, 1, stanceHipYawQ);
        kin_tasks_walk.taskLib[id].J.row(2) = J_base.row(4);
        kin_tasks_walk.taskLib[id].J.row(3) = J_base.row(3);
        kin_tasks_walk.taskLib[id].J.middleRows(4, 3) = JswingToStance;
        SetJointSelector(kin_tasks_walk.taskLib[id].J, 7, swingHipYawQ);
        kin_tasks_walk.taskLib[id].J.row(8) = Jsw.row(4);
        kin_tasks_walk.taskLib[id].J.row(9) = Jsw.row(3);
        kin_tasks_walk.taskLib[id].J.block(0, kWaistYawQ - 1, 10, 3).setZero();
        kin_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(10, model_nv);
        kin_tasks_walk.taskLib[id].dJ.row(0) = dJcomToStance.row(2);
        kin_tasks_walk.taskLib[id].dJ.row(2) = dJ_base.row(4);
        kin_tasks_walk.taskLib[id].dJ.row(3) = dJ_base.row(3);
        kin_tasks_walk.taskLib[id].dJ.middleRows(4, 3) = dJswingToStance;
        kin_tasks_walk.taskLib[id].dJ.row(8) = dJsw.row(4);
        kin_tasks_walk.taskLib[id].dJ.row(9) = dJsw.row(3);
        kin_tasks_walk.taskLib[id].dJ.block(0, kWaistYawQ - 1, 10, 3).setZero();
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
        kin_tasks_walk.taskLib[id].W.diagonal()(kWaistYawQ - 1) = 20.0;
        kin_tasks_walk.taskLib[id].W.diagonal()(kWaistRollQ - 1) = 20.0;
        kin_tasks_walk.taskLib[id].W.diagonal()(kWaistPitchQ - 1) = 20.0;

        int supportId = kin_tasks_walk.getId("RoMoCoSupportOutputs");
        kin_tasks_walk.taskLib[supportId].errX = kin_tasks_walk.taskLib[id].errX.head(4);
        kin_tasks_walk.taskLib[supportId].derrX = kin_tasks_walk.taskLib[id].derrX.head(4);
        kin_tasks_walk.taskLib[supportId].ddxDes = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[supportId].dxDes = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[supportId].kp = kin_tasks_walk.taskLib[id].kp.topLeftCorner(4, 4);
        kin_tasks_walk.taskLib[supportId].kd = kin_tasks_walk.taskLib[id].kd.topLeftCorner(4, 4);
        kin_tasks_walk.taskLib[supportId].J = kin_tasks_walk.taskLib[id].J.topRows(4);
        kin_tasks_walk.taskLib[supportId].dJ = kin_tasks_walk.taskLib[id].dJ.topRows(4);
        kin_tasks_walk.taskLib[supportId].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
        kin_tasks_walk.taskLib[supportId].W.diagonal()(kWaistYawQ - 1) = 20.0;
        kin_tasks_walk.taskLib[supportId].W.diagonal()(kWaistRollQ - 1) = 20.0;
        kin_tasks_walk.taskLib[supportId].W.diagonal()(kWaistPitchQ - 1) = 20.0;

        int swingPosId = kin_tasks_walk.getId("RoMoCoSwingPosition");
        kin_tasks_walk.taskLib[swingPosId].errX = kin_tasks_walk.taskLib[id].errX.segment(4, 3);
        kin_tasks_walk.taskLib[swingPosId].derrX = kin_tasks_walk.taskLib[id].derrX.segment(4, 3);
        kin_tasks_walk.taskLib[swingPosId].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[swingPosId].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[swingPosId].kp = kin_tasks_walk.taskLib[id].kp.block(4, 4, 3, 3);
        kin_tasks_walk.taskLib[swingPosId].kd = kin_tasks_walk.taskLib[id].kd.block(4, 4, 3, 3);
        kin_tasks_walk.taskLib[swingPosId].J = kin_tasks_walk.taskLib[id].J.middleRows(4, 3);
        kin_tasks_walk.taskLib[swingPosId].dJ = kin_tasks_walk.taskLib[id].dJ.middleRows(4, 3);
        kin_tasks_walk.taskLib[swingPosId].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        int swingOriId = kin_tasks_walk.getId("RoMoCoSwingOrientation");
        kin_tasks_walk.taskLib[swingOriId].errX = kin_tasks_walk.taskLib[id].errX.segment(7, 3);
        kin_tasks_walk.taskLib[swingOriId].derrX = kin_tasks_walk.taskLib[id].derrX.segment(7, 3);
        kin_tasks_walk.taskLib[swingOriId].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[swingOriId].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[swingOriId].kp = kin_tasks_walk.taskLib[id].kp.block(7, 7, 3, 3);
        kin_tasks_walk.taskLib[swingOriId].kd = kin_tasks_walk.taskLib[id].kd.block(7, 7, 3, 3);
        kin_tasks_walk.taskLib[swingOriId].J = kin_tasks_walk.taskLib[id].J.middleRows(7, 3);
        kin_tasks_walk.taskLib[swingOriId].dJ = kin_tasks_walk.taskLib[id].dJ.middleRows(7, 3);
        kin_tasks_walk.taskLib[swingOriId].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
        kin_tasks_walk.taskLib[swingOriId].W.diagonal()(kWaistYawQ - 1) = 20.0;
        kin_tasks_walk.taskLib[swingOriId].W.diagonal()(kWaistRollQ - 1) = 20.0;
        kin_tasks_walk.taskLib[swingOriId].W.diagonal()(kWaistPitchQ - 1) = 20.0;

        id = kin_tasks_walk.getId("BaseYaw");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(1);
        const double yawRef = (motionStateCur == DataBus::Walk || motionStateCur == DataBus::Walk2Stand)
                                  ? walk_target_yaw
                                  : base_rpy_des(2);
        const double yawRateRef = (motionStateCur == DataBus::Walk || motionStateCur == DataBus::Walk2Stand)
                                      ? walk_target_yaw_rate
                                      : 0.0;
        kin_tasks_walk.taskLib[id].errX(0) = WrapToPi(yawRef - base_rpy_cur(2));
        if (std::abs(kin_tasks_walk.taskLib[id].errX(0)) > 0.3)
        {
            kin_tasks_walk.taskLib[id].errX(0) = 0.3 * sign(kin_tasks_walk.taskLib[id].errX(0));
        }
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(1);
        kin_tasks_walk.taskLib[id].derrX(0) = yawRateRef - dq(5);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(1);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(1);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(1, 1) * 300.0;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(1, 1) * 30.0;
        kin_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(1, model_nv);
        kin_tasks_walk.taskLib[id].J = J_base.row(5);
        kin_tasks_walk.taskLib[id].dJ = dJ_base.row(5);
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("WaistTrack");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].errX(0) = qIniDes(kWaistYawQ) - q(kWaistYawQ);
        kin_tasks_walk.taskLib[id].errX(1) = qIniDes(kWaistRollQ) - q(kWaistRollQ);
        kin_tasks_walk.taskLib[id].errX(2) = qIniDes(kWaistPitchQ) - q(kWaistPitchQ);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 100;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 20;
        kin_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(3, model_nv);
        SetJointSelector(kin_tasks_walk.taskLib[id].J, 0, kWaistYawQ);
        SetJointSelector(kin_tasks_walk.taskLib[id].J, 1, kWaistRollQ);
        SetJointSelector(kin_tasks_walk.taskLib[id].J, 2, kWaistPitchQ);
        kin_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(3, model_nv);
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("Roll_Pitch_Yaw_Pz");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(4);
        desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(0, 0) = diffRot(base_rot, desRot);
        kin_tasks_walk.taskLib[id].errX(3) = base_pos_des(2) - q(2);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[id].derrX.block<3, 1>(0, 0) = -dq.block<3, 1>(3, 0);
        kin_tasks_walk.taskLib[id].derrX(3) = 0 - dq(2);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Zero(4, 4);
        kin_tasks_walk.taskLib[id].kp.diagonal() << 300.0, 300.0, 150.0, 450.0;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Zero(4, 4);
        kin_tasks_walk.taskLib[id].kd.diagonal() << 25.0, 25.0, 15.0, 35.0;
        Eigen::MatrixXd taskMap = Eigen::MatrixXd::Zero(4, 6);
        taskMap(0, 3) = 1;
        taskMap(1, 4) = 1;
        taskMap(2, 5) = 1;
        taskMap(3, 2) = 1;
        kin_tasks_walk.taskLib[id].J = taskMap * J_base;
        kin_tasks_walk.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("PxPy");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].errX = pCoMDes.block(0, 0, 2, 1) - pCoMCur.block(0, 0, 2, 1);
        for (int i = 0; i < 2; ++i)
        {
            if (std::abs(kin_tasks_walk.taskLib[id].errX(i)) > 0.05)
            {
                kin_tasks_walk.taskLib[id].errX(i) = 0.05 * sign(kin_tasks_walk.taskLib[id].errX(i));
            }
        }
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].derrX = des_dq.block(0, 0, 2, 1) - (Jcom * dq).block(0, 0, 2, 1);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].dxDes = des_dq.block(0, 0, 2, 1);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(2, 2) * 5;
        kin_tasks_walk.taskLib[id].kp(0, 0) = 0.0;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(2, 2) * 4;
        kin_tasks_walk.taskLib[id].kd(0, 0) = 1.0;
        kin_tasks_walk.taskLib[id].J = Jcom.block(0, 0, 2, model_nv);
        kin_tasks_walk.taskLib[id].J.block(0, kUpperBodyQStart - 1, 2, model_nv - (kUpperBodyQStart - 1)).setZero();
        kin_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(2, model_nv);
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("PosRot");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].errX.block(0, 0, 3, 1) = base_pos_des - q.block(0, 0, 3, 1);
        if (fabs(kin_tasks_walk.taskLib[id].errX(0)) >= 0.02)
            kin_tasks_walk.taskLib[id].errX(0) = 0.02 * sign(kin_tasks_walk.taskLib[id].errX(0));
        if (fabs(kin_tasks_walk.taskLib[id].errX(1)) >= 0.02)
            kin_tasks_walk.taskLib[id].errX(1) = 0.02 * sign(kin_tasks_walk.taskLib[id].errX(1));
        if (kin_tasks_walk.taskLib[id].errX(2)>0.005){
            kin_tasks_walk.taskLib[id].errX(2) = 0.005;
        }
        desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(3, 0) = diffRot(base_rot, desRot);
        kin_tasks_walk.taskLib[id].errX(4) -= 0.05 * dq(4);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(6);
        // kin_tasks_walk.taskLib[id].derrX = des_dq.block(0, 0, 6, 1) - dq.block(0, 0, 6, 1);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(6, 6) * 500;
        kin_tasks_walk.taskLib[id].kp.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 500;
        kin_tasks_walk.taskLib[id].kp(0,0) = 100;
        kin_tasks_walk.taskLib[id].kp(4,4) = 800;
        // kin_tasks_walk.taskLib[id].kp(3,3) = 800;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(6, 6) * 10;
        kin_tasks_walk.taskLib[id].kd(4,4) = 10;
        kin_tasks_walk.taskLib[id].J = J_base;
        kin_tasks_walk.taskLib[id].dJ = dJ_base;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("SwingLeg");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(0, 0) = swing_fe_pos_des_W - fe_pos_sw_W;
        desRot = eul2Rot(swing_fe_rpy_des_W(0), swing_fe_rpy_des_W(1), swing_fe_rpy_des_W(2));
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(3, 0) = diffRot(fe_rot_sw_W, desRot);      
        kin_tasks_walk.taskLib[id].errX(4) *= 2;
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(6);
        //        kin_tasks_walk.taskLib[id].derrX=-Jsw*dq;
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Zero(6, 6);
        kin_tasks_walk.taskLib[id].kp.diagonal() << 450.0, 450.0, 450.0, 220.0, 220.0, 220.0;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Zero(6, 6);
        kin_tasks_walk.taskLib[id].kd.diagonal() << 30.0, 30.0, 30.0, 20.0, 20.0, 20.0;
        kin_tasks_walk.taskLib[id].J = Jsw;
        kin_tasks_walk.taskLib[id].J.block(0, kWaistYawQ - 1, 6, 3).setZero(); // exclude waist joints
        kin_tasks_walk.taskLib[id].dJ = dJsw;
        kin_tasks_walk.taskLib[id].dJ.block(0, kWaistYawQ - 1, 6, 3).setZero(); // exclude waist joints
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        // task 6: hand track
        Eigen::VectorXd target_upper_q = qIniDes.segment(kUpperBodyQStart, kUpperBodyDof);

        id = kin_tasks_walk.getId("HandTrackJoints");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(kUpperBodyDof);
        kin_tasks_walk.taskLib[id].errX = target_upper_q - q.segment(kUpperBodyQStart, kUpperBodyDof);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(kUpperBodyDof);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(kUpperBodyDof);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(kUpperBodyDof);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(kUpperBodyDof, kUpperBodyDof) * 50;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(kUpperBodyDof, kUpperBodyDof) * 5;
        kin_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(kUpperBodyDof, model_nv);
        kin_tasks_walk.taskLib[id].J.block(0, kUpperBodyQStart - 1, kUpperBodyDof, kUpperBodyDof) = Eigen::MatrixXd::Identity(kUpperBodyDof, kUpperBodyDof);
        kin_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(kUpperBodyDof, model_nv);
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
    }

    /// -------- stand -------------
    {
        int id = kin_tasks_stand.getId("static_Contact");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(12);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(12);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(12);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(12);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(12, 12) * 0;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(12, 12) * 0;
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(12, model_nv);
        kin_tasks_stand.taskLib[id].J = Jfe;
        kin_tasks_stand.taskLib[id].J.block(0, kWaistYawQ - 1, 12, 3).setZero(); // exclude waist joints
        kin_tasks_stand.taskLib[id].dJ = dJfe;
        kin_tasks_stand.taskLib[id].dJ.block(0, kWaistYawQ - 1, 12, 3).setZero();
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("HipRPY");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        Eigen::Matrix3d desRot = eul2Rot(0, 0, 0);
        kin_tasks_stand.taskLib[id].errX.block<3, 1>(0, 0) = diffRot(hip_link_rot, desRot);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 1000;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 50;
        Eigen::MatrixXd taskMapRPY = Eigen::MatrixXd::Zero(3, 6);
        taskMapRPY(0, 3) = 1;
        taskMapRPY(1, 4) = 1;
        taskMapRPY(2, 5) = 1;
        kin_tasks_stand.taskLib[id].J = taskMapRPY * J_hip_link;
        kin_tasks_stand.taskLib[id].J.block(0, kWaistYawQ - 1, 3, model_nv - (kWaistYawQ - 1)).setZero();
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(3, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("Pz");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].errX(0) = base_pos_des(2) - q(2);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(1, 1) * 2000; // 100
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(1, 1) * 10;
        Eigen::MatrixXd taskMap = Eigen::MatrixXd::Zero(1, 6);
        taskMap(0, 2) = 1;
        kin_tasks_stand.taskLib[id].J = taskMap * J_base;
        kin_tasks_stand.taskLib[id].J.block(0, kWaistYawQ - 1, 1, 3).setZero();
        kin_tasks_stand.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_stand.taskLib[id].dJ.block(0, kWaistYawQ - 1, 1, 3).setZero();
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("CoMTrack");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].errX = pCoMDes.block(0, 0, 2, 1) - pCoMCur.block(0, 0, 2, 1);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(2, 2) * 2000; // 100
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(2, 2) * 100;
        kin_tasks_stand.taskLib[id].J = Jcom.block(0, 0, 2, model_nv);
        kin_tasks_stand.taskLib[id].J.block(0, kUpperBodyQStart - 1, 2, model_nv - (kUpperBodyQStart - 1)).setZero();
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(2, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("CoMXY_HipRPY");
        taskMapRPY = Eigen::MatrixXd::Zero(3, 6);
        taskMapRPY(0, 3) = 1;
        taskMapRPY(1, 4) = 1;
        taskMapRPY(2, 5) = 1;
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(5);
        	kin_tasks_stand.taskLib[id].errX.block(0, 0, 2, 1) = pCoMDes.block(0, 0, 2, 1) - pCoMCur.block(0, 0, 2, 1);
			desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        	kin_tasks_stand.taskLib[id].errX.block<3, 1>(2, 0) = diffRot(hip_link_rot, desRot);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(5);
//                    kin_tasks_stand.taskLib[id].derrX.block(0,0,2,1)=-(Jcom*dq).block(0,0,2,1);
//                    kin_tasks_stand.taskLib[id].derrX.block(2,0,3,1)=-taskMapRPY*J_hip_link*dq;
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(5, 5) * 250; // 100
		kin_tasks_stand.taskLib[id].kp.block(2, 2, 3, 3) = Eigen::MatrixXd::Identity(3,3)*1000;
		kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(5, 5) * 10;
        	kin_tasks_stand.taskLib[id].kd.block(2, 2, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 10;   // 100  // for hip rpy
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(5, model_nv);
        kin_tasks_stand.taskLib[id].J.block(0, 0, 2, model_nv) = Jcom.block(0, 0, 2, model_nv);
        kin_tasks_stand.taskLib[id].J.block(2, 0, 3, model_nv) = taskMapRPY * J_hip_link;
        kin_tasks_stand.taskLib[id].J.block(2, kWaistYawQ - 1, 3, model_nv - (kWaistYawQ - 1)).setZero(); // exclude upper body joints
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(5, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal()(kWaistYawQ - 1) = 200;
        kin_tasks_stand.taskLib[id].W.diagonal()(kWaistRollQ - 1) = 200;

        // define swing arm motion
        Eigen::VectorXd target_upper_q = qIniDes.segment(kUpperBodyQStart, kUpperBodyDof);

        id = kin_tasks_stand.getId("HandTrackJoints");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(kUpperBodyDof);
        kin_tasks_stand.taskLib[id].errX = target_upper_q - q.segment(kUpperBodyQStart, kUpperBodyDof);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(kUpperBodyDof);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(kUpperBodyDof);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(kUpperBodyDof);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(kUpperBodyDof, kUpperBodyDof) * 50;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(kUpperBodyDof, kUpperBodyDof) * 5;
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(kUpperBodyDof, model_nv);
        kin_tasks_stand.taskLib[id].J.block(0, kUpperBodyQStart - 1, kUpperBodyDof, kUpperBodyDof) = Eigen::MatrixXd::Identity(kUpperBodyDof, kUpperBodyDof);
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(kUpperBodyDof, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        // Enter here functions to send actuator commands, like:
        // arm-l: 0-6, arm-r: 7-13, head: 14,15, waist: 16-18, leg-l: 19-24, leg-r: 25-30

        id = kin_tasks_stand.getId("HeadRP");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].errX(0) = qIniDes(kWaistRollQ) - q(kWaistRollQ);
        kin_tasks_stand.taskLib[id].errX(1) = qIniDes(kWaistPitchQ) - q(kWaistPitchQ);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(2, 2) * 100; // 100
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(2, 2) * 10;
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(2, model_nv);
        SetJointSelector(kin_tasks_stand.taskLib[id].J, 0, kWaistRollQ);
        SetJointSelector(kin_tasks_stand.taskLib[id].J, 1, kWaistPitchQ);
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(2, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("Roll_Pitch_Yaw");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_stand.taskLib[id].errX = diffRot(base_rot, desRot);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].derrX = -dq.block<3, 1>(3, 0);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 2000;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 100;
        taskMap = Eigen::MatrixXd::Zero(3, 6);
        taskMap(0, 3) = 1;
        taskMap(1, 4) = 1;
        taskMap(2, 5) = 1;
        kin_tasks_stand.taskLib[id].J = taskMap * J_base;
        kin_tasks_stand.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("WaistTrack");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].errX(0) = qIniDes(kWaistYawQ) - q(kWaistYawQ);
        kin_tasks_stand.taskLib[id].errX(1) = qIniDes(kWaistRollQ) - q(kWaistRollQ);
        kin_tasks_stand.taskLib[id].errX(2) = qIniDes(kWaistPitchQ) - q(kWaistPitchQ);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 200;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 20;
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(3, model_nv);
        SetJointSelector(kin_tasks_stand.taskLib[id].J, 0, kWaistYawQ);
        SetJointSelector(kin_tasks_stand.taskLib[id].J, 1, kWaistRollQ);
        SetJointSelector(kin_tasks_stand.taskLib[id].J, 2, kWaistPitchQ);
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(3, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
    }

    if (motionStateCur == DataBus::Walk || motionStateCur == DataBus::Walk2Stand)
    {
        if (legStateCur == DataBus::DSt)
        {
            kin_tasks_walk.buildPriority({"static_Contact",
                                          "RoMoCoSupportOutputs",
                                          "BaseYaw",
                                          "WaistTrack",
                                          "HandTrackJoints"});
        }
        else
        {
            kin_tasks_walk.buildPriority({"static_Contact",
                                          "BaseYaw",
                                          "RoMoCoSupportOutputs",
                                          "RoMoCoSwingPosition",
                                          "RoMoCoSwingOrientation",
                                          "WaistTrack",
                                          "HandTrackJoints"});
        }
        kin_tasks_walk.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
        delta_q_final_kin = kin_tasks_walk.out_delta_q;
        dq_final_kin = kin_tasks_walk.out_dq;
        ddq_final_kin = kin_tasks_walk.out_ddq;
    }
    else if (motionStateCur == DataBus::Stand)
    {
        kin_tasks_stand.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
        delta_q_final_kin = kin_tasks_stand.out_delta_q;
        dq_final_kin = kin_tasks_stand.out_dq;
        ddq_final_kin = kin_tasks_stand.out_ddq;
    }
    else
    {
        delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
        dq_final_kin = Eigen::VectorXd::Zero(model_nv);
        ddq_final_kin = Eigen::VectorXd::Zero(model_nv);
    }

    // final WBC output collection
}

void WBC_priority::copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols)
{
    int count = 0;

    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            target[count++] = isinf(source(i, j)) ? qpOASES::INFTY : source(i, j);
        }
    }
}

void WBC_priority::setQini(const Eigen::VectorXd &qIniDesIn, const Eigen::VectorXd &qIniCurIn)
{
    qIniDes = qIniDesIn;
    qIniCur = qIniCurIn;
}

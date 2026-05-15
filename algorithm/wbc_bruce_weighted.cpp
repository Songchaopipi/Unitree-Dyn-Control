/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "wbc_bruce_weighted.h"
#include <algorithm>
#include <cmath>

namespace g1kd = g1_kin_dyn;

namespace
{
Eigen::Matrix<double, 6, 1> planeForceReferenceFromWrench(const Eigen::VectorXd &wrench)
{
    if (wrench.size() < 6 || !wrench.allFinite())
    {
        return Eigen::Matrix<double, 6, 1>::Zero();
    }

    const Eigen::Matrix<double, 6, 6> H = g1kd::planeForceToWrenchMap();
    Eigen::Matrix<double, 6, 1> planeForce = H.colPivHouseholderQr().solve(wrench);
    if (!planeForce.allFinite())
    {
        return g1kd::distributeVerticalFootLoad(std::max(0.0, wrench(2)));
    }
    return planeForce;
}
} // namespace

WBC_BruceWeighted::WBC_BruceWeighted(int model_nv_In, double miu_In, double dt)
{
    model_nv = model_nv_In;
    miu = miu_In;
    timeStep = dt;

    q = Eigen::VectorXd::Zero(model_nv + 1);
    dq = Eigen::VectorXd::Zero(model_nv);
    qIniDes = Eigen::VectorXd::Zero(model_nv + 1);
    qIniCur = Eigen::VectorXd::Zero(model_nv + 1);
    des_ddq = Eigen::VectorXd::Zero(model_nv);
    des_dq = Eigen::VectorXd::Zero(model_nv);
    des_delta_q = Eigen::VectorXd::Zero(model_nv);
    Fr_ff = Eigen::VectorXd::Zero(12);
    delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
    dq_final_kin = Eigen::VectorXd::Zero(model_nv);
    ddq_final_kin = Eigen::VectorXd::Zero(model_nv);
    tauJointRes = Eigen::VectorXd::Zero(model_nv - 6);
    eigen_ddq_Opt = Eigen::VectorXd::Zero(model_nv);
    eigen_fr_Opt = Eigen::VectorXd::Zero(12);
    pCoMDes.setZero();
}

void WBC_BruceWeighted::setQini(const Eigen::VectorXd &qIniDesIn, const Eigen::VectorXd &qIniCurIn)
{
    qIniDes = qIniDesIn;
    qIniCur = qIniCurIn;
}

void WBC_BruceWeighted::dataBusRead(const DataBus &robotState)
{
    q = robotState.q;
    dq = robotState.dq;
    des_ddq = robotState.des_ddq;
    des_dq = robotState.des_dq;
    des_delta_q = robotState.des_delta_q;
    Fr_ff = robotState.Fr_ff;
    dyn_M = robotState.dyn_M;
    dyn_M_inv = robotState.dyn_M_inv;
    dyn_Ag = robotState.dyn_Ag;
    dyn_dAg = robotState.dyn_dAg;
    dyn_Non = robotState.dyn_Non;
    J_base = robotState.J_base;
    dJ_base = robotState.dJ_base;
    Jcom = robotState.Jcom_W;
    dJcom = robotState.dJcom_W;
    J_l = robotState.J_l;
    J_r = robotState.J_r;
    dJ_l = robotState.dJ_l;
    dJ_r = robotState.dJ_r;
    base_rot = robotState.base_rot;
    base_pos = robotState.base_pos;
    base_pos_des = robotState.base_pos_des;
    base_rpy_cur = robotState.base_rpy;
    base_rpy_des = robotState.base_rpy_des;
    pCoMCur = robotState.pCoM_W;
    centroidalNmpcEnabled = robotState.centroidal_nmpc_enabled;
    centroidalComPosDes = robotState.centroidal_nmpc_com_pos_des;
    centroidalComVelDes = robotState.centroidal_nmpc_com_vel_des;
    centroidalComAccDes = robotState.centroidal_nmpc_com_acc_des;
    fe_l_pos_cur_W = robotState.fe_l_pos_W;
    fe_r_pos_cur_W = robotState.fe_r_pos_W;
    fe_l_rot_cur_W = robotState.fe_l_rot_W;
    fe_r_rot_cur_W = robotState.fe_r_rot_W;
    swing_fe_pos_des_W = robotState.swing_fe_pos_des_W;
    swing_fe_vel_des_W = robotState.swing_fe_vel_des_W;
    swing_fe_acc_des_W = robotState.swing_fe_acc_des_W;
    swing_fe_rpy_des_W = robotState.swing_fe_rpy_des_W;
    stance_fe_pos_cur_W = robotState.stance_fe_pos_cur_W;
    stanceDesPos_W = robotState.stanceDesPos_W;
    walk_yd = robotState.walk_yd;
    walk_dyd = robotState.walk_dyd;
    walk_d2yd = robotState.walk_d2yd;
    targetYaw = robotState.walk_target_yaw;
    targetYawRate = robotState.walk_target_yaw_rate;
    targetComHeight = robotState.walk_target_com_height;
    desiredVx = robotState.walk_desired_vx;
    desiredVy = robotState.walk_desired_vy;
    legStateCur = robotState.legState;
    stanceLegCur = robotState.walk_stance_leg;
    motionStateCur = robotState.motionState;
    isDoubleSupportCur = robotState.walk_is_double_support || motionStateCur == DataBus::Stand;
    leftContactCur = robotState.walk_left_contact || motionStateCur == DataBus::Stand;
    rightContactCur = robotState.walk_right_contact || motionStateCur == DataBus::Stand;

    Jfe = Eigen::MatrixXd::Zero(12, model_nv);
    Jfe.topRows(6) = J_l;
    Jfe.bottomRows(6) = J_r;
    dJfe = Eigen::MatrixXd::Zero(12, model_nv);
    dJfe.topRows(6) = dJ_l;
    dJfe.bottomRows(6) = dJ_r;

    Jfe_foot = Eigen::MatrixXd::Zero(12, model_nv);
    Jfe_foot.topRows(6) = robotState.J_l_foot;
    Jfe_foot.bottomRows(6) = robotState.J_r_foot;
    dJfe_foot = Eigen::MatrixXd::Zero(12, model_nv);
    dJfe_foot.topRows(6).col(0) = robotState.dJ_l_foot;
    dJfe_foot.bottomRows(6).col(0) = robotState.dJ_r_foot;

    if (isLeftStance())
    {
        Jc = J_l;
        dJc = dJ_l;
        Jc_foot = Jfe_foot.topRows(6);
        Jsw = J_r;
        dJsw = dJ_r;
        fe_pos_sw_W = fe_r_pos_cur_W;
        fe_rot_sw_W = fe_r_rot_cur_W;
    }
    else
    {
        Jc = J_r;
        dJc = dJ_r;
        Jc_foot = Jfe_foot.bottomRows(6);
        Jsw = J_l;
        dJsw = dJ_l;
        fe_pos_sw_W = fe_l_pos_cur_W;
        fe_rot_sw_W = fe_l_rot_cur_W;
    }

    if (!pCoMDesInitialized)
    {
        pCoMDes = pCoMCur;
        pCoMDesInitialized = true;
    }
    updateActiveMotorIds();
}

bool WBC_BruceWeighted::isLeftStance() const
{
    return stanceLegCur == DataBus::LSt;
}

bool WBC_BruceWeighted::isDoubleSupport() const
{
    return isDoubleSupportCur;
}

void WBC_BruceWeighted::updateActiveMotorIds()
{
    activeMotorIds.clear();
    const int legMotorCount = std::min(12, model_nv - 6);

    auto addLegMotors = [&](int start, bool ankleActive)
    {
        for (int i = 0; i < 4 && start + i < legMotorCount; ++i)
        {
            activeMotorIds.push_back(start + i);
        }
        if (ankleActive)
        {
            for (int i = 4; i < 6 && start + i < legMotorCount; ++i)
            {
                activeMotorIds.push_back(start + i);
            }
        }
    };

    if (!isDoubleSupport() && motionStateCur == DataBus::Walk)
    {
        const bool leftStance = isLeftStance();
        addLegMotors(0, !leftStance);
        addLegMotors(6, leftStance);
        return;
    }

    for (int i = 0; i < legMotorCount; ++i)
    {
        activeMotorIds.push_back(i);
    }
}

bool WBC_BruceWeighted::isMotorActive(int motorId) const
{
    return std::find(activeMotorIds.begin(), activeMotorIds.end(), motorId) != activeMotorIds.end();
}

void WBC_BruceWeighted::updateComReference()
{
    const Eigen::Vector3d stancePos =
        isDoubleSupport() ? 0.5 * (fe_l_pos_cur_W + fe_r_pos_cur_W) :
                            (isLeftStance() ? fe_l_pos_cur_W : fe_r_pos_cur_W);
    if (!comReferenceInitialized)
    {
        pCoMRefWorld = pCoMDesInitialized ? pCoMDes : pCoMCur;
        vCoMRefWorld.setZero();
        comReferenceInitialized = true;
    }

    if (motionStateCur == DataBus::Stand)
    {
        pCoMRefWorld = pCoMDesInitialized ? pCoMDes : pCoMCur;
        vCoMRefWorld.setZero();
        aCoMRefWorld.setZero();
        pCoMDes = pCoMRefWorld;
        return;
    }

    if (centroidalNmpcEnabled)
    {
        pCoMRefWorld = centroidalComPosDes;
        vCoMRefWorld = centroidalComVelDes;
        aCoMRefWorld = centroidalComAccDes;
        pCoMDes = pCoMRefWorld;
        return;
    }

    const Eigen::Matrix3d yawToWorld = Rz3(targetYaw);
    vCoMRefWorld = yawToWorld * Eigen::Vector3d(desiredVx, desiredVy, 0.0);
    aCoMRefWorld.setZero();
    pCoMRefWorld.head<2>() += timeStep * vCoMRefWorld.head<2>();
    pCoMRefWorld.z() = stancePos.z() + targetComHeight;

    Eigen::Vector2d xyError = pCoMRefWorld.head<2>() - pCoMCur.head<2>();
    if (xyError.norm() > 0.20)
    {
        xyError = xyError.normalized() * 0.20;
        pCoMRefWorld.head<2>() = pCoMCur.head<2>() + xyError;
    }
    pCoMDes = pCoMRefWorld;
}

Eigen::VectorXd WBC_BruceWeighted::solveDampedLeastSquares(const Eigen::MatrixXd &A,
                                                           const Eigen::VectorXd &b,
                                                           double damping) const
{
    Eigen::MatrixXd lhs = A * A.transpose();
    lhs.diagonal().array() += damping;
    return A.transpose() * lhs.ldlt().solve(b);
}

void WBC_BruceWeighted::buildBruceIkTasks(Eigen::MatrixXd &J,
                                          Eigen::VectorXd &err,
                                          Eigen::VectorXd &dx,
                                          Eigen::VectorXd &weight) const
{
    const bool ds = isDoubleSupport();
    const int rows = ds ? 4 : 9;
    J = Eigen::MatrixXd::Zero(rows, model_nv);
    err = Eigen::VectorXd::Zero(rows);
    dx = Eigen::VectorXd::Zero(rows);
    weight = Eigen::VectorXd::Ones(rows);

    const Eigen::MatrixXd stanceJ =
        isDoubleSupport() ? 0.5 * (J_l + J_r) : (isLeftStance() ? J_l : J_r);
    const Eigen::MatrixXd stanceDJ = isLeftStance() ? dJ_l : dJ_r;
    const Eigen::Vector3d stancePos =
        isDoubleSupport() ? 0.5 * (fe_l_pos_cur_W + fe_r_pos_cur_W) :
                            (isLeftStance() ? fe_l_pos_cur_W : fe_r_pos_cur_W);
    (void)stanceDJ;

    int row = 0;
    const Eigen::MatrixXd comToStanceJ = Jcom - stanceJ.topRows(3);
    const Eigen::Vector3d comToStance = pCoMCur - stancePos;

    J.row(row) = comToStanceJ.row(2);
    const double targetComHeight =
        (motionStateCur == DataBus::Walk && walk_yd.size() >= 1) ?
            walk_yd(0) :
            (pCoMDes.z() - stancePos.z());
    err(row) = targetComHeight - comToStance.z();
    weight(row) = 100.0;
    ++row;

    J.row(row) = J_base.row(5);
    const double yawRef = (motionStateCur == DataBus::Walk) ? targetYaw : base_rpy_des(2);
    err(row) = wrapToPi(yawRef - base_rpy_cur(2));
    weight(row) = 10.0;
    ++row;

    const g1kd::Kin1D basePitch = g1kd::baseDeltaPitchKinematics(J_base, dJ_base, base_rot, dq);
    const g1kd::Kin1D baseRoll = g1kd::baseDeltaRollKinematics(J_base, dJ_base, base_rot, dq);

    J.row(row) = basePitch.jacobian;
    err(row) = vectorValueOrDefault(walk_yd, 2) - basePitch.position;
    dx(row) = vectorValueOrDefault(walk_dyd, 2);
    weight(row) = 10.0;
    ++row;

    J.row(row) = baseRoll.jacobian;
    err(row) = vectorValueOrDefault(walk_yd, 3) - baseRoll.position;
    dx(row) = vectorValueOrDefault(walk_dyd, 3);
    weight(row) = 10.0;
    ++row;

    if (!ds)
    {
        const Eigen::Vector3d swingToStance = fe_pos_sw_W - stance_fe_pos_cur_W;
        Eigen::Vector3d swingToStanceDes = swing_fe_pos_des_W - stanceDesPos_W;
        if (motionStateCur == DataBus::Walk && walk_yd.size() >= 7)
        {
            swingToStanceDes = walk_yd.segment<3>(4);
        }
        J.middleRows(row, 3) = Jsw.topRows(3) - stanceJ.topRows(3);
        err.segment(row, 3) = swingToStanceDes - swingToStance;
        weight.segment(row, 3) << 20.0, 20.0, 30.0;
        row += 3;

        const g1kd::Kin1D swingPitch = g1kd::footDeltaPitchKinematics(Jsw, dJsw, fe_rot_sw_W, dq);
        const g1kd::Kin1D swingRoll = g1kd::footDeltaRollKinematics(Jsw, dJsw, fe_rot_sw_W, dq);
        J.row(row) = swingPitch.jacobian;
        err(row) = -swingPitch.position;
        weight(row) = 5.0;
        ++row;

        J.row(row) = swingRoll.jacobian;
        err(row) = -swingRoll.position;
        weight(row) = 5.0;
        ++row;
    }

    (void)comToStance;
}

void WBC_BruceWeighted::computeDdq(Pin_KinDyn &pinKinDynIn)
{
    (void)pinKinDynIn;
    Eigen::MatrixXd Jh;
    Eigen::VectorXd dJhdq;
    Eigen::VectorXd forceRef;
    buildContact(Jh, dJhdq, forceRef);
    updateComReference();

    Eigen::MatrixXd Jtask;
    Eigen::VectorXd err, dx, weight;
    buildBruceIkTasks(Jtask, err, dx, weight);
    Eigen::MatrixXd JhJhT = Jh * Jh.transpose();
    JhJhT.diagonal().array() += 1e-6;
    const Eigen::MatrixXd JhPinv = Jh.transpose() * JhJhT.ldlt().solve(Eigen::MatrixXd::Identity(Jh.rows(), Jh.rows()));
    const Eigen::MatrixXd Nhol = Eigen::MatrixXd::Identity(model_nv, model_nv) - JhPinv * Jh;

    Eigen::MatrixXd Jproj = Jtask * Nhol;
    for (int i = 0; i < Jproj.rows(); ++i)
    {
        Jproj.row(i) *= weight(i);
        err(i) *= weight(i);
        dx(i) *= weight(i);
    }

    delta_q_final_kin = Nhol * solveDampedLeastSquares(Jproj, err, 1e-4);
    dq_final_kin = Nhol * solveDampedLeastSquares(Jproj, dx, 1e-4);
    for (int i = 0; i < model_nv - 6; ++i)
    {
        if (!isMotorActive(i))
        {
            delta_q_final_kin(6 + i) = 0.0;
            dq_final_kin(6 + i) = 0.0;
        }
    }

    std::vector<AccTask> tasks;
    buildBruceAccelerationTasks(tasks);
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(model_nv, model_nv) * 1e-6;
    Eigen::VectorXd g = Eigen::VectorXd::Zero(model_nv);
    for (const auto &task : tasks)
    {
        if (task.J.rows() == 0)
        {
            continue;
        }
        const Eigen::VectorXd residualBias = task.dJdq - task.desiredAcc;
        const Eigen::MatrixXd W = task.weight.asDiagonal();
        H.noalias() += task.J.transpose() * W * task.J;
        g.noalias() += task.J.transpose() * W * residualBias;
    }
    ddq_final_kin = -H.ldlt().solve(g);
}

void WBC_BruceWeighted::buildBruceAccelerationTasks(std::vector<AccTask> &tasks) const
{
    tasks.clear();

    auto addTask = [&](const Eigen::MatrixXd &J,
                       const Eigen::VectorXd &dJdq,
                       const Eigen::VectorXd &desiredAcc,
                       const Eigen::VectorXd &weight)
    {
        AccTask task;
        task.J = J;
        task.dJdq = dJdq;
        task.desiredAcc = desiredAcc;
        task.weight = weight;
        tasks.push_back(task);
    };

    if (dyn_Ag.rows() >= 6 && dyn_dAg.rows() >= 6)
    {
        const Eigen::Vector3d angularMomentum = (dyn_Ag * dq).tail<3>();
        const Eigen::MatrixXd Jang = dyn_Ag.bottomRows(3);
        const Eigen::VectorXd dJdq = dyn_dAg.bottomRows(3) * dq;
        const Eigen::Vector3d desired = -Eigen::Vector3d(10.0, 10.0, 1.0).cwiseProduct(angularMomentum);
        addTask(Jang, dJdq, desired, (Eigen::Vector3d() << 10.0, 10.0, 1.0).finished());
    }

    const Eigen::Vector3d comVel = Jcom * dq;
    Eigen::Vector3d comErr = pCoMRefWorld - pCoMCur;
    comErr.x() = std::clamp(comErr.x(), -0.20, 0.20);
    comErr.y() = std::clamp(comErr.y(), -0.20, 0.20);
    const Eigen::Vector3d comKp(50.0, 50.0, 450.0);
    const Eigen::Vector3d comKd(15.0, 5.0, 35.0);
    const Eigen::Vector3d desiredComAcc =
        aCoMRefWorld + comKp.cwiseProduct(comErr) + comKd.cwiseProduct(vCoMRefWorld - comVel);
    addTask(Jcom, Eigen::VectorXd::Zero(3), desiredComAcc,
            (Eigen::Vector3d() << 10.0, 10.0, 100.0).finished());

    Eigen::MatrixXd Jbase = Eigen::MatrixXd::Zero(3, model_nv);
    Eigen::Vector3d dJdqBase = Eigen::Vector3d::Zero();
    Jbase.row(0) = J_base.row(5);
    Jbase.row(1) = J_base.row(4);
    Jbase.row(2) = J_base.row(3);
    dJdqBase(0) = (dJ_base.row(5) * dq)(0);
    dJdqBase(1) = (dJ_base.row(4) * dq)(0);
    dJdqBase(2) = (dJ_base.row(3) * dq)(0);
    Eigen::Vector3d baseErr;
    const double yawRef = (motionStateCur == DataBus::Walk) ? targetYaw : base_rpy_des(2);
    const double yawRateRef = (motionStateCur == DataBus::Walk) ? targetYawRate : 0.0;
    baseErr << wrapToPi(yawRef - base_rpy_cur(2)),
        base_rpy_des(1) - base_rpy_cur(1),
        base_rpy_des(0) - base_rpy_cur(0);
    Eigen::Vector3d baseVelErr;
    baseVelErr << yawRateRef - dq(5), -dq(4), -dq(3);
    const Eigen::Vector3d desiredBaseAcc =
        Eigen::Vector3d(150.0, 300.0, 300.0).cwiseProduct(baseErr) +
        Eigen::Vector3d(15.0, 25.0, 25.0).cwiseProduct(baseVelErr);
    addTask(Jbase, dJdqBase, desiredBaseAcc,
            (Eigen::Vector3d() << 10.0, 10.0, 10.0).finished());

    if (!isDoubleSupport())
    {
        const g1kd::Kin1D stanceYaw = g1kd::frameYawKinematics(isLeftStance() ? fe_l_rot_cur_W : fe_r_rot_cur_W,
                                                               Jc,
                                                               dJc,
                                                               dq);
        Eigen::VectorXd desiredYaw = Eigen::VectorXd::Zero(1);
        desiredYaw(0) = 180.0 * wrapToPi(yawRef - stanceYaw.position) +
                        18.0 * (yawRateRef - stanceYaw.velocity);
        Eigen::VectorXd dJdqYaw = Eigen::VectorXd::Constant(1, stanceYaw.dJdq);
        Eigen::MatrixXd JstanceYaw = Eigen::MatrixXd::Zero(1, model_nv);
        JstanceYaw.row(0) = stanceYaw.jacobian;
        addTask(JstanceYaw, dJdqYaw, desiredYaw, Eigen::VectorXd::Constant(1, 1000.0));

        Eigen::MatrixXd JstanceOri = Eigen::MatrixXd::Zero(2, model_nv);
        const g1kd::Kin1D stancePitch = g1kd::footDeltaPitchKinematics(Jc, dJc, isLeftStance() ? fe_l_rot_cur_W : fe_r_rot_cur_W, dq);
        const g1kd::Kin1D stanceRoll = g1kd::footDeltaRollKinematics(Jc, dJc, isLeftStance() ? fe_l_rot_cur_W : fe_r_rot_cur_W, dq);
        JstanceOri.row(0) = stancePitch.jacobian;
        JstanceOri.row(1) = stanceRoll.jacobian;
        Eigen::VectorXd dJdqStanceOri = Eigen::VectorXd::Zero(2);
        dJdqStanceOri << stancePitch.dJdq, stanceRoll.dJdq;
        addTask(JstanceOri, dJdqStanceOri, Eigen::VectorXd::Zero(2),
                (Eigen::Vector2d() << 1000.0, 1000.0).finished());

        const Eigen::MatrixXd stanceJ = isLeftStance() ? J_l : J_r;
        const Eigen::Vector3d swingToStance = fe_pos_sw_W - stance_fe_pos_cur_W;
        Eigen::Vector3d swingToStanceDes = swing_fe_pos_des_W - stanceDesPos_W;
        Eigen::Vector3d swingVelDes = swing_fe_vel_des_W;
        Eigen::Vector3d swingAccDes = swing_fe_acc_des_W;
        if (motionStateCur == DataBus::Walk && walk_yd.size() >= 7)
        {
            swingToStanceDes = walk_yd.segment<3>(4);
        }
        if (motionStateCur == DataBus::Walk && walk_dyd.size() >= 7)
        {
            swingVelDes = walk_dyd.segment<3>(4);
        }
        if (motionStateCur == DataBus::Walk && walk_d2yd.size() >= 7)
        {
            swingAccDes = walk_d2yd.segment<3>(4);
        }
        const Eigen::MatrixXd JswingRel = Jsw.topRows(3) - stanceJ.topRows(3);
        const Eigen::VectorXd dJswingRel = (dJsw.topRows(3) - (isLeftStance() ? dJ_l.topRows(3) : dJ_r.topRows(3))) * dq;
        const Eigen::Vector3d desiredSwingAcc =
            swingAccDes +
            Eigen::Vector3d(450.0, 450.0, 450.0).cwiseProduct(swingToStanceDes - swingToStance) -
            Eigen::Vector3d(30.0, 30.0, 30.0).cwiseProduct(JswingRel * dq - swingVelDes);
        addTask(JswingRel, dJswingRel, desiredSwingAcc,
                (Eigen::Vector3d() << 20.0, 20.0, 30.0).finished());

        const g1kd::Kin1D swingYaw = g1kd::frameYawKinematics(fe_rot_sw_W, Jsw, dJsw, dq);
        Eigen::MatrixXd JswingYaw = Eigen::MatrixXd::Zero(1, model_nv);
        JswingYaw.row(0) = swingYaw.jacobian;
        Eigen::VectorXd dJdqSwingYaw = Eigen::VectorXd::Constant(1, swingYaw.dJdq);
        Eigen::VectorXd desiredSwingYaw = Eigen::VectorXd::Zero(1);
        desiredSwingYaw(0) = 180.0 * wrapToPi(yawRef - swingYaw.position) +
                              18.0 * (yawRateRef - swingYaw.velocity);
        addTask(JswingYaw, dJdqSwingYaw, desiredSwingYaw, Eigen::VectorXd::Constant(1, 10.0));

        const g1kd::Kin1D swingPitch = g1kd::footDeltaPitchKinematics(Jsw, dJsw, fe_rot_sw_W, dq);
        const g1kd::Kin1D swingRoll = g1kd::footDeltaRollKinematics(Jsw, dJsw, fe_rot_sw_W, dq);
        Eigen::MatrixXd JswingOri = Eigen::MatrixXd::Zero(2, model_nv);
        JswingOri.row(0) = swingPitch.jacobian;
        JswingOri.row(1) = swingRoll.jacobian;
        Eigen::VectorXd dJdqSwingOri = Eigen::VectorXd::Zero(2);
        dJdqSwingOri << swingPitch.dJdq, swingRoll.dJdq;
        Eigen::VectorXd desiredSwingOri = Eigen::VectorXd::Zero(2);
        desiredSwingOri << 220.0 * (0.0 - swingPitch.position) - 20.0 * swingPitch.velocity,
            220.0 * (0.0 - swingRoll.position) - 20.0 * swingRoll.velocity;
        addTask(JswingOri, dJdqSwingOri, desiredSwingOri,
                (Eigen::Vector2d() << 5.0, 5.0).finished());
    }
}

void WBC_BruceWeighted::buildContact(Eigen::MatrixXd &Jh,
                                     Eigen::VectorXd &dJhdq,
                                     Eigen::VectorXd &forceRef) const
{
    const bool useLeft = leftContactCur;
    const bool useRight = rightContactCur;
    const int contactCount = (useLeft ? 1 : 0) + (useRight ? 1 : 0);

    if (contactCount == 2)
    {
        Jh = Jfe_foot;
        dJhdq = Eigen::VectorXd::Zero(12);
        dJhdq.head(6) = dJfe_foot.topRows(6).col(0);
        dJhdq.tail(6) = dJfe_foot.bottomRows(6).col(0);
        forceRef = Eigen::VectorXd::Zero(12);
        forceRef.segment<6>(0) = planeForceReferenceFromWrench(Fr_ff.segment<6>(0));
        forceRef.segment<6>(6) = planeForceReferenceFromWrench(Fr_ff.segment<6>(6));
    }
    else if (useLeft)
    {
        Jh = Jfe_foot.topRows(6);
        dJhdq = dJfe_foot.topRows(6).col(0);
        forceRef = planeForceReferenceFromWrench(Fr_ff.segment<6>(0));
    }
    else if (useRight)
    {
        Jh = Jfe_foot.bottomRows(6);
        dJhdq = dJfe_foot.bottomRows(6).col(0);
        forceRef = planeForceReferenceFromWrench(Fr_ff.segment<6>(6));
    }
    else
    {
        Jh.resize(0, model_nv);
        dJhdq.resize(0);
        forceRef.resize(0);
    }
}

void WBC_BruceWeighted::computeTau()
{
    Eigen::MatrixXd Jh;
    Eigen::VectorXd dJhdq;
    Eigen::VectorXd forceRef;
    buildContact(Jh, dJhdq, forceRef);

    const int nuFull = model_nv - 6;
    const int nu = static_cast<int>(activeMotorIds.size());
    const int nh = Jh.rows();
    const int nVar = model_nv + nu + nh;
    const int nEq = model_nv + nh;
    const int nFric = 11 * (nh / 6);
    const int nIneq = nFric + 2 * nu;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nVar, nVar);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(nVar);

    std::vector<AccTask> tasks;
    buildBruceAccelerationTasks(tasks);
    for (const auto &task : tasks)
    {
        if (task.J.rows() == 0)
        {
            continue;
        }
        const Eigen::VectorXd residualBias = task.dJdq - task.desiredAcc;
        const Eigen::MatrixXd W = task.weight.asDiagonal();
        H.block(0, 0, model_nv, model_nv).noalias() += task.J.transpose() * W * task.J;
        g.head(model_nv).noalias() += task.J.transpose() * W * residualBias;
    }

    H.diagonal().head(model_nv).array() += 1e-6;
    H.diagonal().segment(model_nv, nu).array() += 1e-6;
    H.diagonal().tail(nh).array() += 1e-6;
    if (nh > 0 && forceRef.size() == nh)
    {
        const double forceTrackWeight = centroidalNmpcEnabled ? 1e-3 : 1e-5;
        H.block(model_nv + nu, model_nv + nu, nh, nh).diagonal().array() += forceTrackWeight;
        g.segment(model_nv + nu, nh).noalias() -= forceTrackWeight * forceRef;
    }

    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(nEq, nVar);
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(nEq);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(model_nv, nu);
    for (int i = 0; i < nu; ++i)
    {
        B(6 + activeMotorIds[i], i) = 1.0;
    }
    Aeq.block(0, 0, model_nv, model_nv) = dyn_M;
    Aeq.block(0, model_nv, model_nv, nu) = -B;
    Aeq.block(0, model_nv + nu, model_nv, nh) = -Jh.transpose();
    beq.head(model_nv) = -dyn_Non;
    Aeq.block(model_nv, 0, nh, model_nv) = Jh;
    beq.tail(nh) = -dJhdq;

    Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(nIneq, nVar);
    Eigen::VectorXd ubIneq = Eigen::VectorXd::Constant(nIneq, 1e10);
    Eigen::VectorXd lbIneq = Eigen::VectorXd::Constant(nIneq, -1e10);
    const Eigen::Matrix<double, 11, 6> footCone =
        g1kd::roMoCoPlaneFootCone(miu, foot_half_width, foot_l_front, foot_l_back, foot_yaw_friction);
    for (int i = 0; i < nh / 6; ++i)
    {
        const int row = 11 * i;
        Aineq.block(row, model_nv + nu + 6 * i, 11, 6) = footCone;
        ubIneq.segment(row, 11).setZero();
        ubIneq(row) = -f_z_low;
    }

    const int torqueRow = nFric;
    for (int i = 0; i < nu; ++i)
    {
        Aineq(torqueRow + i, model_nv + i) = 1.0;
        Aineq(torqueRow + nu + i, model_nv + i) = -1.0;
        const double tauLimit = g1kd::motorTorqueLimit(activeMotorIds[i]);
        ubIneq(torqueRow + i) = tauLimit;
        ubIneq(torqueRow + nu + i) = tauLimit;
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nEq + nIneq, nVar);
    Eigen::VectorXd lbA = Eigen::VectorXd::Constant(nEq + nIneq, -1e10);
    Eigen::VectorXd ubA = Eigen::VectorXd::Constant(nEq + nIneq, 1e10);
    A.topRows(nEq) = Aeq;
    lbA.head(nEq) = beq;
    ubA.head(nEq) = beq;
    A.bottomRows(nIneq) = Aineq;
    lbA.tail(nIneq) = lbIneq;
    ubA.tail(nIneq) = ubIneq;

    std::vector<qpOASES::real_t> qpH(nVar * nVar), qpg(nVar), qpA((nEq + nIneq) * nVar);
    std::vector<qpOASES::real_t> qplbA(nEq + nIneq), qpubA(nEq + nIneq);
    copyEigenToReal(qpH.data(), H);
    copyEigenToReal(qpg.data(), g);
    copyEigenToReal(qpA.data(), A);
    copyEigenToReal(qplbA.data(), lbA);
    copyEigenToReal(qpubA.data(), ubA);

    qpOASES::QProblem prob(nVar, nEq + nIneq);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    prob.setOptions(options);
    nWSR = 300;
    cpu_time = timeStep;
    qpOASES::returnValue res = prob.init(qpH.data(), qpg.data(), qpA.data(), nullptr, nullptr,
                                         qplbA.data(), qpubA.data(), nWSR, &cpu_time);
    qpStatus = qpOASES::getSimpleStatus(res);

    std::vector<qpOASES::real_t> sol(nVar, 0.0);
    prob.getPrimalSolution(sol.data());
    if (res == qpOASES::SUCCESSFUL_RETURN)
    {
        eigen_ddq_Opt = Eigen::VectorXd::Zero(model_nv);
        tauJointRes = Eigen::VectorXd::Zero(nuFull);
        eigen_fr_Opt = Eigen::VectorXd::Zero(12);
        for (int i = 0; i < model_nv; ++i)
        {
            eigen_ddq_Opt(i) = sol[i];
        }
        for (int i = 0; i < nu; ++i)
        {
            tauJointRes(activeMotorIds[i]) = sol[model_nv + i];
        }
        if (leftContactCur && rightContactCur)
        {
            for (int i = 0; i < 12; ++i)
            {
                eigen_fr_Opt(i) = sol[model_nv + nu + i];
            }
        }
        else if (leftContactCur)
        {
            for (int i = 0; i < 6; ++i)
            {
                eigen_fr_Opt(i) = sol[model_nv + nu + i];
            }
        }
        else if (rightContactCur)
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
        eigen_fr_Opt = Eigen::VectorXd::Zero(12);
        if (leftContactCur && rightContactCur)
        {
            eigen_fr_Opt = forceRef;
        }
        else if (leftContactCur)
        {
            eigen_fr_Opt.segment<6>(0) = forceRef;
        }
        else if (rightContactCur)
        {
            eigen_fr_Opt.segment<6>(6) = forceRef;
        }
        Eigen::VectorXd tauRes = dyn_M * eigen_ddq_Opt + dyn_Non - Jfe_foot.transpose() * eigen_fr_Opt;
        tauJointRes = Eigen::VectorXd::Zero(nuFull);
        for (int i = 0; i < nu; ++i)
        {
            tauJointRes(activeMotorIds[i]) = tauRes(6 + activeMotorIds[i]);
        }
    }

    const Eigen::Matrix<double, 6, 6> H_contact = g1kd::planeForceToWrenchMap();
    Eigen::VectorXd wrenchForLog = Eigen::VectorXd::Zero(12);
    wrenchForLog.segment<6>(0) = H_contact * eigen_fr_Opt.segment<6>(0);
    wrenchForLog.segment<6>(6) = H_contact * eigen_fr_Opt.segment<6>(6);
    eigen_fr_Opt = wrenchForLog;
}

void WBC_BruceWeighted::dataBusWrite(DataBus &robotState)
{
    robotState.wbc_delta_q_final = delta_q_final_kin;
    robotState.wbc_dq_final = dq_final_kin;
    robotState.wbc_ddq_final = ddq_final_kin;
    robotState.wbc_tauJointRes = tauJointRes;
    robotState.wbc_FrRes = eigen_fr_Opt;
    robotState.wbc_active_motor_ids = activeMotorIds;
    robotState.qp_status = qpStatus;
    robotState.qp_nWSR = nWSR;
    robotState.qp_cpuTime = cpu_time;
}

void WBC_BruceWeighted::copyEigenToReal(qpOASES::real_t *target, const Eigen::MatrixXd &source) const
{
    int count = 0;
    for (int i = 0; i < source.rows(); ++i)
    {
        for (int j = 0; j < source.cols(); ++j)
        {
            target[count++] = std::isinf(source(i, j)) ? qpOASES::INFTY : source(i, j);
        }
    }
}

void WBC_BruceWeighted::copyEigenToReal(qpOASES::real_t *target, const Eigen::VectorXd &source) const
{
    for (int i = 0; i < source.rows(); ++i)
    {
        target[i] = std::isinf(source(i)) ? qpOASES::INFTY : source(i);
    }
}

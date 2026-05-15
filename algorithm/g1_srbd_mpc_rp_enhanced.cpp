#include "g1_srbd_mpc_rp_enhanced.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "pino_kin_dyn.h"
#include "useful_math.h"

namespace
{
constexpr double kGravity = 9.80665;
constexpr double kInf = 1e10;

Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d S;
    S << 0.0, -v.z(), v.y(),
        v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return S;
}

Eigen::Matrix3d quatToMat(const Eigen::Quaterniond &qIn)
{
    Eigen::Quaterniond q = qIn.normalized();
    if (!q.coeffs().allFinite() || q.norm() < 1e-8)
    {
        return Eigen::Matrix3d::Identity();
    }
    return q.toRotationMatrix();
}

Eigen::Quaterniond quatFromQ(const Eigen::VectorXd &q)
{
    if (q.size() < 7)
    {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(q(6), q(3), q(4), q(5));
}

Eigen::Vector3d vectorFromArray(const double value[3])
{
    return Eigen::Vector3d(value[0], value[1], value[2]);
}

double vectorValueOrZero(const std::vector<double> &values, int index)
{
    return (index >= 0 && index < static_cast<int>(values.size())) ? values[index] : 0.0;
}

Eigen::Vector3d comVelocityWorld(const DataBus &robotState)
{
    if (robotState.Jcom_W.rows() == 3 && robotState.Jcom_W.cols() == robotState.dq.size())
    {
        const Eigen::Vector3d vCom = robotState.Jcom_W * robotState.dq;
        if (vCom.allFinite())
        {
            return vCom;
        }
    }
    if (robotState.dq.size() >= 3)
    {
        return robotState.dq.head<3>();
    }
    return Eigen::Vector3d::Zero();
}
} // namespace

G1SrbdMpcRpEnhanced::G1SrbdMpcRpEnhanced(int horizon, double dt)
{
    horizon_ = std::max(1, horizon);
    dt_ = std::max(1e-4, dt);
}

bool G1SrbdMpcRpEnhanced::solveFromDataBus(const DataBus &robotState, double mass, const Eigen::Matrix3d &inertiaBody)
{
    return solve(buildInputFromDataBus(robotState, mass, inertiaBody));
}

G1SrbdMpcRpEnhanced::Input G1SrbdMpcRpEnhanced::buildInputFromDataBus(const DataBus &robotState,
                                                   double mass,
                                                   const Eigen::Matrix3d &inertiaBody) const
{
    Input input;
    input.mass = std::max(1.0, mass);
    input.inertiaBody = inertiaBody.allFinite() ? inertiaBody : Eigen::Matrix3d::Identity();
    input.orientation = quatFromQ(robotState.q);
    const Eigen::Vector3d vCom = comVelocityWorld(robotState);
    const double waistRoll = vectorValueOrZero(robotState.motors_pos_cur, waistRollJointId);
    const double waistPitch = vectorValueOrZero(robotState.motors_pos_cur, waistPitchJointId);
    const double waistRollRate = vectorValueOrZero(robotState.motors_vel_cur, waistRollJointId);
    const double waistPitchRate = vectorValueOrZero(robotState.motors_vel_cur, waistPitchJointId);
    input.current.setZero();
    input.current << robotState.base_rpy.x(), robotState.base_rpy.y(), robotState.base_rpy.z(),
        robotState.pCoM_W.x(), robotState.pCoM_W.y(), robotState.pCoM_W.z(),
        robotState.base_omega_W.x(), robotState.base_omega_W.y(), robotState.base_omega_W.z(),
        vCom.x(), vCom.y(), vCom.z(), kGravity,
        waistRoll, waistPitch,
        waistRollRate, waistPitchRate;
    input.contactPositionWorld = {robotState.fe_l_pos_W, robotState.fe_r_pos_W};
    input.reference.resize(kNx, horizon_);
    input.contactTable.resize(horizon_, 2);

    Eigen::Vector3d desiredVel = Eigen::Vector3d::Zero();
    if (robotState.motionState == DataBus::Walk)
    {
        desiredVel = robotState.desV_W;
    }
    else if (robotState.centroidal_nmpc_enabled)
    {
        desiredVel = robotState.centroidal_nmpc_com_vel_des;
    }
    Eigen::Vector3d refCom = robotState.pCoM_W;
    if (robotState.motionState == DataBus::Walk)
    {
        const double supportZ =
            robotState.walk_is_double_support ? 0.5 * (robotState.fe_l_pos_W.z() + robotState.fe_r_pos_W.z()) :
                                                robotState.stance_fe_pos_cur_W.z();
        refCom.z() = supportZ + robotState.walk_target_com_height;
    }
    else if (robotState.centroidal_nmpc_enabled)
    {
        refCom = robotState.centroidal_nmpc_com_pos_des;
    }

    for (int k = 0; k < horizon_; ++k)
    {
        input.contactTable(k, 0) = (robotState.motionState == DataBus::Stand || robotState.walk_left_contact) ? 1 : 0;
        input.contactTable(k, 1) = (robotState.motionState == DataBus::Stand || robotState.walk_right_contact) ? 1 : 0;
        const Eigen::Vector3d pRef = refCom + static_cast<double>(k + 1) * dt_ * desiredVel;
        input.reference.col(k) << 0.0, 0.0, robotState.walk_target_yaw,
            pRef.x(), pRef.y(), pRef.z(),
            0.0, 0.0, robotState.walk_target_yaw_rate,
            desiredVel.x(), desiredVel.y(), desiredVel.z(),
            kGravity,
            torsoRollRef, torsoPitchRef,
            0.0, 0.0;
    }
    return input;
}

void G1SrbdMpcRpEnhanced::buildDiscreteModel(const Input &input,
                                   int knot,
                                   Eigen::Matrix<double, kNx, kNx> &A,
                                   Eigen::Matrix<double, kNx, kNu> &B) const
{
    A.setIdentity();
    B.setZero();

    const Eigen::Matrix3d R = quatToMat(input.orientation);
    Eigen::Matrix3d IangBody = Eigen::Matrix3d::Zero();
    IangBody(0, 0) = std::max(1e-6, lowerBodyInertiaRoll);
    IangBody(1, 1) = std::max(1e-6, lowerBodyInertiaPitch);
    IangBody(2, 2) = std::max(1e-6, wholeBodyInertiaYaw);
    const Eigen::Matrix3d IangInvWorld = R * IangBody.inverse() * R.transpose();
    const Eigen::Matrix<double, 6, 6> HContact = g1_kin_dyn::planeForceToWrenchMap();

    Eigen::Matrix<double, kNx, kNx> Ac = Eigen::Matrix<double, kNx, kNx>::Zero();
    Ac.block<3, 3>(kIdxRpy, kIdxOmega) = Rz3(input.current(2));
    Ac.block<3, 3>(kIdxPos, kIdxVel).setIdentity();
    Ac(kIdxVel + 2, kIdxGravity) = -1.0;
    Ac.block<2, 2>(kIdxTorsoRp, kIdxTorsoRpRate).setIdentity();

    Eigen::Matrix<double, kNx, kNu> Bc = Eigen::Matrix<double, kNx, kNu>::Zero();
    Eigen::Vector3d com = input.current.segment<3>(kIdxPos);
    if (input.reference.cols() > knot)
    {
        com = input.reference.col(knot).segment<3>(kIdxPos);
    }
    const std::array<Eigen::Vector3d, 2> *contactPositions = &input.contactPositionWorld;
    if (static_cast<int>(input.contactPositionWorldHorizon.size()) > knot)
    {
        contactPositions = &input.contactPositionWorldHorizon[knot];
    }
    for (int leg = 0; leg < 2; ++leg)
    {
        const Eigen::Vector3d r = (*contactPositions)[leg] - com;
        Eigen::Matrix<double, 6, 6> G = Eigen::Matrix<double, 6, 6>::Zero();
        G.block<3, 3>(0, 0).setIdentity();
        G.block<3, 3>(3, 0) = skew(r);
        G.block<3, 3>(3, 3).setIdentity();

        Eigen::Matrix<double, 6, 6> planeToCentroidal = G * HContact;
        Bc.block(kIdxVel, 6 * leg, 3, 6) = planeToCentroidal.topRows<3>() / input.mass;
        Bc.block(kIdxOmega, 6 * leg, 3, 6) = IangInvWorld * planeToCentroidal.bottomRows<3>();
    }

    Eigen::Matrix<double, 3, 2> SrpBody;
    SrpBody << 1.0, 0.0,
        0.0, 1.0,
        0.0, 0.0;
    const Eigen::Matrix<double, 3, 2> SrpWorld = R * SrpBody;
    Bc.block<3, 2>(kIdxOmega, kIdxTorsoInput) = -IangInvWorld * SrpWorld;

    Eigen::Matrix2d ItrInv = Eigen::Matrix2d::Zero();
    ItrInv(0, 0) = 1.0 / std::max(1e-6, torsoInertiaRoll);
    ItrInv(1, 1) = 1.0 / std::max(1e-6, torsoInertiaPitch);
    Bc.block<2, 2>(kIdxTorsoRpRate, kIdxTorsoInput) = ItrInv;

    A += dt_ * Ac;
    B = dt_ * Bc;
}

bool G1SrbdMpcRpEnhanced::solve(const Input &input)
{
    firstPlaneForces_.setZero();
    firstWrenches_.setZero();
    firstTorsoTorqueRp_.setZero();
    firstTorsoRp_.setZero();
    firstTorsoRpRate_.setZero();
    qpStatus_ = -1;

    if (input.reference.cols() != horizon_ || input.contactTable.rows() != horizon_)
    {
        return false;
    }

    const int nx = kNx;
    const int nu = kNu;
    const int nVar = nu * horizon_;
    const int nState = nx * horizon_;
    const int coneRowsPerFoot = 11;
    const int forceBoundRowsPerFoot = 6;
    const int rowsPerFoot = coneRowsPerFoot + forceBoundRowsPerFoot;
    const int contactRowsPerKnot = 2 * rowsPerFoot;
    const int torsoTorqueRowsPerKnot = 4;
    const int torsoStateRowsPerKnot = 4;
    const int rowsPerKnot = contactRowsPerKnot + torsoTorqueRowsPerKnot + torsoStateRowsPerKnot;
    const int nCon = rowsPerKnot * horizon_;

    Eigen::MatrixXd Aqp = Eigen::MatrixXd::Zero(nState, nx);
    Eigen::MatrixXd Bqp = Eigen::MatrixXd::Zero(nState, nVar);
    Eigen::Matrix<double, kNx, kNx> Apow = Eigen::Matrix<double, kNx, kNx>::Identity();
    std::vector<Eigen::Matrix<double, kNx, kNx>> AdList(horizon_);
    std::vector<Eigen::Matrix<double, kNx, kNu>> BdList(horizon_);
    for (int i = 0; i < horizon_; ++i)
    {
        buildDiscreteModel(input, i, AdList[i], BdList[i]);
        Apow = AdList[i] * Apow;
        Aqp.block(i * nx, 0, nx, nx) = Apow;
        Eigen::Matrix<double, kNx, kNx> Aij = Eigen::Matrix<double, kNx, kNx>::Identity();
        for (int j = i; j >= 0; --j)
        {
            Bqp.block(i * nx, j * nu, nx, nu) = Aij * BdList[j];
            Aij = Aij * AdList[j];
        }
    }

    Eigen::VectorXd x0 = input.current;
    Eigen::VectorXd xd = Eigen::VectorXd::Zero(nState);
    for (int k = 0; k < horizon_; ++k)
    {
        xd.segment(k * nx, nx) = input.reference.col(k);
    }

    Eigen::Matrix<double, kNx, 1> stateWeights;
    stateWeights << 80.0, 80.0, 120.0,
        120.0, 120.0, 220.0,
        1.0, 1.0, 4.0,
        40.0, 40.0, 20.0,
        0.0,
        20.0, 80.0,
        2.0, 8.0;
    Eigen::Matrix<double, kNu, 1> inputWeights;
    inputWeights << 1e-4, 1e-4, 2e-4, 1e-4, 1e-4, 2e-4,
        1e-4, 1e-4, 2e-4, 1e-4, 1e-4, 2e-4,
        1e-3, 2e-3;
    Eigen::VectorXd uNom = Eigen::VectorXd::Zero(nVar);
    for (int k = 0; k < horizon_; ++k)
    {
        int activeContacts = 0;
        for (int leg = 0; leg < 2; ++leg)
        {
            activeContacts += input.contactTable(k, leg) != 0 ? 1 : 0;
        }
        if (activeContacts == 0)
        {
            continue;
        }
        const double nominalFz = input.mass * kGravity / static_cast<double>(activeContacts);
        for (int leg = 0; leg < 2; ++leg)
        {
            if (input.contactTable(k, leg) != 0)
            {
                uNom.segment<6>(k * nu + 6 * leg) =
                    g1_kin_dyn::distributeVerticalFootLoad(nominalFz);
            }
        }
    }

    Eigen::VectorXd Wdiag = stateWeights.replicate(horizon_, 1);
    Eigen::VectorXd Rdiag = inputWeights.replicate(horizon_, 1);
    Eigen::MatrixXd H = 2.0 * (Bqp.transpose() * Wdiag.asDiagonal() * Bqp);
    H.diagonal() += 2.0 * Rdiag;
    Eigen::VectorXd g = 2.0 * Bqp.transpose() * Wdiag.asDiagonal() * (Aqp * x0 - xd);
    for (int k = 0; k < horizon_; ++k)
    {
        for (int j = 0; j < kNuContact; ++j)
        {
            const int idx = k * nu + j;
            H(idx, idx) += 2.0 * nominalForceWeight;
            g(idx) -= 2.0 * nominalForceWeight * uNom(idx);
        }
    }
    for (int j = 0; j < kNuTorso; ++j)
    {
        const int idx0 = kIdxTorsoInput + j;
        H(idx0, idx0) += 2.0 * torsoTorqueRateWeight;
        g(idx0) -= 2.0 * torsoTorqueRateWeight * lastTorsoTorqueRp_(j);
    }
    for (int k = 1; k < horizon_; ++k)
    {
        for (int j = 0; j < kNuTorso; ++j)
        {
            const int idxCurr = k * nu + kIdxTorsoInput + j;
            const int idxPrev = (k - 1) * nu + kIdxTorsoInput + j;
            H(idxCurr, idxCurr) += 2.0 * torsoTorqueRateWeight;
            H(idxPrev, idxPrev) += 2.0 * torsoTorqueRateWeight;
            H(idxCurr, idxPrev) -= 2.0 * torsoTorqueRateWeight;
            H(idxPrev, idxCurr) -= 2.0 * torsoTorqueRateWeight;
        }
    }
    H.diagonal().array() += 1e-8;

    Eigen::MatrixXd Acon = Eigen::MatrixXd::Zero(nCon, nVar);
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(nCon, -kInf);
    Eigen::VectorXd ub = Eigen::VectorXd::Zero(nCon);
    const Eigen::Matrix<double, 11, 6> cone =
        g1_kin_dyn::roMoCoPlaneFootCone(mu, footHalfWidth, footFront, footBack, yawFriction);
    auto addCondensedStateBound =
        [&](int row, int knot, int stateIdx, double lower, double upper)
    {
        Acon.row(row) = Bqp.block(knot * nx + stateIdx, 0, 1, nVar);
        const double freeState =
            (Aqp.block(knot * nx + stateIdx, 0, 1, nx) * x0)(0);
        lb(row) = lower - freeState;
        ub(row) = upper - freeState;
    };

    for (int k = 0; k < horizon_; ++k)
    {
        const int knotRow = k * rowsPerKnot;
        for (int leg = 0; leg < 2; ++leg)
        {
            const int row = knotRow + leg * rowsPerFoot;
            const int col = k * nu + leg * 6;
            Acon.block(row, col, coneRowsPerFoot, 6) = cone;
            const bool contact = input.contactTable(k, leg) != 0;
            ub.segment(row, coneRowsPerFoot).setZero();
            ub(row) = contact ? -fzLow : 0.0;
            const int forceBoundRow = row + coneRowsPerFoot;
            for (int j = 0; j < 6; ++j)
            {
                Acon(forceBoundRow + j, col + j) = 1.0;
                lb(forceBoundRow + j) = contact ? -fMax : 0.0;
                ub(forceBoundRow + j) = contact ? fMax : 0.0;
            }
        }

        const int torsoRow = knotRow + contactRowsPerKnot;
        const int tauCol = k * nu + kIdxTorsoInput;
        Acon(torsoRow + 0, tauCol + 0) = 1.0;
        ub(torsoRow + 0) = torsoTorqueMaxRoll;
        Acon(torsoRow + 1, tauCol + 0) = -1.0;
        ub(torsoRow + 1) = torsoTorqueMaxRoll;
        Acon(torsoRow + 2, tauCol + 1) = 1.0;
        ub(torsoRow + 2) = torsoTorqueMaxPitch;
        Acon(torsoRow + 3, tauCol + 1) = -1.0;
        ub(torsoRow + 3) = torsoTorqueMaxPitch;

        const int torsoStateRow = torsoRow + torsoTorqueRowsPerKnot;
        addCondensedStateBound(torsoStateRow + 0, k, kIdxTorsoRp + 0,
                               -torsoRollMax, torsoRollMax);
        addCondensedStateBound(torsoStateRow + 1, k, kIdxTorsoRp + 1,
                               -torsoPitchMax, torsoPitchMax);
        addCondensedStateBound(torsoStateRow + 2, k, kIdxTorsoRpRate + 0,
                               -torsoRollRateMax, torsoRollRateMax);
        addCondensedStateBound(torsoStateRow + 3, k, kIdxTorsoRpRate + 1,
                               -torsoPitchRateMax, torsoPitchRateMax);
    }

    std::vector<qpOASES::real_t> qpH(nVar * nVar), qpg(nVar), qpA(nCon * nVar), qplbA(nCon), qpubA(nCon);
    copyEigenToReal(qpH.data(), H);
    copyEigenToReal(qpg.data(), g);
    copyEigenToReal(qpA.data(), Acon);
    copyEigenToReal(qplbA.data(), lb);
    copyEigenToReal(qpubA.data(), ub);

    qpOASES::QProblem prob(nVar, nCon);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    prob.setOptions(options);
    qpOASES::int_t nWSR = 300;
    qpOASES::real_t cpuTime = dt_;
    const qpOASES::returnValue res =
        prob.init(qpH.data(), qpg.data(), qpA.data(), nullptr, nullptr,
                  qplbA.data(), qpubA.data(), nWSR, &cpuTime);
    qpStatus_ = qpOASES::getSimpleStatus(res);
    if (res != qpOASES::SUCCESSFUL_RETURN)
    {
        return false;
    }

    std::vector<qpOASES::real_t> sol(nVar, 0.0);
    prob.getPrimalSolution(sol.data());
    for (int i = 0; i < kNuContact; ++i)
    {
        firstPlaneForces_(i) = sol[i];
    }
    firstTorsoTorqueRp_(0) = sol[kIdxTorsoInput + 0];
    firstTorsoTorqueRp_(1) = sol[kIdxTorsoInput + 1];
    Eigen::VectorXd U = Eigen::VectorXd::Zero(nVar);
    for (int i = 0; i < nVar; ++i)
    {
        U(i) = sol[i];
    }
    const Eigen::VectorXd Xpred = Aqp * x0 + Bqp * U;
    if (Xpred.size() >= nx)
    {
        firstTorsoRp_(0) = Xpred(kIdxTorsoRp + 0);
        firstTorsoRp_(1) = Xpred(kIdxTorsoRp + 1);
        firstTorsoRpRate_(0) = Xpred(kIdxTorsoRpRate + 0);
        firstTorsoRpRate_(1) = Xpred(kIdxTorsoRpRate + 1);
    }
    lastTorsoTorqueRp_ = firstTorsoTorqueRp_;
    const Eigen::Matrix<double, 6, 6> HContact = g1_kin_dyn::planeForceToWrenchMap();
    firstWrenches_.segment<6>(0) = HContact * firstPlaneForces_.segment<6>(0);
    firstWrenches_.segment<6>(6) = HContact * firstPlaneForces_.segment<6>(6);
    return firstPlaneForces_.allFinite();
}

void G1SrbdMpcRpEnhanced::dataBusWrite(DataBus &robotState) const
{
    robotState.Fr_ff = firstWrenches_;
    robotState.qpStatus_MPC = qpStatus_;
    robotState.srbd_mpc_rp_enabled = (qpStatus_ == 0);
    robotState.srbd_mpc_torso_tau_rp = firstTorsoTorqueRp_;
    robotState.srbd_mpc_torso_rp_des = firstTorsoRp_;
    robotState.srbd_mpc_torso_rp_rate_des = firstTorsoRpRate_;
}

void G1SrbdMpcRpEnhanced::copyEigenToReal(qpOASES::real_t *target, const Eigen::MatrixXd &source) const
{
    int count = 0;
    for (int i = 0; i < source.rows(); ++i)
    {
        for (int j = 0; j < source.cols(); ++j)
        {
            target[count++] = std::isfinite(source(i, j)) ? source(i, j) : 0.0;
        }
    }
}

void G1SrbdMpcRpEnhanced::copyEigenToReal(qpOASES::real_t *target, const Eigen::VectorXd &source) const
{
    for (int i = 0; i < source.size(); ++i)
    {
        target[i] = std::isfinite(source(i)) ? source(i) : 0.0;
    }
}

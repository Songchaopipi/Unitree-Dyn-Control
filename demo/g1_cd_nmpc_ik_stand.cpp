/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "data_bus.h"
#include "data_logger.h"
#include "g1_centroidal_nmpc.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"

namespace
{
constexpr int kTotalMotorDof = 29;
constexpr int kHorizon = 20;
constexpr int kMpcSolveEveryIterations = 10;   // * 每10个仿真步求解一次NMPC
constexpr double kDefaultMpcSegmentDt = 0.02;  // * 离散化单位时间长度
constexpr double kGravity = 9.80665;
constexpr double kFeedforwardEnableTime = 0.0;  // first 3 s: IK-PD only; after this: full feedforward, no ramp
constexpr double kDefaultFeedforwardScale = 1.0;
constexpr double kDefaultPrintHz = 1.0;
constexpr int kLegMotorCount = 12;

struct RuntimeOptions
{
    bool headless{false};
    bool print{false};
    bool nmpcLog{false};
    bool nmpcSolverLog{false};
    double duration{8.0};
    double standTime{1.0};
    double squatDuration{3.0};
    double squatTargetComZ{std::numeric_limits<double>::quiet_NaN()};
    double mpcSegmentDt{kDefaultMpcSegmentDt};
    double ffScale{kDefaultFeedforwardScale};
    double ffSign{-1.0};
    double nmpcTolerance{1e-5};
    double nmpcMuInit{1e-8};
    int nmpcMaxIterations{30};
    int nmpcMaxAlIterations{2};
    double ikComXyKp{1.0};
    double ikComXyLimit{0.04};
    double printHz{kDefaultPrintHz};
};

const std::vector<std::string> &jointNames()
{
    static const std::vector<std::string> names = {
        "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
        "left_knee", "left_ankle_pitch", "left_ankle_roll",
        "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
        "right_knee", "right_ankle_pitch", "right_ankle_roll",
        "waist_yaw", "waist_roll", "waist_pitch",
        "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
        "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
        "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
        "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"};
    return names;
}

std::vector<double> makeG1StandPose()
{
    std::vector<double> q(kTotalMotorDof, 0.0);
    q[0] = -0.2;
    q[3] = 0.4;
    q[4] = -0.2;
    q[6] = -0.2;
    q[9] = 0.4;
    q[10] = -0.2;

    q[15] = 0.7;
    q[18] = -0.5;
    q[20] = -0.5;
    q[22] = 0.7;
    q[25] = -0.5;
    q[27] = -0.5;
    return q;
}

double totalModelMass(const mjModel *model)
{
    double mass = 0.0;
    for (int i = 0; i < model->nbody; ++i)
    {
        mass += model->body_mass[i];
    }
    return mass;
}

std::string firstExistingPath(const std::string &buildRelative, const std::string &rootRelative)
{
    std::ifstream buildFile(buildRelative);
    if (buildFile.good())
    {
        return buildRelative;
    }
    return rootRelative;
}

double smoothStep(double x)
{
    const double u = std::clamp(x, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

double vectorValueOrZero(const Eigen::VectorXd &value, int index)
{
    return (index >= 0 && index < value.size()) ? value(index) : 0.0;
}


struct CdNmpcTrackingDebug
{
    Eigen::Vector3d com{Eigen::Vector3d::Zero()};
    Eigen::Vector3d comRef{Eigen::Vector3d::Zero()};
    Eigen::Vector3d comErr{Eigen::Vector3d::Zero()};

    Eigen::Vector3d linMom{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linMomRef{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linMomErr{Eigen::Vector3d::Zero()};

    Eigen::Vector3d angMom{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angMomRef{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angMomErr{Eigen::Vector3d::Zero()};

    Eigen::Vector3d predCom{Eigen::Vector3d::Zero()};
    Eigen::Vector3d predComVel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d predComErr{Eigen::Vector3d::Zero()};
    Eigen::Vector3d predLinMom{Eigen::Vector3d::Zero()};
    Eigen::Vector3d predLinMomErr{Eigen::Vector3d::Zero()};

    double comCost{0.0};
    double linMomCost{0.0};
    double angMomCost{0.0};
    double totalStateTrackingCost{0.0};

    double predComCost{0.0};
    double predLinMomCost{0.0};
    double predTotalTrackingCost{0.0};
};

// Debug-only tracking-cost decomposition. These weights are for diagnosis,
// not guaranteed to be identical to the solver's internal weights.
// For exact per-node solver cost, expose the internal cost terms from
// G1CentroidalNmpc directly.
CdNmpcTrackingDebug computeCdNmpcTrackingDebug(const G1CentroidalNmpc::Input &input,
                                               const G1CentroidalNmpc *nmpc = nullptr)
{
    CdNmpcTrackingDebug dbg;
    dbg.com = input.current.head<3>();
    dbg.linMom = input.current.segment<3>(3);
    dbg.angMom = input.current.tail<3>();

    dbg.comRef = input.comReference;
    dbg.linMomRef = input.mass * input.comVelocityReference;
    dbg.angMomRef.setZero();

    dbg.comErr = dbg.com - dbg.comRef;
    dbg.linMomErr = dbg.linMom - dbg.linMomRef;
    dbg.angMomErr = dbg.angMom - dbg.angMomRef;

    const Eigen::Vector3d wCom(120.0, 120.0, 220.0);
    const Eigen::Vector3d wLinMom(1.0, 1.0, 1.0);
    const Eigen::Vector3d wAngMom(1.0, 1.0, 4.0);

    dbg.comCost = 0.5 * dbg.comErr.cwiseAbs2().dot(wCom);
    dbg.linMomCost = 0.5 * dbg.linMomErr.cwiseAbs2().dot(wLinMom);
    dbg.angMomCost = 0.5 * dbg.angMomErr.cwiseAbs2().dot(wAngMom);
    dbg.totalStateTrackingCost = dbg.comCost + dbg.linMomCost + dbg.angMomCost;

    if (nmpc != nullptr)
    {
        dbg.predCom = nmpc->firstComPosition();
        dbg.predComVel = nmpc->firstComVelocity();
        dbg.predLinMom = input.mass * dbg.predComVel;
        dbg.predComErr = dbg.predCom - dbg.comRef;
        dbg.predLinMomErr = dbg.predLinMom - dbg.linMomRef;
        dbg.predComCost = 0.5 * dbg.predComErr.cwiseAbs2().dot(wCom);
        dbg.predLinMomCost = 0.5 * dbg.predLinMomErr.cwiseAbs2().dot(wLinMom);
        dbg.predTotalTrackingCost = dbg.predComCost + dbg.predLinMomCost;
    }
    return dbg;
}

void printVector3Line(const char *label,
                      const Eigen::Vector3d &value,
                      double normValue,
                      double costValue)
{
    std::cout << "  " << std::left << std::setw(13) << label << std::right
              << ": ["
              << std::setw(10) << std::setprecision(5) << value.x() << ", "
              << std::setw(10) << value.y() << ", "
              << std::setw(10) << value.z() << "]"
              << "  norm=" << std::setw(10) << normValue
              << "  cost=" << std::setw(12) << costValue << "\n";
}

void updateRobotState(MJ_Interface &mjInterface, Pin_KinDyn &kinDyn, DataBus &robotState)
{
    mjInterface.updateSensorValues();
    mjInterface.dataBusWrite(robotState);
    kinDyn.dataBusRead(robotState);
    kinDyn.computeJ_dJ();
    kinDyn.computeDyn();
    kinDyn.dataBusWrite(robotState);
}

void tuneStandingPvtGains(PVT_Ctr &pvtCtr)
{
    pvtCtr.setJointPD(360.0, 15.0, "left_hip_pitch_joint");
    pvtCtr.setJointPD(420.0, 20.0, "left_hip_roll_joint");
    pvtCtr.setJointPD(260.0, 10.0, "left_hip_yaw_joint");
    pvtCtr.setJointPD(420.0, 18.0, "left_knee_joint");
    pvtCtr.setJointPD(320.0, 16.0, "left_ankle_pitch_joint");
    pvtCtr.setJointPD(320.0, 16.0, "left_ankle_roll_joint");

    pvtCtr.setJointPD(360.0, 15.0, "right_hip_pitch_joint");
    pvtCtr.setJointPD(420.0, 20.0, "right_hip_roll_joint");
    pvtCtr.setJointPD(260.0, 10.0, "right_hip_yaw_joint");
    pvtCtr.setJointPD(420.0, 18.0, "right_knee_joint");
    pvtCtr.setJointPD(320.0, 16.0, "right_ankle_pitch_joint");
    pvtCtr.setJointPD(320.0, 16.0, "right_ankle_roll_joint");
}

Eigen::VectorXd contactPlaneForceFeedforward(const DataBus &robotState,
                                             const Eigen::Matrix<double, 12, 1> &contactPlaneForces,
                                             double scale,
                                             double jacobianTransposeSign)
{
    Eigen::VectorXd tauMotor = Eigen::VectorXd::Zero(robotState.model_nv - 6);
    const double sign = jacobianTransposeSign >= 0.0 ? 1.0 : -1.0;
    if (robotState.J_l_foot.rows() >= 6 && robotState.J_l_foot.cols() == robotState.model_nv)
    {
        // J_l_foot is the RoMoCo contact-plane constraint Jacobian, so it pairs
        // with firstPlaneForces(), not the spatial 6D foot wrench.
        tauMotor.head(kLegMotorCount / 2).noalias() += sign *
            robotState.J_l_foot.block(0, 6, 6, kLegMotorCount / 2).transpose() *
            contactPlaneForces.segment<6>(0);
    }
    if (robotState.J_r_foot.rows() >= 6 && robotState.J_r_foot.cols() == robotState.model_nv)
    {
        tauMotor.segment(kLegMotorCount / 2, kLegMotorCount / 2).noalias() += sign *
            robotState.J_r_foot.block(0, 12, 6, kLegMotorCount / 2).transpose() *
            contactPlaneForces.segment<6>(6);
    }

    tauMotor *= scale;
    for (int i = 0; i < tauMotor.size(); ++i)
    {
        const double limit = g1_kin_dyn::motorTorqueLimit(i);
        tauMotor(i) = std::clamp(tauMotor(i), -limit, limit);
    }
    return tauMotor;
}

void fillStandBus(DataBus &robotState,
                  const Eigen::Vector3d &comDes,
                  double yawDes,
                  double targetComHeight,
                  double nominalFootForce)
{
    robotState.motionState = DataBus::Stand;
    robotState.legState = DataBus::DSt;
    robotState.walk_stance_leg = DataBus::DSt;
    robotState.walk_left_contact = true;
    robotState.walk_right_contact = true;
    robotState.walk_is_double_support = true;
    robotState.walk_time_to_impact = 0.0;
    robotState.phi = 0.0;
    robotState.stance_fe_pos_cur_W = 0.5 * (robotState.fe_l_pos_W + robotState.fe_r_pos_W);
    robotState.stance_fe_rot_cur_W = robotState.fe_l_rot_W;
    robotState.walk_target_yaw = yawDes;
    robotState.walk_target_yaw_rate = 0.0;
    robotState.walk_target_com_height = targetComHeight;
    robotState.base_rpy_des << 0.0, 0.0, yawDes;
    robotState.base_vel_des.setZero();
    robotState.base_omega_des.setZero();
    robotState.desV_W.setZero();
    robotState.js_vel_des.setZero();
    robotState.js_omega_des.setZero();
    robotState.centroidal_nmpc_enabled = true;
    robotState.centroidal_nmpc_com_pos_des = comDes;
    robotState.centroidal_nmpc_com_vel_des.setZero();
    robotState.centroidal_nmpc_com_acc_des.setZero();
    robotState.Fr_ff.setZero(12);
    robotState.Fr_ff.segment<6>(0) << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;
    robotState.Fr_ff.segment<6>(6) << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;
}

void printLegTorqueBreakdown(const std::vector<double> &tauTotal,
                             const Eigen::VectorXd &tauFeedforward,
                             int start,
                             int count)
{
    const auto &names = jointNames();
    std::cout << "       joint                     total      IK-PD     NMPC-ff\n";
    for (int i = start; i < start + count && i < static_cast<int>(tauTotal.size()); ++i)
    {
        const std::string &name = (i < static_cast<int>(names.size())) ? names[i] : std::to_string(i);
        const double total = tauTotal[i];
        const double ff = vectorValueOrZero(tauFeedforward, i);
        const double ikPd = total - ff;
        std::cout << "  [" << std::setw(2) << i << "] "
                  << std::left << std::setw(22) << name << std::right
                  << std::setw(10) << std::fixed << std::setprecision(3) << total
                  << std::setw(10) << ikPd
                  << std::setw(10) << ff << " Nm\n";
    }
}

void printRuntimeSummary(double time,
                         const DataBus &robotState,
                         const Eigen::Vector3d &comDes,
                         const G1CentroidalNmpc &nmpc,
                         const Eigen::Matrix<double, 12, 1> &contactPlaneForces,
                         const Eigen::VectorXd &tauFeedforward,
                         const CdNmpcTrackingDebug &trackingDebug,
                         double ffSign,
                         int ikStatus,
                         double ikErrNorm,
                         double ffScale,
                         double ffSwitch)
{
    const Eigen::Vector3d velDesWorld = Eigen::Vector3d::Zero();
    const Eigen::Vector2d swingPhase = Eigen::Vector2d::Zero();
    const bool leftContact = robotState.walk_left_contact;
    const bool rightContact = robotState.walk_right_contact;

    std::cout << "\n"
              << "======== G1 Aligator CD-NMPC + IK-PD Stand  t="
              << std::fixed << std::setprecision(2) << time << " s ========\n";

    std::cout << "contact L/R: " << (leftContact ? 1 : 0) << " / " << (rightContact ? 1 : 0)
              << "   swing phase L/R: " << std::setprecision(3) << swingPhase.x()
              << " / " << swingPhase.y()
              << "   tSwing/tDS/tImpact: " << robotState.tSwing
              << " / " << robotState.tDoubleSupport
              << " / " << robotState.walk_time_to_impact
              << "   v_des_W: [" << velDesWorld.x() << ", " << velDesWorld.y() << ", " << velDesWorld.z() << "]\n";

    std::cout << "CoM cur/des/nmpc: [" << std::setprecision(4) << robotState.pCoM_W.x()
              << ", " << robotState.pCoM_W.y()
              << ", " << robotState.pCoM_W.z() << "] / ["
              << comDes.x() << ", " << comDes.y() << ", " << comDes.z() << "] / ["
              << nmpc.firstComPosition().x() << ", " << nmpc.firstComPosition().y()
              << ", " << nmpc.firstComPosition().z() << "]"
              << "   base rpy: [" << robotState.base_rpy.x()
              << ", " << robotState.base_rpy.y()
              << ", " << robotState.base_rpy.z() << "]"
              << "   nmpc status/iter/time: " << nmpc.status()
              << " / " << nmpc.iterations()
              << " / " << nmpc.solveTime() * 1e3 << " ms\n";

    std::cout << "IK status/err: " << ikStatus
              << " / " << std::setprecision(6) << ikErrNorm
              << "   ff scale/switch/sign: " << ffScale
              << " / " << ffSwitch
              << " / " << (ffSign >= 0.0 ? "+1" : "-1") << "\n";

    std::cout << "\nCD-NMPC current-state tracking debug cost\n";
    std::cout << "  state vector: x = [com, linear momentum, angular momentum]\n";
    printVector3Line("com err", trackingDebug.comErr,
                     trackingDebug.comErr.norm(), trackingDebug.comCost);
    printVector3Line("lin mom err", trackingDebug.linMomErr,
                     trackingDebug.linMomErr.norm(), trackingDebug.linMomCost);
    printVector3Line("ang mom err", trackingDebug.angMomErr,
                     trackingDebug.angMomErr.norm(), trackingDebug.angMomCost);
    std::cout << "  " << std::left << std::setw(13) << "state total" << std::right
              << ": " << std::setw(12) << std::setprecision(6)
              << trackingDebug.totalStateTrackingCost << "\n";
    std::cout << "  current com     : [" << std::setw(10) << trackingDebug.com.x()
              << ", " << std::setw(10) << trackingDebug.com.y()
              << ", " << std::setw(10) << trackingDebug.com.z() << "]\n";
    std::cout << "  current lin mom : [" << std::setw(10) << trackingDebug.linMom.x()
              << ", " << std::setw(10) << trackingDebug.linMom.y()
              << ", " << std::setw(10) << trackingDebug.linMom.z() << "]\n";
    std::cout << "  current ang mom : [" << std::setw(10) << trackingDebug.angMom.x()
              << ", " << std::setw(10) << trackingDebug.angMom.y()
              << ", " << std::setw(10) << trackingDebug.angMom.z() << "]\n";

    std::cout << "\nCD-NMPC first predicted CoM / linear momentum tracking\n";
    printVector3Line("pred com err", trackingDebug.predComErr,
                     trackingDebug.predComErr.norm(), trackingDebug.predComCost);
    printVector3Line("pred lin mom", trackingDebug.predLinMomErr,
                     trackingDebug.predLinMomErr.norm(), trackingDebug.predLinMomCost);
    std::cout << "  " << std::left << std::setw(13) << "pred total" << std::right
              << ": " << std::setw(12) << std::setprecision(6)
              << trackingDebug.predTotalTrackingCost << "\n";

    std::cout << "\nNMPC desired wrench [Fx Fy Fz Mx My Mz]\n";
    std::cout << "  L: "
              << std::setw(9) << std::setprecision(3) << vectorValueOrZero(robotState.Fr_ff, 0)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 1)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 2)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 3)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 4)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 5) << "\n";
    std::cout << "  R: "
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 6)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 7)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 8)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 9)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 10)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 11) << "\n";

    std::cout << "Contact-plane forces used by J^T, ff sign=" << (ffSign >= 0.0 ? "+1" : "-1") << "\n";
    std::cout << "  L: "
              << std::setw(9) << contactPlaneForces(0)
              << std::setw(9) << contactPlaneForces(1)
              << std::setw(9) << contactPlaneForces(2)
              << std::setw(9) << contactPlaneForces(3)
              << std::setw(9) << contactPlaneForces(4)
              << std::setw(9) << contactPlaneForces(5) << "\n";
    std::cout << "  R: "
              << std::setw(9) << contactPlaneForces(6)
              << std::setw(9) << contactPlaneForces(7)
              << std::setw(9) << contactPlaneForces(8)
              << std::setw(9) << contactPlaneForces(9)
              << std::setw(9) << contactPlaneForces(10)
              << std::setw(9) << contactPlaneForces(11) << "\n";

    std::cout << "\nLeg torque breakdown [Nm]\n";
    std::cout << "Left leg\n";
    printLegTorqueBreakdown(robotState.motors_tor_out, tauFeedforward, 0, 6);
    std::cout << "\nRight leg\n";
    printLegTorqueBreakdown(robotState.motors_tor_out, tauFeedforward, 6, 6);
    std::cout << "\n";
}

RuntimeOptions parseOptions(int argc, const char **argv)
{
    RuntimeOptions options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--headless")
        {
            options.headless = true;
        }
        else if (arg == "--print")
        {
            options.print = true;
        }
        else if (arg == "--nmpc-log")
        {
            options.nmpcLog = true;
        }
        else if (arg == "--nmpc-solver-log")
        {
            options.nmpcLog = true;
            options.nmpcSolverLog = true;
        }
        else if (arg == "--duration" && i + 1 < argc)
        {
            options.duration = std::stod(argv[++i]);
        }
        else if (arg == "--stand-time" && i + 1 < argc)
        {
            options.standTime = std::max(0.0, std::stod(argv[++i]));
        }
        else if (arg == "--squat-duration" && i + 1 < argc)
        {
            options.squatDuration = std::max(1e-3, std::stod(argv[++i]));
        }
        else if ((arg == "--squat-target" || arg == "--com-z") && i + 1 < argc)
        {
            options.squatTargetComZ = std::stod(argv[++i]);
        }
        else if (arg == "--mpc-dt" && i + 1 < argc)
        {
            options.mpcSegmentDt = std::max(0.005, std::stod(argv[++i]));
        }
        else if (arg == "--ff-scale" && i + 1 < argc)
        {
            options.ffScale = std::stod(argv[++i]);
        }
        else if (arg == "--ff-sign" && i + 1 < argc)
        {
            options.ffSign = std::stod(argv[++i]) >= 0.0 ? 1.0 : -1.0;
        }
        else if (arg == "--nmpc-max-iter" && i + 1 < argc)
        {
            options.nmpcMaxIterations = std::max(1, std::stoi(argv[++i]));
        }
        else if (arg == "--nmpc-max-al-iter" && i + 1 < argc)
        {
            options.nmpcMaxAlIterations = std::max(1, std::stoi(argv[++i]));
        }
        else if (arg == "--nmpc-tol" && i + 1 < argc)
        {
            options.nmpcTolerance = std::max(1e-10, std::stod(argv[++i]));
        }
        else if (arg == "--nmpc-mu-init" && i + 1 < argc)
        {
            options.nmpcMuInit = std::max(1e-12, std::stod(argv[++i]));
        }
        else if (arg == "--ik-com-xy-kp" && i + 1 < argc)
        {
            options.ikComXyKp = std::max(0.0, std::stod(argv[++i]));
        }
        else if (arg == "--ik-com-xy-limit" && i + 1 < argc)
        {
            options.ikComXyLimit = std::max(0.0, std::stod(argv[++i]));
        }
        else if (arg == "--print-hz" && i + 1 < argc)
        {
            options.printHz = std::max(0.05, std::stod(argv[++i]));
        }
    }
    return options;
}
} // namespace

char error[1000] = "Could not load binary model";
mjModel *mj_model = nullptr;
mjData *mj_data = nullptr;

int main(int argc, const char **argv)
{
    const RuntimeOptions options = parseOptions(argc, argv);

    const std::string modelXml = firstExistingPath("../models/g1/g1_29_withsensor.xml",
                                                   "models/g1/g1_29_withsensor.xml");
    const std::string modelUrdf = firstExistingPath("../models/g1/g1_29_withsensor.urdf",
                                                    "models/g1/g1_29_withsensor.urdf");
    const std::string jointConfig = firstExistingPath("../common/joint_ctrl_config.json",
                                                      "common/joint_ctrl_config.json");
    const std::string logPath =
        firstExistingPath("../record/matlabReadDataScript.txt", "record/matlabReadDataScript.txt") ==
                "../record/matlabReadDataScript.txt" ?
            "../record/g1_cd_nmpc_ik_stand.log" :
            "record/g1_cd_nmpc_ik_stand.log";

    mj_model = mj_loadXML(modelXml.c_str(), 0, error, 1000);
    if (mj_model)
    {
        mj_data = mj_makeData(mj_model);
    }
    if (!mj_model || !mj_data)
    {
        std::cerr << error << std::endl;
        return 1;
    }

    UIctr uiController(mj_model, mj_data);
    MJ_Interface mjInterface(mj_model, mj_data);
    Pin_KinDyn kinDyn(modelUrdf);
    DataBus robotState(kinDyn.model_nv);
    PVT_Ctr pvtCtr(mj_model->opt.timestep, jointConfig.c_str());
    tuneStandingPvtGains(pvtCtr);

    const double simDt = mj_model->opt.timestep;
    (void)simDt;
    G1CentroidalNmpc centroidalNmpc(kHorizon, options.mpcSegmentDt);
    centroidalNmpc.maxIterations = options.nmpcMaxIterations;
    centroidalNmpc.maxAlIterations = options.nmpcMaxAlIterations;
    centroidalNmpc.tolerance = options.nmpcTolerance;
    centroidalNmpc.muInit = options.nmpcMuInit;
    centroidalNmpc.logSolve = options.nmpcLog;
    centroidalNmpc.logSolverIterations = options.nmpcSolverLog;
    DataLogger logger(logPath);

    const std::vector<double> standPoseStd = makeG1StandPose();
    const Eigen::VectorXd standPose =
        Eigen::Map<const Eigen::VectorXd>(standPoseStd.data(), standPoseStd.size());
    const double modelMass = totalModelMass(mj_model);
    const double nominalFootForce = 0.5 * modelMass * kGravity;

    mjInterface.setMotorsPosition(standPoseStd);
    updateRobotState(mjInterface, kinDyn, robotState);
    pvtCtr.motor_pos_des_old = standPoseStd;

    const Eigen::Vector3d initialBasePos = robotState.base_pos;
    const Eigen::Vector3d initialCom = robotState.pCoM_W;
    const Eigen::Vector3d baseMinusCom = initialBasePos - initialCom;
    const double initialYaw = robotState.base_rpy.z();
    const double targetComZ = std::isfinite(options.squatTargetComZ) ?
                                  std::min(options.squatTargetComZ, initialCom.z()) :
                                  initialCom.z();
    const Eigen::Matrix3d initialBaseRot = robotState.base_rot;
    const double groundZ = 0.5 * (robotState.fe_l_pos_W.z() + robotState.fe_r_pos_W.z());
    const double targetComHeight = targetComZ - groundZ;
    const std::array<Eigen::Matrix3d, 2> initialFootRotW = {robotState.fe_l_rot_W, robotState.fe_r_rot_W};
    const std::array<Eigen::Vector3d, 2> footHoldW = {robotState.fe_l_pos_W, robotState.fe_r_pos_W};
    const Eigen::VectorXd standHoldPose =
        Eigen::Map<const Eigen::VectorXd>(robotState.motors_pos_cur.data(), robotState.model_nv - 6);
    const Eigen::VectorXd lockedUpperBodyPose = standHoldPose;

    Eigen::VectorXd ikStandPose = standPose;
    Eigen::Matrix<double, 12, 1> lastNmpcPlaneForces = Eigen::Matrix<double, 12, 1>::Zero();
    lastNmpcPlaneForces.segment<6>(0) = g1_kin_dyn::distributeVerticalFootLoad(nominalFootForce);
    lastNmpcPlaneForces.segment<6>(6) = g1_kin_dyn::distributeVerticalFootLoad(nominalFootForce);
    Eigen::Matrix<double, 12, 1> lastNmpcWrench = Eigen::Matrix<double, 12, 1>::Zero();
    lastNmpcWrench << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0,
        0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;

    logger.addIterm("simTime", 1);
    logger.addIterm("baseRpy", 3);
    logger.addIterm("basePos", 3);
    logger.addIterm("com", 3);
    logger.addIterm("comDes", 3);
    logger.addIterm("nmpcCom", 3);
    logger.addIterm("nmpcComVel", 3);
    logger.addIterm("nmpcComAcc", 3);
    logger.addIterm("cdComErr", 3);
    logger.addIterm("cdLinMomErr", 3);
    logger.addIterm("cdAngMomErr", 3);
    logger.addIterm("cdTrackCost", 4);
    logger.addIterm("cdPredComErr", 3);
    logger.addIterm("cdPredLinMomErr", 3);
    logger.addIterm("cdPredTrackCost", 3);
    logger.addIterm("ikStatus", 1);
    logger.addIterm("ikErrNorm", 1);
    logger.addIterm("Fr_ff", 12);
    logger.addIterm("tau_ff", 29);
    logger.addIterm("nmpcStatus", 1);
    logger.addIterm("nmpcIter", 1);
    logger.addIterm("nmpcCpuTime", 1);
    logger.finishItermAdding();

    if (!options.headless)
    {
        uiController.iniGLFW();
        uiController.enableTracking();
        uiController.createWindow("G1 Aligator CD-NMPC + IK-PD Stand", false);
    }

    int iterationCounter = 0;
    bool hasNmpcSolution = false;
    double nextPrintTime = 0.0;
    int ikStatus = 0;
    double ikErrNorm = 0.0;
    CdNmpcTrackingDebug cdNmpcTrackingDebug;

    while ((options.headless || !glfwWindowShouldClose(uiController.window)) && mj_data->time < options.duration)
    {
        const mjtNum simStart = mj_data->time;
        while ((options.headless || uiController.runSim) &&
               mj_data->time - simStart < 1.0 / 60.0 &&
               mj_data->time < options.duration)
        {
            mj_step(mj_model, mj_data);
            updateRobotState(mjInterface, kinDyn, robotState);

            const double time = mj_data->time;
            const double squatPhase = smoothStep((time - options.standTime) / options.squatDuration);
            Eigen::Vector3d comDes = initialCom;
            comDes.z() = initialCom.z() + (targetComZ - initialCom.z()) * squatPhase;
            const double yawDes = initialYaw;

            fillStandBus(robotState, comDes, yawDes, targetComHeight, nominalFootForce);

            if (iterationCounter % kMpcSolveEveryIterations == 0)
            {
                G1CentroidalNmpc::Input nmpcInput;
                nmpcInput.mass = modelMass;
                nmpcInput.current.head<3>() = robotState.pCoM_W;
                Eigen::Vector3d comVel = Eigen::Vector3d::Zero();
                if (robotState.Jcom_W.rows() == 3 && robotState.Jcom_W.cols() == robotState.dq.size())
                {
                    comVel = robotState.Jcom_W * robotState.dq;
                }
                else if (robotState.dq.size() >= 3)
                {
                    comVel = robotState.dq.head<3>();
                }
                nmpcInput.current.segment<3>(3) = modelMass * comVel;
                nmpcInput.current.tail<3>().setZero();
                if (robotState.dyn_Ag.rows() >= 6 && robotState.dyn_Ag.cols() == robotState.dq.size())
                {
                    nmpcInput.current.tail<3>() = (robotState.dyn_Ag * robotState.dq).tail<3>();
                }
                nmpcInput.comReference = comDes;
                nmpcInput.comVelocityReference.setZero();
                nmpcInput.comAccelerationReference.setZero();
                nmpcInput.contactPositionWorld = {robotState.fe_l_pos_W, robotState.fe_r_pos_W};
                nmpcInput.contactActive = {{true, true}};

                cdNmpcTrackingDebug = computeCdNmpcTrackingDebug(nmpcInput);
                if (centroidalNmpc.solve(nmpcInput))
                {
                    lastNmpcWrench = centroidalNmpc.firstWrenches();
                    lastNmpcPlaneForces = centroidalNmpc.firstPlaneForces();
                    centroidalNmpc.dataBusWrite(robotState);
                    cdNmpcTrackingDebug = computeCdNmpcTrackingDebug(nmpcInput, &centroidalNmpc);
                    hasNmpcSolution = true;
                }
                else
                {
                    robotState.qpStatus_MPC = centroidalNmpc.status();
                    robotState.centroidal_nmpc_status = centroidalNmpc.status();
                }
            }
            if (hasNmpcSolution)
            {
                robotState.Fr_ff = lastNmpcWrench;
            }

            const Eigen::Matrix3d yawDelta = Rz3(yawDes - initialYaw);
            const Eigen::Matrix3d leftFootRotWDes = yawDelta * initialFootRotW[0];
            const Eigen::Matrix3d rightFootRotWDes = yawDelta * initialFootRotW[1];
            Eigen::Vector3d basePosForIk = initialBasePos + (comDes - initialCom);
            const Eigen::Vector2d comXyCorrection =
                (options.ikComXyKp * (comDes.head<2>() - robotState.pCoM_W.head<2>()))
                    .cwiseMax(-options.ikComXyLimit)
                    .cwiseMin(options.ikComXyLimit);
            basePosForIk.head<2>() += comXyCorrection;
            const double baseZDes = comDes.z() + baseMinusCom.z();
            const double comZCorrection =
                std::clamp(comDes.z() - robotState.pCoM_W.z(), -0.04, 0.04);
            basePosForIk.z() = baseZDes + comZCorrection;
            const Eigen::Matrix3d baseRotForIk = yawDelta * initialBaseRot;
            const Eigen::Vector3d leftFootPosLDes = baseRotForIk.transpose() * (footHoldW[0] - basePosForIk);
            const Eigen::Vector3d rightFootPosLDes = baseRotForIk.transpose() * (footHoldW[1] - basePosForIk);
            const Eigen::Matrix3d leftFootRotLDes = baseRotForIk.transpose() * leftFootRotWDes;
            const Eigen::Matrix3d rightFootRotLDes = baseRotForIk.transpose() * rightFootRotWDes;
            Pin_KinDyn::IkRes ikRes =
                kinDyn.computeInK_Leg(leftFootRotLDes, leftFootPosLDes, rightFootRotLDes, rightFootPosLDes);
            ikStatus = ikRes.status;
            ikErrNorm = ikRes.err.norm();
            if (ikRes.err.norm() < 8e-2 && ikRes.jointPosRes.size() >= 12)
            {
                ikStandPose.head(kLegMotorCount) = ikRes.jointPosRes.head(kLegMotorCount);
            }
            ikStandPose.tail(ikStandPose.size() - kLegMotorCount) =
                lockedUpperBodyPose.tail(lockedUpperBodyPose.size() - kLegMotorCount);

            const double ikRamp = std::clamp(time / 0.5, 0.0, 1.0);
            const Eigen::VectorXd motorPosDes = (1.0 - ikRamp) * standHoldPose + ikRamp * ikStandPose;
            robotState.motors_pos_des = eigen2std(motorPosDes);
            robotState.motors_vel_des.assign(robotState.model_nv - 6, 0.0);
            const double ffSwitch = (time >= kFeedforwardEnableTime) ? 1.0 : 0.0;
            const Eigen::VectorXd tauFeedforward =
                contactPlaneForceFeedforward(robotState, lastNmpcPlaneForces,
                                             ffSwitch * options.ffScale, options.ffSign);
            robotState.motors_tor_des = eigen2std(tauFeedforward);

            pvtCtr.dataBusRead(robotState);
            pvtCtr.calMotorsPVT(0.02);
            pvtCtr.dataBusWrite(robotState);
            mjInterface.setMotorsTorque(robotState.motors_tor_out);

            if (options.print && time + 1e-12 >= nextPrintTime)
            {
                printRuntimeSummary(time, robotState, comDes, centroidalNmpc,
                                    lastNmpcPlaneForces, tauFeedforward,
                                    cdNmpcTrackingDebug, options.ffSign,
                                    ikStatus, ikErrNorm, options.ffScale, ffSwitch);
                nextPrintTime += 1.0 / options.printHz;
                if (nextPrintTime < time)
                {
                    nextPrintTime = time + 1.0 / options.printHz;
                }
            }

            logger.startNewLine();
            logger.recItermData("simTime", time);
            logger.recItermData("baseRpy", robotState.base_rpy);
            logger.recItermData("basePos", robotState.base_pos);
            logger.recItermData("com", robotState.pCoM_W);
            logger.recItermData("comDes", comDes);
            logger.recItermData("nmpcCom", centroidalNmpc.firstComPosition());
            logger.recItermData("nmpcComVel", centroidalNmpc.firstComVelocity());
            logger.recItermData("nmpcComAcc", centroidalNmpc.firstComAcceleration());
            logger.recItermData("cdComErr", cdNmpcTrackingDebug.comErr);
            logger.recItermData("cdLinMomErr", cdNmpcTrackingDebug.linMomErr);
            logger.recItermData("cdAngMomErr", cdNmpcTrackingDebug.angMomErr);
            logger.recItermData("cdTrackCost", (Eigen::Vector4d() << cdNmpcTrackingDebug.comCost,
                                                cdNmpcTrackingDebug.linMomCost,
                                                cdNmpcTrackingDebug.angMomCost,
                                                cdNmpcTrackingDebug.totalStateTrackingCost)
                                                   .finished());
            logger.recItermData("cdPredComErr", cdNmpcTrackingDebug.predComErr);
            logger.recItermData("cdPredLinMomErr", cdNmpcTrackingDebug.predLinMomErr);
            logger.recItermData("cdPredTrackCost", (Eigen::Vector3d() << cdNmpcTrackingDebug.predComCost,
                                                    cdNmpcTrackingDebug.predLinMomCost,
                                                    cdNmpcTrackingDebug.predTotalTrackingCost)
                                                       .finished());
            logger.recItermData("ikStatus", static_cast<double>(ikStatus));
            logger.recItermData("ikErrNorm", ikErrNorm);
            logger.recItermData("Fr_ff", robotState.Fr_ff);
            logger.recItermData("tau_ff", tauFeedforward);
            logger.recItermData("nmpcStatus", static_cast<double>(centroidalNmpc.status()));
            logger.recItermData("nmpcIter", static_cast<double>(centroidalNmpc.iterations()));
            logger.recItermData("nmpcCpuTime", centroidalNmpc.solveTime());
            logger.finishLine();

            ++iterationCounter;
        }

        if (!options.headless)
        {
            uiController.updateScene();
        }
    }

    if (!options.headless)
    {
        uiController.Close();
    }
    return 0;
}
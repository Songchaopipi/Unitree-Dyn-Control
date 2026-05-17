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
#include "foot_placement.h"
#include "../g1_kinodynamics_nmpc.h"
#include "gait_scheduler.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"

namespace
{
constexpr int kTotalMotorDof = 29;
constexpr int kLegMotorCount = 12;
constexpr int kDefaultHorizon = 12;
constexpr int kDefaultMpcSolveEveryIterations = 5;
constexpr double kDefaultMpcSegmentDt = 0.02;
constexpr double kDefaultSingleSupportTime = 0.3;
constexpr double kDefaultDoubleSupportTime = 0.0;
constexpr double kGravity = 9.80665;
constexpr double kFeedforwardRampTime = 0.0;
constexpr double kDefaultPrintHz = 1.0;
constexpr double kPreWalkStanceShiftY = 0.035;
constexpr double kIkRampTime = 0.5;

struct RuntimeOptions
{
    bool headless{false};
    bool print{false};
    double duration{13.0};
    double standTime{1.0};
    double squatDuration{4.0};
    double squatHoldTime{1.0};
    double squatTargetComZ{0.65};
    double squatDrop{0.02};
    double maxSquatDrop{std::numeric_limits<double>::infinity()};
    bool useSquatDrop{false};
    double shiftY{kPreWalkStanceShiftY};
    int horizon{kDefaultHorizon};
    int mpcSolveEveryIterations{kDefaultMpcSolveEveryIterations};
    double mpcSegmentDt{kDefaultMpcSegmentDt};
    double singleSupportTime{kDefaultSingleSupportTime};
    double doubleSupportTime{kDefaultDoubleSupportTime};
    double vx{0.08};
    double vy{0.0};
    double yawRate{0.0};
    double velocityRamp{2.0};
    double swingHeight{0.08};
    int demoSteps{std::numeric_limits<int>::max()};
    double ffScale{1.0};
    double wrenchSign{-1.0};
    double nmpcTolerance{1e-5};
    double nmpcMuInit{1e-8};
    int nmpcMaxIterations{1};
    int nmpcMaxAlIterations{2};
    double nmpcLineSearchAlphaMin{1e-3};
    int nmpcMaxLineSearchSteps{2};
    int nmpcThreads{1};
    bool nmpcLinearRollout{false};
    bool nmpcParallelLq{false};
    double printHz{kDefaultPrintHz};
};

// 函数说明：返回当前支撑腿的对侧，用于交替规划摆动腿。
DataBus::LegState oppositeStance(DataBus::LegState stance)
{
    return stance == DataBus::LSt ? DataBus::RSt : DataBus::LSt;
}

struct GaitSample
{
    bool leftContact{true};
    bool rightContact{true};
    bool doubleSupport{true};
    DataBus::LegState stanceLeg{DataBus::LSt};
    DataBus::LegState legState{DataBus::DSt};
    double phi{0.0};
    double timeToImpact{0.0};
};

// 函数说明：按预览时间推演步态相位，得到未来接触模式。
GaitSample previewGaitSample(const DataBus &robotState, double previewTime)
{
    GaitSample out;
    if (robotState.motionState != DataBus::Walk)
    {
        return out;
    }

    const double singleSupportTime = std::max(1e-3, robotState.tSwing);
    const double doubleSupportTime = std::max(0.0, robotState.tDoubleSupport);
    bool doubleSupport = robotState.walk_is_double_support;
    DataBus::LegState stanceLeg = robotState.walk_stance_leg;
    if (stanceLeg != DataBus::LSt && stanceLeg != DataBus::RSt)
    {
        stanceLeg = DataBus::LSt;
    }
    double phaseElapsed = doubleSupport ?
                              std::clamp(robotState.walk_phase_time, 0.0, doubleSupportTime) :
                              std::clamp(robotState.phi, 0.0, 1.0) * singleSupportTime;
    double remainingTime = std::max(0.0, previewTime);

    auto advancePhase = [&]()
    {
        phaseElapsed = 0.0;
        if (doubleSupport)
        {
            stanceLeg = oppositeStance(stanceLeg);
            doubleSupport = false;
        }
        else if (doubleSupportTime > 1e-9)
        {
            doubleSupport = true;
        }
        else
        {
            stanceLeg = oppositeStance(stanceLeg);
            doubleSupport = false;
        }
    };

    while (remainingTime > 1e-12)
    {
        const double phaseDuration = doubleSupport ? doubleSupportTime : singleSupportTime;
        const double phaseRemaining = std::max(0.0, phaseDuration - phaseElapsed);
        if (remainingTime <= phaseRemaining + 1e-12)
        {
            phaseElapsed += remainingTime;
            break;
        }
        remainingTime -= phaseRemaining;
        advancePhase();
    }

    out.doubleSupport = doubleSupport;
    out.stanceLeg = stanceLeg;
    out.legState = doubleSupport ? DataBus::DSt : stanceLeg;
    out.leftContact = doubleSupport || stanceLeg == DataBus::LSt;
    out.rightContact = doubleSupport || stanceLeg == DataBus::RSt;
    out.phi = doubleSupport ? 0.0 : std::clamp(phaseElapsed / singleSupportTime, 0.0, 1.0);
    out.timeToImpact = doubleSupport ? 0.0 : std::max(0.0, singleSupportTime - phaseElapsed);
    return out;
}

// 函数说明：根据步态预览生成 horizon 内的左右脚接触表。
Eigen::Matrix<int, Eigen::Dynamic, G1KinodynamicsNmpc::kNumFeet>
previewContactTable(const DataBus &robotState, int horizon, double segmentDt)
{
    Eigen::Matrix<int, Eigen::Dynamic, G1KinodynamicsNmpc::kNumFeet> table(
        std::max(1, horizon), G1KinodynamicsNmpc::kNumFeet);
    if (robotState.motionState != DataBus::Walk)
    {
        table.setOnes();
        return table;
    }

    for (int i = 0; i < table.rows(); ++i)
    {
        const GaitSample future = previewGaitSample(robotState, static_cast<double>(i) * segmentDt);
        table(i, 0) = future.leftContact ? 1 : 0;
        table(i, 1) = future.rightContact ? 1 : 0;
    }
    return table;
}

// 函数说明：按步态预览生成每个 running knot 的足端位姿参考。
std::vector<std::array<pinocchio::SE3, G1KinodynamicsNmpc::kNumFeet>>
previewFootPoseReferences(const DataBus &robotState,
                          const std::array<Eigen::Vector3d, G1KinodynamicsNmpc::kNumFeet> &currentFootPositions,
                          const std::array<Eigen::Vector3d, G1KinodynamicsNmpc::kNumFeet> &touchdownFootPositions,
                          const std::array<Eigen::Matrix3d, G1KinodynamicsNmpc::kNumFeet> &footRotations,
                          int horizon,
                          double segmentDt)
{
    const int rows = std::max(1, horizon);
    std::vector<std::array<pinocchio::SE3, G1KinodynamicsNmpc::kNumFeet>> refs(
        static_cast<std::size_t>(rows));
    for (int k = 0; k < rows; ++k)
    {
        std::array<Eigen::Vector3d, G1KinodynamicsNmpc::kNumFeet> positions = currentFootPositions;
        if (robotState.motionState == DataBus::Walk)
        {
            const GaitSample future = previewGaitSample(robotState, static_cast<double>(k) * segmentDt);
            if (future.leftContact && !robotState.walk_left_contact)
            {
                positions[0] = touchdownFootPositions[0];
            }
            if (future.rightContact && !robotState.walk_right_contact)
            {
                positions[1] = touchdownFootPositions[1];
            }
        }
        refs[static_cast<std::size_t>(k)] = {{
            pinocchio::SE3(footRotations[0], positions[0]),
            pinocchio::SE3(footRotations[1], positions[1]),
        }};
    }
    return refs;
}

// 函数说明：从 DataBus 中读取左右摆动相位。
Eigen::Vector2d swingPhaseFromRobotState(const DataBus &robotState)
{
    Eigen::Vector2d phase = Eigen::Vector2d::Zero();
    if (robotState.motionState != DataBus::Walk || robotState.walk_is_double_support)
    {
        return phase;
    }

    if (robotState.walk_stance_leg == DataBus::LSt)
    {
        phase(1) = std::clamp(robotState.phi, 0.0, 1.0);
    }
    else if (robotState.walk_stance_leg == DataBus::RSt)
    {
        phase(0) = std::clamp(robotState.phi, 0.0, 1.0);
    }
    return phase;
}

// 函数说明：返回 G1 站立初始关节姿态。
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

// 函数说明：从 MuJoCo 模型中累计所有 body 质量。
double totalModelMass(const mjModel *model)
{
    double mass = 0.0;
    for (int i = 0; i < model->nbody; ++i)
    {
        mass += model->body_mass[i];
    }
    return mass;
}

// 函数说明：优先返回 build 相对路径，否则返回仓库根目录相对路径。
std::string firstExistingPath(const std::string &buildRelative, const std::string &rootRelative)
{
    std::ifstream buildFile(buildRelative);
    if (buildFile.good())
    {
        return buildRelative;
    }
    return rootRelative;
}

// 函数说明：安全读取向量指定下标，越界时返回 0。
double vectorValueOrZero(const Eigen::VectorXd &value, int index)
{
    return (index >= 0 && index < value.size()) ? value(index) : 0.0;
}

// 函数说明：三次 smooth step，用于下蹲、偏移和速度指令平滑。
double smoothStep(double x)
{
    const double u = std::clamp(x, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

// 函数说明：把旋转和平移打包成 Pinocchio SE3。
pinocchio::SE3 makeFootPose(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation)
{
    return pinocchio::SE3(rotation, translation);
}

// 函数说明：更新浮动基 q_ref 的位置和姿态部分。
void setFloatingBaseReference(Eigen::VectorXd &qRef,
                              const Eigen::Vector3d &basePosition,
                              const Eigen::Matrix3d &baseRotation)
{
    if (qRef.size() < 7)
    {
        return;
    }
    Eigen::Quaterniond quat(baseRotation);
    quat.normalize();
    qRef.head<3>() = basePosition;
    qRef(3) = quat.x();
    qRef(4) = quat.y();
    qRef(5) = quat.z();
    qRef(6) = quat.w();
}

// 函数说明：从 MuJoCo/Pinocchio 同步机器人状态到 DataBus。
void updateRobotState(MJ_Interface &mjInterface, Pin_KinDyn &kinDyn, DataBus &robotState)
{
    mjInterface.updateSensorValues();
    mjInterface.dataBusWrite(robotState);
    kinDyn.dataBusRead(robotState);
    kinDyn.computeJ_dJ();
    kinDyn.computeDyn();
    kinDyn.dataBusWrite(robotState);
}
// 函数说明：给站立/行走 demo 设置较温和的 PVT 增益。
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

// 函数说明：用 Pinocchio RNEA 计算 ID 前馈力矩。
Eigen::VectorXd inverseDynamicsFeedforward(const DataBus &robotState,
                                           const Eigen::Matrix<double, 12, 1> &wrenches,
                                           const Eigen::VectorXd &jointAccelerations,
                                           double scale,
                                           double wrenchSign)
{
    const int nv = robotState.model_nv;
    const int actuated = nv - 6;
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(actuated);
    if (robotState.dyn_M.rows() != nv || robotState.dyn_M.cols() != nv ||
        robotState.dyn_Non.size() != nv)
    {
        return tau;
    }

    Eigen::VectorXd ddq = Eigen::VectorXd::Zero(nv);
    if (jointAccelerations.size() == actuated)
    {
        ddq.tail(actuated) = jointAccelerations;
    }

    tau = (robotState.dyn_M * ddq + robotState.dyn_Non).tail(actuated);
    const double sign = wrenchSign >= 0.0 ? 1.0 : -1.0;
    if (robotState.J_l.rows() >= 6 && robotState.J_l.cols() == nv)
    {
        tau.noalias() += sign * robotState.J_l.block(0, 6, 6, actuated).transpose() * wrenches.segment<6>(0);
    }
    if (robotState.J_r.rows() >= 6 && robotState.J_r.cols() == nv)
    {
        tau.noalias() += sign * robotState.J_r.block(0, 6, 6, actuated).transpose() * wrenches.segment<6>(6);
    }

    tau *= scale;
    for (int i = 0; i < tau.size(); ++i)
    {
        const double limit = g1_kin_dyn::motorTorqueLimit(i);
        tau(i) = std::clamp(tau(i), -limit, limit);
    }
    return tau;
}

// 函数说明：按当前接触脚数量分配重力补偿 wrench。
Eigen::Matrix<double, 12, 1> nominalContactWrench(bool leftContact,
                                                  bool rightContact,
                                                  double modelMass)
{
    Eigen::Matrix<double, 12, 1> wrench = Eigen::Matrix<double, 12, 1>::Zero();
    const int contactCount = static_cast<int>(leftContact) + static_cast<int>(rightContact);
    if (contactCount <= 0)
    {
        return wrench;
    }
    const double fz = modelMass * kGravity / static_cast<double>(contactCount);
    if (leftContact)
    {
        wrench(2) = fz;
    }
    if (rightContact)
    {
        wrench(8) = fz;
    }
    return wrench;
}

// 函数说明：对 NMPC 输出 wrench 做有限性检查和摩擦/法向力/力矩限幅。
Eigen::Matrix<double, 12, 1> sanitizeKinoWrench(const Eigen::Matrix<double, 12, 1> &candidate,
                                                bool leftContact,
                                                bool rightContact,
                                                double modelMass)
{
    const Eigen::Matrix<double, 12, 1> fallback =
        nominalContactWrench(leftContact, rightContact, modelMass);
    Eigen::Matrix<double, 12, 1> out = candidate.allFinite() ? candidate : fallback;
    const double fzNom = modelMass * kGravity /
                         std::max(1, static_cast<int>(leftContact) + static_cast<int>(rightContact));

    for (int foot = 0; foot < G1KinodynamicsNmpc::kNumFeet; ++foot)
    {
        const bool active = (foot == 0) ? leftContact : rightContact;
        Eigen::Ref<Eigen::Matrix<double, 6, 1>> w = out.segment<6>(6 * foot);
        if (!active)
        {
            w.setZero();
            continue;
        }
        if (!w.allFinite() || w.z() < 0.15 * fzNom)
        {
            w = fallback.segment<6>(6 * foot);
        }
        w(2) = std::clamp(w(2), 0.35 * fzNom, 1.6 * fzNom);
        const double tangentialLimit = 0.65 * std::max(1.0, w(2));
        w(0) = std::clamp(w(0), -tangentialLimit, tangentialLimit);
        w(1) = std::clamp(w(1), -tangentialLimit, tangentialLimit);
        w(3) = std::clamp(w(3), -0.06 * w(2), 0.06 * w(2));
        w(4) = std::clamp(w(4), -0.08 * w(2), 0.08 * w(2));
        w(5) = std::clamp(w(5), -0.03 * w(2), 0.03 * w(2));
    }
    return out;
}

// 函数说明：检查并限幅 NMPC 输出的关节加速度。
Eigen::VectorXd sanitizeJointAccelerations(const Eigen::VectorXd &candidate, int size, double limit)
{
    Eigen::VectorXd out = Eigen::VectorXd::Zero(size);
    if (candidate.size() != size || !candidate.allFinite())
    {
        return out;
    }
    out = candidate;
    for (int i = 0; i < out.size(); ++i)
    {
        out(i) = std::clamp(out(i), -limit, limit);
    }
    return out;
}

// 函数说明：打印腿部总力矩、PVT 分量和 ID 前馈分量，便于调试。
void printLegTorqueBreakdown(const std::vector<double> &tauTotal,
                             const Eigen::VectorXd &tauFeedforward,
                             int start,
                             int count)
{
    const auto &names = jointNames();
    std::cout << "       joint                     total      PVT       ID-ff\n";
    for (int i = start; i < start + count && i < static_cast<int>(tauTotal.size()); ++i)
    {
        const std::string &name = (i < static_cast<int>(names.size())) ? names[i] : std::to_string(i);
        const double total = tauTotal[i];
        const double ff = vectorValueOrZero(tauFeedforward, i);
        const double pvt = total - ff;
        std::cout << "  [" << std::setw(2) << i << "] "
                  << std::left << std::setw(22) << name << std::right
                  << std::setw(10) << std::fixed << std::setprecision(3) << total
                  << std::setw(10) << pvt
                  << std::setw(10) << ff << " Nm\n";
    }
}

// 函数说明：按固定频率打印运行状态、接触模式、NMPC 耗时和 wrench。
void printRuntimeSummary(double time,
                         const DataBus &robotState,
                         const Eigen::Vector3d &comDes,
                         const Eigen::Vector3d &velDesWorld,
                         const G1KinodynamicsNmpc &nmpc,
                         const Eigen::Matrix<double, 12, 1> &wrenches,
                         const Eigen::VectorXd &jointAccelerations,
                         const Eigen::VectorXd &tauFeedforward,
                         const Eigen::Vector2d &swingPhase,
                         int stepsDone,
                         bool leftContact,
                         bool rightContact,
                         double ffScale,
                         double wrenchSign)
{
    std::cout << "\n"
              << "======== G1 Aligator Kino-NMPC + ID ALIP Walk  t="
              << std::fixed << std::setprecision(2) << time << " s ========\n";
    std::cout << "contact L/R: " << (leftContact ? 1 : 0) << " / " << (rightContact ? 1 : 0)
              << "   swing phase L/R: " << std::setprecision(3) << swingPhase.x()
              << " / " << swingPhase.y()
              << "   tSwing/tDS/tImpact: " << robotState.tSwing
              << " / " << robotState.tDoubleSupport
              << " / " << robotState.walk_time_to_impact
              << "   steps: " << stepsDone
              << "   v_des_W: [" << velDesWorld.x() << ", "
              << velDesWorld.y() << ", " << velDesWorld.z() << "]\n";
    std::cout << "CoM cur/des: [" << std::setprecision(4) << robotState.pCoM_W.x()
              << ", " << robotState.pCoM_W.y()
              << ", " << robotState.pCoM_W.z() << "] / ["
              << comDes.x() << ", "
              << comDes.y() << ", "
              << comDes.z() << "]"
              << "   base rpy: ["
              << robotState.base_rpy.x() << ", "
              << robotState.base_rpy.y() << ", "
              << robotState.base_rpy.z() << "]"
              << "   nmpc status/iter/time: " << nmpc.status()
              << " / " << nmpc.iterations()
              << " / " << nmpc.solveTime() * 1e3 << " ms\n";
    std::cout << "ID ff scale/sign: " << ffScale
              << " / " << (wrenchSign >= 0.0 ? "+1" : "-1")
              << "   joint acc norm: " << jointAccelerations.norm() << "\n";

    std::cout << "\nKino-NMPC desired wrench [Fx Fy Fz Mx My Mz]\n";
    std::cout << "  L: "
              << std::setw(9) << std::setprecision(3) << wrenches(0)
              << std::setw(9) << wrenches(1)
              << std::setw(9) << wrenches(2)
              << std::setw(9) << wrenches(3)
              << std::setw(9) << wrenches(4)
              << std::setw(9) << wrenches(5) << "\n";
    std::cout << "  R: "
              << std::setw(9) << wrenches(6)
              << std::setw(9) << wrenches(7)
              << std::setw(9) << wrenches(8)
              << std::setw(9) << wrenches(9)
              << std::setw(9) << wrenches(10)
              << std::setw(9) << wrenches(11) << "\n";

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
            options.useSquatDrop = false;
        }
        else if (arg == "--squat-drop" && i + 1 < argc)
        {
            options.squatDrop = std::max(0.0, std::stod(argv[++i]));
            options.useSquatDrop = true;
        }
        else if (arg == "--max-squat-drop" && i + 1 < argc)
        {
            options.maxSquatDrop = std::max(0.0, std::stod(argv[++i]));
        }
        else if (arg == "--squat-hold" && i + 1 < argc)
        {
            options.squatHoldTime = std::max(0.0, std::stod(argv[++i]));
        }
        else if ((arg == "--shift-y" || arg == "--prewalk-shift-y") && i + 1 < argc)
        {
            options.shiftY = std::stod(argv[++i]);
        }
        else if ((arg == "--t-swing" || arg == "--single-support") && i + 1 < argc)
        {
            options.singleSupportTime = std::max(0.12, std::stod(argv[++i]));
        }
        else if ((arg == "--t-ds" || arg == "--double-support") && i + 1 < argc)
        {
            options.doubleSupportTime = std::max(0.0, std::stod(argv[++i]));
        }
        else if (arg == "--vx" && i + 1 < argc)
        {
            options.vx = std::stod(argv[++i]);
        }
        else if (arg == "--vy" && i + 1 < argc)
        {
            options.vy = std::stod(argv[++i]);
        }
        else if (arg == "--yaw-rate" && i + 1 < argc)
        {
            options.yawRate = std::stod(argv[++i]);
        }
        else if (arg == "--velocity-ramp" && i + 1 < argc)
        {
            options.velocityRamp = std::max(1e-3, std::stod(argv[++i]));
        }
        else if (arg == "--swing-height" && i + 1 < argc)
        {
            options.swingHeight = std::max(0.0, std::stod(argv[++i]));
        }
        else if ((arg == "--steps" || arg == "--demo-steps") && i + 1 < argc)
        {
            options.demoSteps = std::max(1, std::stoi(argv[++i]));
        }
        else if (arg == "--horizon" && i + 1 < argc)
        {
            options.horizon = std::max(1, std::stoi(argv[++i]));
        }
        else if (arg == "--mpc-solve-every" && i + 1 < argc)
        {
            options.mpcSolveEveryIterations = std::max(1, std::stoi(argv[++i]));
        }
        else if (arg == "--mpc-dt" && i + 1 < argc)
        {
            options.mpcSegmentDt = std::max(0.005, std::stod(argv[++i]));
        }
        else if (arg == "--ff-scale" && i + 1 < argc)
        {
            options.ffScale = std::stod(argv[++i]);
        }
        else if (arg == "--wrench-sign" && i + 1 < argc)
        {
            options.wrenchSign = std::stod(argv[++i]) >= 0.0 ? 1.0 : -1.0;
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
        else if ((arg == "--nmpc-ls-alpha-min" || arg == "--nmpc-alpha-min") && i + 1 < argc)
        {
            options.nmpcLineSearchAlphaMin = std::max(1e-8, std::stod(argv[++i]));
        }
        else if ((arg == "--nmpc-ls-max-steps" || arg == "--nmpc-max-ls-steps") && i + 1 < argc)
        {
            options.nmpcMaxLineSearchSteps = std::max(1, std::stoi(argv[++i]));
        }
        else if ((arg == "--nmpc-threads" || arg == "--nmpc-num-threads") && i + 1 < argc)
        {
            options.nmpcThreads = std::max(1, std::stoi(argv[++i]));
        }
        else if (arg == "--nmpc-linear-rollout")
        {
            options.nmpcLinearRollout = true;
        }
        else if (arg == "--nmpc-parallel-lq")
        {
            options.nmpcParallelLq = true;
            options.nmpcLinearRollout = true;
        }
        else if (arg == "--nmpc-fast-parallel")
        {
            options.nmpcParallelLq = true;
            options.nmpcLinearRollout = true;
            options.nmpcThreads = std::max(options.nmpcThreads, 4);
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

// 函数说明：初始化 MuJoCo/Pinocchio/NMPC/ID 控制管线，并运行 ALIP 行走 demo。
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
            "../record/g1_kino_nmpc_id_alip_walk.log" :
            "record/g1_kino_nmpc_id_alip_walk.log";

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
    GaitScheduler gaitScheduler(options.singleSupportTime, simDt);
    gaitScheduler.doubleSupportTime = options.doubleSupportTime;
    gaitScheduler.stepNumDes = options.demoSteps;
    FootPlacement footPlacement;

    G1KinodynamicsNmpc nmpc(kinDyn.model_biped,
                            {kinDyn.l_foot_frame, kinDyn.r_foot_frame},
                            options.horizon,
                            options.mpcSegmentDt);
    nmpc.tolerance = options.nmpcTolerance;
    nmpc.muInit = options.nmpcMuInit;
    nmpc.maxIterations = options.nmpcMaxIterations;
    nmpc.maxAlIterations = options.nmpcMaxAlIterations;
    nmpc.lineSearchAlphaMin = options.nmpcLineSearchAlphaMin;
    nmpc.maxLineSearchSteps = options.nmpcMaxLineSearchSteps;
    nmpc.numThreads = options.nmpcThreads;
    nmpc.useLinearRollout = options.nmpcLinearRollout;
    nmpc.useParallelLq = options.nmpcParallelLq;

    DataLogger logger(logPath);
    const std::vector<double> standPoseStd = makeG1StandPose();
    const Eigen::VectorXd standPose =
        Eigen::Map<const Eigen::VectorXd>(standPoseStd.data(), standPoseStd.size());
    const double modelMass = totalModelMass(mj_model);
    const double nominalFootForce = 0.5 * modelMass * kGravity;
    const double squatEndTime = options.standTime + options.squatDuration;
    const double walkStartTime = squatEndTime + options.squatHoldTime;

    mjInterface.setMotorsPosition(standPoseStd);
    updateRobotState(mjInterface, kinDyn, robotState);
    pvtCtr.motor_pos_des_old = standPoseStd;

    const Eigen::Vector3d initialBasePos = robotState.base_pos;
    const Eigen::Vector3d initialCom = robotState.pCoM_W;
    const Eigen::Vector3d baseMinusCom = initialBasePos - initialCom;
    const Eigen::Matrix3d initialBaseRot = robotState.base_rot;
    const double initialYaw = robotState.base_rpy.z();
    const double groundZ = 0.5 * (robotState.fe_l_pos_W.z() + robotState.fe_r_pos_W.z());
    const std::array<Eigen::Matrix3d, 2> initialFootRotW = {robotState.fe_l_rot_W, robotState.fe_r_rot_W};
    const Eigen::VectorXd standHoldPose =
        Eigen::Map<const Eigen::VectorXd>(robotState.motors_pos_cur.data(), robotState.model_nv - 6);
    const Eigen::VectorXd lockedUpperBodyPose = standHoldPose;
    double requestedComZ = options.useSquatDrop ?
                               initialCom.z() - options.squatDrop :
                               options.squatTargetComZ;
    requestedComZ = std::min(requestedComZ, initialCom.z());
    if (std::isfinite(options.maxSquatDrop))
    {
        requestedComZ = std::max(requestedComZ, initialCom.z() - options.maxSquatDrop);
    }
    const double targetComZ = requestedComZ;
    const double targetComHeight = targetComZ - groundZ;
    footPlacement.kp_vx = 0.03;
    footPlacement.kp_vy = 0.035;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = options.swingHeight;
    footPlacement.legLength = targetComHeight;
    footPlacement.robotMass = modelMass;
    footPlacement.useAngularMomentumState = true;
    footPlacement.dt = simDt;
    robotState.width_hips = 0.20;
    robotState.slop.setZero();

    Eigen::Vector3d comDes = initialCom;
    Eigen::VectorXd qRef = robotState.q;
    setFloatingBaseReference(qRef, initialBasePos, initialBaseRot);
    if (qRef.size() >= 7)
    {
        const int actuatedRefSize = std::min<int>(standPose.size(), qRef.size() - 7);
        qRef.segment(7, actuatedRefSize) = standPose.head(actuatedRefSize);
    }
    const Eigen::VectorXd vRef = Eigen::VectorXd::Zero(robotState.model_nv);
    Eigen::Matrix<double, 12, 1> lastWrenches = Eigen::Matrix<double, 12, 1>::Zero();
    lastWrenches << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0,
        0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;
    Eigen::VectorXd lastJointAccelerations = Eigen::VectorXd::Zero(robotState.model_nv - 6);
    Eigen::VectorXd tauFeedforward = Eigen::VectorXd::Zero(robotState.model_nv - 6);
    Eigen::VectorXd ikStandPose = standHoldPose;
    std::array<Eigen::Vector3d, 2> footHoldW = {robotState.fe_l_pos_W, robotState.fe_r_pos_W};

    logger.addIterm("simTime", 1);
    logger.addIterm("baseRpy", 3);
    logger.addIterm("basePos", 3);
    logger.addIterm("com", 3);
    logger.addIterm("comDes", 3);
    logger.addIterm("velDesWorld", 3);
    logger.addIterm("contactLR", 2);
    logger.addIterm("swingPhase", 2);
    logger.addIterm("wrench", 12);
    logger.addIterm("jointAcc", robotState.model_nv - 6);
    logger.addIterm("tau_ff", robotState.model_nv - 6);
    logger.addIterm("nmpcStatus", 1);
    logger.addIterm("nmpcIter", 1);
    logger.addIterm("nmpcCpuTime", 1);
    logger.finishItermAdding();

    if (!options.headless)
    {
        uiController.iniGLFW();
        uiController.enableTracking();
        uiController.createWindow("G1 Aligator Kino-NMPC + ID ALIP Walk", false);
    }

    int iterationCounter = 0;
    bool walkingStarted = false;
    bool gaitStarted = false;
    bool hasNmpcSolution = false;
    bool hasPreviousContactMode = false;
    std::array<bool, G1KinodynamicsNmpc::kNumFeet> previousContactMode = {{true, true}};
    double nextPrintTime = 0.0;
    double yawDes = initialYaw;
    Eigen::Vector3d worldComDesired = initialCom;

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
            Eigen::Vector3d preWalkComDes = initialCom;
            preWalkComDes.z() = initialCom.z() + (targetComZ - initialCom.z()) * squatPhase;
            const double shiftPhase =
                smoothStep((time - squatEndTime) / std::max(1e-3, options.squatHoldTime));
            Eigen::Vector3d firstStanceComDes = initialCom;
            firstStanceComDes.y() = initialCom.y() + options.shiftY;
            firstStanceComDes.z() = targetComZ;
            preWalkComDes.y() =
                initialCom.y() + (firstStanceComDes.y() - initialCom.y()) * shiftPhase;
            const bool walkingEnabled = time >= walkStartTime;
            bool forceMpcSolve = false;

            if (walkingEnabled && !walkingStarted)
            {
                walkingStarted = true;
                forceMpcSolve = true;
                worldComDesired = preWalkComDes;
                worldComDesired.z() = targetComZ;
                yawDes = robotState.base_rpy.z();
                footHoldW = {robotState.fe_l_pos_W, robotState.fe_r_pos_W};
                gaitScheduler.stop();
                gaitScheduler.stepNumDes = options.demoSteps;
                gaitStarted = false;
                hasNmpcSolution = false;
                nmpc.resetWarmStart();
                lastWrenches = nominalContactWrench(true, true, modelMass);
                lastJointAccelerations.setZero();
                hasPreviousContactMode = false;
            }

            Eigen::Vector3d velDesWorld = Eigen::Vector3d::Zero();
            double yawRateDes = 0.0;
            if (walkingEnabled)
            {
                const double velocityScale = smoothStep((time - walkStartTime) / options.velocityRamp);
                yawRateDes = velocityScale * options.yawRate;
                yawDes += yawRateDes * simDt;
                velDesWorld = Rz3(yawDes) *
                              Eigen::Vector3d(velocityScale * options.vx,
                                              velocityScale * options.vy,
                                              0.0);

                robotState.motionState = DataBus::Walk;
                robotState.js_vel_des = velDesWorld;
                robotState.js_omega_des << 0.0, 0.0, yawRateDes;
                robotState.desV_W = velDesWorld;
                robotState.base_rpy_des << 0.0, 0.0, yawDes;
                robotState.base_vel_des = velDesWorld;
                robotState.base_omega_des << 0.0, 0.0, yawRateDes;
                robotState.walk_target_yaw = yawDes;
                robotState.walk_target_yaw_rate = yawRateDes;
                robotState.walk_target_com_height = targetComHeight;
                if (!gaitStarted)
                {
                    gaitScheduler.start();
                    gaitStarted = true;
                }
                gaitScheduler.dataBusRead(robotState);
                gaitScheduler.step();
                gaitScheduler.dataBusWrite(robotState);

                footPlacement.dataBusRead(robotState);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(robotState);
                // 说明：保留 FootPlacement/ALIP 原始落足点，避免 demo 层限幅削弱恢复步长。

                worldComDesired.head<2>() += velDesWorld.head<2>() * simDt;
                worldComDesired.z() = targetComZ;
                comDes = worldComDesired;
            }
            else
            {
                comDes = preWalkComDes;
                worldComDesired = preWalkComDes;
                yawDes = initialYaw;
                robotState.motionState = DataBus::Stand;
                gaitScheduler.stop();
                gaitStarted = false;
                robotState.legState = DataBus::DSt;
                robotState.legStateNext = DataBus::LSt;
                robotState.walk_stance_leg = DataBus::DSt;
                robotState.walk_left_contact = true;
                robotState.walk_right_contact = true;
                robotState.walk_is_double_support = true;
                robotState.walk_phase_time = 0.0;
                robotState.walk_time_to_impact = 0.0;
                robotState.tSwing = options.singleSupportTime;
                robotState.tDoubleSupport = options.doubleSupportTime;
                robotState.phi = 0.0;
                robotState.walk_target_yaw = initialYaw;
                robotState.walk_target_yaw_rate = 0.0;
                robotState.walk_target_com_height = targetComHeight;
                robotState.desV_W.setZero();
                robotState.js_vel_des.setZero();
                robotState.js_omega_des.setZero();
                robotState.base_rpy_des << 0.0, 0.0, initialYaw;
                robotState.base_vel_des.setZero();
                robotState.base_omega_des.setZero();
            }

            std::array<Eigen::Vector3d, 2> footDesW = footHoldW;
            std::array<Eigen::Vector3d, 2> mpcFootTouchdownW = footHoldW;
            if (walkingEnabled)
            {
                if (!robotState.walk_left_contact)
                {
                    footHoldW[0] = robotState.fe_l_pos_W;
                    footHoldW[1] = robotState.fe_r_pos_W;
                    footDesW[0] = robotState.swingDesPosCur_W;
                    footDesW[1] = footHoldW[1];
                    mpcFootTouchdownW = footHoldW;
                    mpcFootTouchdownW[0] = robotState.swingDesPosFinal_W;
                }
                else if (!robotState.walk_right_contact)
                {
                    footHoldW[0] = robotState.fe_l_pos_W;
                    footHoldW[1] = robotState.fe_r_pos_W;
                    footDesW[0] = footHoldW[0];
                    footDesW[1] = robotState.swingDesPosCur_W;
                    mpcFootTouchdownW = footHoldW;
                    mpcFootTouchdownW[1] = robotState.swingDesPosFinal_W;
                }
                else
                {
                    footHoldW[0] = robotState.fe_l_pos_W;
                    footHoldW[1] = robotState.fe_r_pos_W;
                    footDesW = footHoldW;
                    mpcFootTouchdownW = footHoldW;
                }
            }
            else
            {
                footHoldW[0] = robotState.fe_l_pos_W;
                footHoldW[1] = robotState.fe_r_pos_W;
                footDesW = footHoldW;
                mpcFootTouchdownW = footHoldW;
            }

            const bool leftContact = robotState.walk_left_contact;
            const bool rightContact = robotState.walk_right_contact;
            const Eigen::Vector2d swingPhase = swingPhaseFromRobotState(robotState);
            const Eigen::Matrix<int, Eigen::Dynamic, G1KinodynamicsNmpc::kNumFeet> contactTable =
                previewContactTable(robotState, options.horizon, options.mpcSegmentDt);
            const bool contactModeChanged =
                hasPreviousContactMode &&
                (previousContactMode[0] != leftContact || previousContactMode[1] != rightContact);
            if (contactModeChanged)
            {
                forceMpcSolve = true;
                hasNmpcSolution = false;
                nmpc.resetWarmStart();
                lastWrenches = nominalContactWrench(leftContact, rightContact, modelMass);
                lastJointAccelerations.setZero();
                std::cout << "[ALIP-WALK] contact switch t=" << std::fixed << std::setprecision(3)
                          << time << " L/R=" << (leftContact ? 1 : 0) << "/"
                          << (rightContact ? 1 : 0)
                          << " phi=" << std::setprecision(2) << robotState.phi << "\n";
            }
            previousContactMode = {{leftContact, rightContact}};
            hasPreviousContactMode = true;

            Eigen::Vector3d baseRef = initialBasePos + (comDes - initialCom);
            baseRef.z() = comDes.z() + baseMinusCom.z();
            robotState.base_pos_des = baseRef;

            const Eigen::Matrix3d yawDelta = Rz3(yawDes - initialYaw);
            const Eigen::Matrix3d baseRotRef = yawDelta * initialBaseRot;
            setFloatingBaseReference(qRef, baseRef, baseRotRef);

            const Eigen::Matrix3d leftFootRotWDes = yawDelta * initialFootRotW[0];
            const Eigen::Matrix3d rightFootRotWDes = yawDelta * initialFootRotW[1];
            const std::array<Eigen::Matrix3d, G1KinodynamicsNmpc::kNumFeet> footRotationsWDes = {
                leftFootRotWDes, rightFootRotWDes};
            Eigen::Vector3d basePosForIk = walkingEnabled ? robotState.base_pos : baseRef;
            const double comZCorrection =
                std::clamp(comDes.z() - robotState.pCoM_W.z(), -0.04, 0.04);
            basePosForIk.z() = baseRef.z() + comZCorrection;
            const Eigen::Vector3d leftFootPosLDes = baseRotRef.transpose() * (footDesW[0] - basePosForIk);
            const Eigen::Vector3d rightFootPosLDes = baseRotRef.transpose() * (footDesW[1] - basePosForIk);
            const Eigen::Matrix3d leftFootRotLDes = baseRotRef.transpose() * leftFootRotWDes;
            const Eigen::Matrix3d rightFootRotLDes = baseRotRef.transpose() * rightFootRotWDes;
            const Pin_KinDyn::IkRes ikRes =
                kinDyn.computeInK_Leg(leftFootRotLDes, leftFootPosLDes, rightFootRotLDes, rightFootPosLDes);
            if (ikRes.err.norm() < 8e-2 && ikRes.jointPosRes.size() >= kLegMotorCount)
            {
                ikStandPose.head(kLegMotorCount) = ikRes.jointPosRes.head(kLegMotorCount);
            }
            ikStandPose.tail(ikStandPose.size() - kLegMotorCount) =
                lockedUpperBodyPose.tail(lockedUpperBodyPose.size() - kLegMotorCount);
            if (qRef.size() >= 7)
            {
                const int actuatedRefSize = std::min<int>(ikStandPose.size(), qRef.size() - 7);
                qRef.segment(7, actuatedRefSize) = ikStandPose.head(actuatedRefSize);
            }

            const std::array<pinocchio::SE3, G1KinodynamicsNmpc::kNumFeet> footPoseRef = {
                makeFootPose(leftFootRotWDes, footDesW[0]),
                makeFootPose(rightFootRotWDes, footDesW[1])};
            const std::vector<std::array<pinocchio::SE3, G1KinodynamicsNmpc::kNumFeet>> footPoseRefHorizon =
                previewFootPoseReferences(robotState, footDesW, mpcFootTouchdownW, footRotationsWDes,
                                          options.horizon, options.mpcSegmentDt);

            const bool nmpcEnabled = walkingStarted;
            if (nmpcEnabled &&
                (forceMpcSolve || iterationCounter % options.mpcSolveEveryIterations == 0))
            {
                G1KinodynamicsNmpc::Input input;
                input.q = robotState.q;
                input.v = robotState.dq;
                input.qReference = qRef;
                input.vReference = vRef;
                input.footPoseReference = footPoseRef;
                input.footPoseReferenceHorizon = footPoseRefHorizon;
                input.contactActive = {{leftContact, rightContact}};
                input.contactTable = contactTable;
                const Eigen::Matrix<double, 12, 1> nominalWrench =
                    nominalContactWrench(leftContact, rightContact, modelMass);
                input.wrenchReference[0] = nominalWrench.segment<6>(0);
                input.wrenchReference[1] = nominalWrench.segment<6>(6);

                if (nmpc.solve(input))
                {
                    lastWrenches = sanitizeKinoWrench(nmpc.firstWrenches(),
                                                      leftContact, rightContact, modelMass);
                    lastJointAccelerations = sanitizeJointAccelerations(
                        nmpc.firstJointAccelerations(),
                        robotState.model_nv - 6,
                        std::min(10.0, nmpc.jointAccelerationLimit));
                    hasNmpcSolution = true;
                }
                else
                {
                    hasNmpcSolution = false;
                    lastWrenches = nominalContactWrench(leftContact, rightContact, modelMass);
                    lastJointAccelerations.setZero();
                }
            }

            if (!hasNmpcSolution)
            {
                lastWrenches = nominalContactWrench(leftContact, rightContact, modelMass);
                lastJointAccelerations.setZero();
            }
            lastWrenches = sanitizeKinoWrench(lastWrenches, leftContact, rightContact, modelMass);
            const double ffRamp = (kFeedforwardRampTime > 1e-9) ?
                                      std::clamp(time / kFeedforwardRampTime, 0.0, 1.0) :
                                      1.0;
            tauFeedforward = inverseDynamicsFeedforward(robotState, lastWrenches, lastJointAccelerations,
                                                        ffRamp * options.ffScale, options.wrenchSign);

            const double ikRamp = std::clamp(time / kIkRampTime, 0.0, 1.0);
            const Eigen::VectorXd motorPosDes = (1.0 - ikRamp) * standHoldPose + ikRamp * ikStandPose;
            robotState.motors_pos_des = eigen2std(motorPosDes);
            robotState.motors_vel_des.assign(robotState.model_nv - 6, 0.0);
            robotState.motors_tor_des = eigen2std(tauFeedforward);

            pvtCtr.dataBusRead(robotState);
            pvtCtr.calMotorsPVT(0.02);
            pvtCtr.dataBusWrite(robotState);
            mjInterface.setMotorsTorque(robotState.motors_tor_out);

            if (options.print && time + 1e-12 >= nextPrintTime)
            {
                printRuntimeSummary(time, robotState, comDes, velDesWorld, nmpc,
                                    lastWrenches, lastJointAccelerations, tauFeedforward,
                                    swingPhase, gaitScheduler.stepNumCur,
                                    leftContact, rightContact,
                                    options.ffScale, options.wrenchSign);
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
            logger.recItermData("velDesWorld", velDesWorld);
            logger.recItermData("contactLR", (Eigen::Vector2d() << static_cast<double>(leftContact),
                                               static_cast<double>(rightContact))
                                                  .finished());
            logger.recItermData("swingPhase", swingPhase);
            logger.recItermData("wrench", lastWrenches);
            logger.recItermData("jointAcc", lastJointAccelerations);
            logger.recItermData("tau_ff", tauFeedforward);
            logger.recItermData("nmpcStatus", static_cast<double>(nmpc.status()));
            logger.recItermData("nmpcIter", static_cast<double>(nmpc.iterations()));
            logger.recItermData("nmpcCpuTime", nmpc.solveTime());
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

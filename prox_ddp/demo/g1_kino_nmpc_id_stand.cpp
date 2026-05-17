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
#include "../g1_kinodynamics_nmpc.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"

namespace
{
constexpr int kTotalMotorDof = 29;
constexpr int kLegMotorCount = 12;
constexpr int kDefaultHorizon = 12;
constexpr int kDefaultMpcSolveEveryIterations = 25;
constexpr double kDefaultMpcSegmentDt = 0.02;
constexpr double kGravity = 9.80665;
constexpr double kFeedforwardRampTime = 0.0;
constexpr double kDefaultPrintHz = 1.0;
constexpr double kPreWalkStanceShiftY = 0.035;
constexpr double kIkRampTime = 0.5;

struct RuntimeOptions
{
    bool headless{false};
    bool print{false};
    double duration{8.0};
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
    double ffScale{1.0};
    double wrenchSign{-1.0};
    double nmpcTolerance{1e-5};
    double nmpcMuInit{1e-8};
    int nmpcMaxIterations{1};
    int nmpcMaxAlIterations{2};
    double nmpcLineSearchAlphaMin{1e-3};
    int nmpcMaxLineSearchSteps{4};
    int nmpcThreads{1};
    bool nmpcLinearRollout{false};
    bool nmpcParallelLq{false};
    double printHz{kDefaultPrintHz};
};

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

// 函数说明：三次 smooth step，用于下蹲和偏移平滑。
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
// 函数说明：给站立 demo 设置较温和的 PVT 增益。
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

// 函数说明：按固定频率打印站立 demo 状态、NMPC 耗时和 wrench。
void printRuntimeSummary(double time,
                         const DataBus &robotState,
                         const Eigen::Vector3d &comDes,
                         const G1KinodynamicsNmpc &nmpc,
                         const Eigen::Matrix<double, 12, 1> &wrenches,
                         const Eigen::VectorXd &jointAccelerations,
                         const Eigen::VectorXd &tauFeedforward,
                         double ffScale,
                         double wrenchSign)
{
    std::cout << "\n"
              << "======== G1 Aligator Kino-NMPC + ID Stand  t="
              << std::fixed << std::setprecision(2) << time << " s ========\n";
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

// 函数说明：初始化 MuJoCo/Pinocchio/NMPC/ID 控制管线，并运行下蹲偏移站立 demo。
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
            "../record/g1_kino_nmpc_id_stand.log" :
            "record/g1_kino_nmpc_id_stand.log";

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
    const std::array<Eigen::Vector3d, 2> footHoldW = {robotState.fe_l_pos_W, robotState.fe_r_pos_W};
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

    Eigen::Vector3d comDes = initialCom;
    Eigen::VectorXd qRef = robotState.q;
    setFloatingBaseReference(qRef, initialBasePos, initialBaseRot);
    if (qRef.size() >= 7)
    {
        const int actuatedRefSize = std::min<int>(standPose.size(), qRef.size() - 7);
        qRef.segment(7, actuatedRefSize) = standPose.head(actuatedRefSize);
    }
    const Eigen::VectorXd vRef = Eigen::VectorXd::Zero(robotState.model_nv);
    const std::array<pinocchio::SE3, G1KinodynamicsNmpc::kNumFeet> footPoseRef = {
        makeFootPose(robotState.fe_l_rot_W, robotState.fe_l_pos_W),
        makeFootPose(robotState.fe_r_rot_W, robotState.fe_r_pos_W)};

    Eigen::Matrix<double, 12, 1> lastWrenches = Eigen::Matrix<double, 12, 1>::Zero();
    lastWrenches << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0,
        0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;
    Eigen::VectorXd lastJointAccelerations = Eigen::VectorXd::Zero(robotState.model_nv - 6);
    Eigen::VectorXd tauFeedforward = Eigen::VectorXd::Zero(robotState.model_nv - 6);
    Eigen::VectorXd ikStandPose = standHoldPose;

    logger.addIterm("simTime", 1);
    logger.addIterm("baseRpy", 3);
    logger.addIterm("basePos", 3);
    logger.addIterm("com", 3);
    logger.addIterm("comDes", 3);
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
        uiController.createWindow("G1 Aligator Kino-NMPC + ID Stand", false);
    }

    int iterationCounter = 0;
    bool hasNmpcSolution = false;
    double nextPrintTime = 0.0;

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
            comDes = preWalkComDes;

            Eigen::Vector3d baseRef = initialBasePos + (comDes - initialCom);
            baseRef.z() = comDes.z() + baseMinusCom.z();
            setFloatingBaseReference(qRef, baseRef, initialBaseRot);

            const Eigen::Matrix3d yawDelta = Eigen::Matrix3d::Identity();
            const Eigen::Matrix3d leftFootRotWDes = yawDelta * initialFootRotW[0];
            const Eigen::Matrix3d rightFootRotWDes = yawDelta * initialFootRotW[1];
            Eigen::Vector3d basePosForIk = baseRef;
            const double comZCorrection =
                std::clamp(comDes.z() - robotState.pCoM_W.z(), -0.04, 0.04);
            basePosForIk.z() += comZCorrection;
            const Eigen::Matrix3d baseRotForIk = yawDelta * initialBaseRot;
            const Eigen::Vector3d leftFootPosLDes = baseRotForIk.transpose() * (footHoldW[0] - basePosForIk);
            const Eigen::Vector3d rightFootPosLDes = baseRotForIk.transpose() * (footHoldW[1] - basePosForIk);
            const Eigen::Matrix3d leftFootRotLDes = baseRotForIk.transpose() * leftFootRotWDes;
            const Eigen::Matrix3d rightFootRotLDes = baseRotForIk.transpose() * rightFootRotWDes;
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

            robotState.motionState = DataBus::Stand;
            robotState.legState = DataBus::DSt;
            robotState.walk_stance_leg = DataBus::DSt;
            robotState.walk_left_contact = true;
            robotState.walk_right_contact = true;
            robotState.walk_is_double_support = true;
            robotState.walk_phase_time = 0.0;
            robotState.walk_time_to_impact = 0.0;
            robotState.phi = 0.0;
            robotState.walk_target_yaw = initialYaw;
            robotState.walk_target_yaw_rate = 0.0;
            robotState.walk_target_com_height = targetComHeight;
            robotState.base_pos_des = baseRef;
            robotState.base_rpy_des << 0.0, 0.0, initialYaw;
            robotState.base_vel_des.setZero();
            robotState.base_omega_des.setZero();
            robotState.desV_W.setZero();
            robotState.js_vel_des.setZero();
            robotState.js_omega_des.setZero();

            if (iterationCounter % options.mpcSolveEveryIterations == 0)
            {
                G1KinodynamicsNmpc::Input input;
                input.q = robotState.q;
                input.v = robotState.dq;
                input.qReference = qRef;
                input.vReference = vRef;
                input.footPoseReference = footPoseRef;
                input.contactActive = {{true, true}};
                input.wrenchReference[0] << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;
                input.wrenchReference[1] << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;

                if (nmpc.solve(input))
                {
                    lastWrenches = nmpc.firstWrenches();
                    lastJointAccelerations = nmpc.firstJointAccelerations();
                    hasNmpcSolution = true;
                }
            }

            if (!hasNmpcSolution)
            {
                lastJointAccelerations.setZero();
            }
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
                printRuntimeSummary(time, robotState, comDes, nmpc, lastWrenches, lastJointAccelerations,
                                    tauFeedforward, options.ffScale, options.wrenchSign);
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

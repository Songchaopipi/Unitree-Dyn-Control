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
#include "foot_placement.h"
#include "gait_scheduler.h"
#include "g1_centroidal_nmpc.h"
#include "joystick_interpreter.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"
#include "wbc_bruce_weighted.h"
#include "wbc_priority.h"

namespace
{
constexpr int kTotalMotorDof = 29;
constexpr int kHorizon = 10;
constexpr int kMpcSolveEveryIterations = 5;
constexpr double kMpcDtDefault = 0.04;
constexpr double kSingleSupportDefault = 0.36;
constexpr double kDoubleSupportDefault = 0.04;
constexpr double kGravity = 9.80665;
constexpr double kPreWalkStanceShiftY = 0.04;
constexpr double kDefaultPrintHz = 1.0;
constexpr double kDoubleSupportTangentialLimit = 12.0;
constexpr double kSingleSupportTangentialLimit = 8.0;
constexpr double kDoubleSupportRollMomentLimit = 0.8;
constexpr double kDoubleSupportPitchMomentLimit = 2.0;
constexpr double kDoubleSupportYawMomentLimit = 0.5;
constexpr double kSingleSupportRollMomentLimit = 0.5;
constexpr double kSingleSupportPitchMomentLimit = 1.0;
constexpr double kSingleSupportYawMomentLimit = 0.3;
constexpr double kDemoStepXMin = -0.03;
constexpr double kDemoStepXMax = 0.05;
constexpr double kDemoStepWidthMin = 0.20;
constexpr double kDemoStepWidthMax = 0.24;
constexpr double kDemoTouchdownDrop = 0.005;
constexpr int kDefaultDemoSteps = 2;

struct RuntimeOptions
{
    bool headless{false};
    bool print{false};
    bool recordVideo{false};
    bool nmpcLog{false};
    bool nmpcSolverLog{false};
    double duration{10.0};
    double standTime{1.0};
    double squatDuration{2.0};
    double shiftDuration{1.0};
    double squatTargetComZ{0.65};
    double vx{0.08};
    double vy{0.0};
    double yawRate{0.0};
    double velocityRamp{1.0};
    double swingHeight{0.10};
    double singleSupportTime{kSingleSupportDefault};
    double doubleSupportTime{kDoubleSupportDefault};
    double mpcDt{kMpcDtDefault};
    double ffScale{1.0};
    double printHz{kDefaultPrintHz};
    int demoSteps{kDefaultDemoSteps};
};

DataBus::LegState oppositeStance(DataBus::LegState stance)
{
    return stance == DataBus::LSt ? DataBus::RSt : DataBus::LSt;
}

struct GaitSample
{
    bool leftContact{true};
    bool rightContact{true};
    bool doubleSupport{true};
    DataBus::LegState stanceLeg{DataBus::RSt};
    double phi{0.0};
};

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
        stanceLeg = DataBus::RSt;
    }

    double phaseElapsed = doubleSupport ?
                              std::clamp(robotState.walk_phase_time, 0.0, doubleSupportTime) :
                              std::clamp(robotState.phi, 0.0, 1.0) * singleSupportTime;
    double remainingTime = std::max(0.0, previewTime);

    auto advance = [&]()
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
        const double duration = doubleSupport ? doubleSupportTime : singleSupportTime;
        const double phaseRemaining = std::max(0.0, duration - phaseElapsed);
        if (remainingTime <= phaseRemaining + 1e-12)
        {
            phaseElapsed += remainingTime;
            break;
        }
        remainingTime -= phaseRemaining;
        advance();
    }

    out.doubleSupport = doubleSupport;
    out.stanceLeg = stanceLeg;
    out.leftContact = doubleSupport || stanceLeg == DataBus::LSt;
    out.rightContact = doubleSupport || stanceLeg == DataBus::RSt;
    out.phi = doubleSupport ? 0.0 : std::clamp(phaseElapsed / singleSupportTime, 0.0, 1.0);
    return out;
}

Eigen::Matrix<int, Eigen::Dynamic, 2> previewContactTable(const DataBus &robotState,
                                                          double segmentDt,
                                                          bool holdDoubleSupport)
{
    Eigen::Matrix<int, Eigen::Dynamic, 2> table(kHorizon, 2);
    if (robotState.motionState != DataBus::Walk || holdDoubleSupport)
    {
        table.setOnes();
        return table;
    }

    for (int i = 0; i < kHorizon; ++i)
    {
        const GaitSample sample = previewGaitSample(robotState, static_cast<double>(i) * segmentDt);
        table(i, 0) = sample.leftContact ? 1 : 0;
        table(i, 1) = sample.rightContact ? 1 : 0;
    }
    return table;
}

std::vector<std::array<Eigen::Vector3d, 2>>
previewContactPositions(const DataBus &robotState,
                        const std::array<Eigen::Vector3d, 2> &currentFeet,
                        const std::array<Eigen::Vector3d, 2> &nextFeet,
                        double segmentDt,
                        bool holdDoubleSupport)
{
    std::vector<std::array<Eigen::Vector3d, 2>> positions(kHorizon, currentFeet);
    if (holdDoubleSupport)
    {
        return positions;
    }
    for (int k = 0; k < kHorizon; ++k)
    {
        const GaitSample sample = previewGaitSample(robotState, static_cast<double>(k) * segmentDt);
        positions[k] = currentFeet;
        if (sample.leftContact && !robotState.walk_left_contact)
        {
            positions[k][0] = nextFeet[0];
        }
        if (sample.rightContact && !robotState.walk_right_contact)
        {
            positions[k][1] = nextFeet[1];
        }
    }
    return positions;
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

std::string outputPathNearRecordScript(const std::string &fileName)
{
    return firstExistingPath("../record/matlabReadDataScript.txt", "record/matlabReadDataScript.txt") ==
                   "../record/matlabReadDataScript.txt" ?
               "../record/" + fileName :
               "record/" + fileName;
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

void updateRobotState(MJ_Interface &mjInterface, Pin_KinDyn &kinDyn, DataBus &robotState)
{
    mjInterface.updateSensorValues();
    mjInterface.dataBusWrite(robotState);
    kinDyn.dataBusRead(robotState);
    kinDyn.computeJ_dJ();
    kinDyn.computeDyn();
    kinDyn.dataBusWrite(robotState);
}

void setNominalWrench(DataBus &robotState,
                      bool leftContact,
                      bool rightContact,
                      double nominalFootForce)
{
    robotState.Fr_ff.setZero(12);
    if (leftContact)
    {
        robotState.Fr_ff.segment<6>(0) << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;
    }
    if (rightContact)
    {
        robotState.Fr_ff.segment<6>(6) << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;
    }
}

Eigen::Matrix<double, 12, 1> nominalWrench(bool leftContact,
                                           bool rightContact,
                                           double mass)
{
    Eigen::Matrix<double, 12, 1> wrench = Eigen::Matrix<double, 12, 1>::Zero();
    const int contacts = static_cast<int>(leftContact) + static_cast<int>(rightContact);
    if (contacts == 0)
    {
        return wrench;
    }
    const double fz = mass * kGravity / static_cast<double>(contacts);
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

Eigen::Matrix<double, 12, 1> sanitizeWrench(const Eigen::Matrix<double, 12, 1> &candidate,
                                            const Eigen::Matrix<double, 12, 1> &fallback,
                                            bool leftContact,
                                            bool rightContact)
{
    Eigen::Matrix<double, 12, 1> out = candidate.allFinite() ? candidate : fallback;
    if (!leftContact)
    {
        out.segment<6>(0).setZero();
    }
    if (!rightContact)
    {
        out.segment<6>(6).setZero();
    }
    return out;
}

Eigen::Matrix<double, 12, 1> limitMpcWrenches(const Eigen::Matrix<double, 12, 1> &wrenchIn,
                                              bool leftContact,
                                              bool rightContact,
                                              double mass)
{
    const Eigen::Matrix<double, 12, 1> fallback = nominalWrench(leftContact, rightContact, mass);
    Eigen::Matrix<double, 12, 1> out =
        sanitizeWrench(wrenchIn, fallback, leftContact, rightContact);

    const int activeContacts = static_cast<int>(leftContact) + static_cast<int>(rightContact);
    if (activeContacts == 0)
    {
        return Eigen::Matrix<double, 12, 1>::Zero();
    }

    const bool doubleSupport = leftContact && rightContact;
    const double fzNom = mass * kGravity / static_cast<double>(activeContacts);
    const double tangentialLimit =
        doubleSupport ? kDoubleSupportTangentialLimit : kSingleSupportTangentialLimit;
    const double rollMomentLimit =
        doubleSupport ? kDoubleSupportRollMomentLimit : kSingleSupportRollMomentLimit;
    const double pitchMomentLimit =
        doubleSupport ? kDoubleSupportPitchMomentLimit : kSingleSupportPitchMomentLimit;
    const double yawMomentLimit =
        doubleSupport ? kDoubleSupportYawMomentLimit : kSingleSupportYawMomentLimit;
    const double fzMin = doubleSupport ? 20.0 : 0.90 * mass * kGravity;
    const double fzMax = doubleSupport ? 0.82 * mass * kGravity : 1.12 * mass * kGravity;

    for (int foot = 0; foot < 2; ++foot)
    {
        const bool active = foot == 0 ? leftContact : rightContact;
        Eigen::Matrix<double, 6, 1> wrench = out.segment<6>(6 * foot);
        if (!active)
        {
            out.segment<6>(6 * foot).setZero();
            continue;
        }
        if (!wrench.allFinite())
        {
            wrench = fallback.segment<6>(6 * foot);
        }
        wrench.x() = std::clamp(wrench.x(), -tangentialLimit, tangentialLimit);
        wrench.y() = std::clamp(wrench.y(), -tangentialLimit, tangentialLimit);
        wrench.z() = std::clamp(wrench.z(), fzMin, std::max(fzMin, fzMax));
        wrench(3) = std::clamp(wrench(3), -rollMomentLimit, rollMomentLimit);
        wrench(4) = std::clamp(wrench(4), -pitchMomentLimit, pitchMomentLimit);
        wrench(5) = std::clamp(wrench(5), -yawMomentLimit, yawMomentLimit);
        if (doubleSupport)
        {
            const double maxPerFoot = 0.82 * mass * kGravity;
            wrench.z() = std::clamp(wrench.z(), 20.0, maxPerFoot);
            if (wrench.z() < 0.25 * fzNom)
            {
                wrench = fallback.segment<6>(6 * foot);
            }
        }
        out.segment<6>(6 * foot) = wrench;
    }
    return out;
}

Eigen::Vector3d clampDemoFootstepTarget(const DataBus &robotState,
                                        const Eigen::Vector3d &targetW,
                                        bool swingLeft)
{
    const Eigen::Vector3d stanceW = swingLeft ? robotState.fe_r_pos_W : robotState.fe_l_pos_W;
    const Eigen::Matrix3d yawRot = Rz3(robotState.walk_target_yaw);
    Eigen::Vector3d rel = yawRot.transpose() * (targetW - stanceW);
    rel.x() = std::clamp(rel.x(), kDemoStepXMin, kDemoStepXMax);
    if (swingLeft)
    {
        rel.y() = std::clamp(rel.y(), kDemoStepWidthMin, kDemoStepWidthMax);
    }
    else
    {
        rel.y() = std::clamp(rel.y(), -kDemoStepWidthMax, -kDemoStepWidthMin);
    }
    Eigen::Vector3d out = stanceW + yawRot * rel;
    out.z() = stanceW.z() - kDemoTouchdownDrop;
    return out;
}

void clampDemoSwingFootPlan(DataBus &robotState)
{
    const bool swingLeft = !robotState.walk_left_contact && robotState.walk_right_contact;
    const bool swingRight = !robotState.walk_right_contact && robotState.walk_left_contact;
    if (!swingLeft && !swingRight)
    {
        return;
    }

    const Eigen::Vector3d start = robotState.swingStartPos_W;
    const Eigen::Vector3d oldTarget = robotState.swingDesPosFinal_W;
    const Eigen::Vector3d oldCurrent = robotState.swingDesPosCur_W;
    const Eigen::Vector3d target = clampDemoFootstepTarget(robotState, oldTarget, swingLeft);
    const double phi = std::clamp(robotState.phi, 0.0, 1.0);
    const double s = smoothStep(phi);
    const double ds = 6.0 * phi * (1.0 - phi) / std::max(1e-3, robotState.tSwing);
    const double d2s = (6.0 - 12.0 * phi) /
                       std::max(1e-6, robotState.tSwing * robotState.tSwing);
    const Eigen::Vector3d oldLine = (1.0 - s) * start + s * oldTarget;
    Eigen::Vector3d current = (1.0 - s) * start + s * target;
    const double clearance = std::max(0.0, oldCurrent.z() - oldLine.z());
    current.z() += clearance;

    const Eigen::Vector3d vel = ds * (target - start);
    const Eigen::Vector3d acc = d2s * (target - start);
    robotState.swingDesPosFinal_W = target;
    robotState.swingDesPosCur_W = current;
    robotState.swing_fe_pos_des_W = current;
    robotState.swing_fe_vel_des_W = vel;
    robotState.swing_fe_acc_des_W = acc;
    if (robotState.walk_yd.size() >= 7)
    {
        robotState.walk_yd.segment<3>(4) = current - robotState.stanceDesPos_W;
    }
    if (robotState.walk_dyd.size() >= 7)
    {
        robotState.walk_dyd.segment<3>(4) = vel;
    }
    if (robotState.walk_d2yd.size() >= 7)
    {
        robotState.walk_d2yd.segment<3>(4) = acc;
    }
}

G1CentroidalNmpc::Input buildWalkingMpcInput(const DataBus &robotState,
                                             const Eigen::Vector3d &comRef,
                                             const Eigen::Vector3d &velRefWorld,
                                             const Eigen::Matrix<int, Eigen::Dynamic, 2> &contactTable,
                                             const std::array<Eigen::Vector3d, 2> &currentContactPositions,
                                             const std::vector<std::array<Eigen::Vector3d, 2>> &contactPositionsHorizon,
                                             double mass,
                                             double mpcDt)
{
    G1CentroidalNmpc::Input input;
    input.mass = std::max(1.0, mass);
    input.current.setZero();
    input.current.head<3>() = robotState.pCoM_W;
    Eigen::Vector3d comVel = Eigen::Vector3d::Zero();
    if (robotState.Jcom_W.rows() == 3 && robotState.Jcom_W.cols() == robotState.dq.size())
    {
        comVel = robotState.Jcom_W * robotState.dq;
    }
    else if (robotState.dq.size() >= 3)
    {
        comVel = robotState.dq.head<3>();
    }
    input.current.segment<3>(3) = input.mass * comVel;
    if (robotState.dyn_Ag.rows() >= 6 && robotState.dyn_Ag.cols() == robotState.dq.size())
    {
        input.current.tail<3>() = (robotState.dyn_Ag * robotState.dq).tail<3>();
    }

    input.contactPositionWorld = currentContactPositions;
    input.contactPositionWorldHorizon = contactPositionsHorizon;
    input.contactTable = contactTable;
    input.contactActive = {{
        contactTable.rows() > 0 ? contactTable(0, 0) != 0 : robotState.walk_left_contact,
        contactTable.rows() > 0 ? contactTable(0, 1) != 0 : robotState.walk_right_contact}};
    input.comReference = comRef;
    input.comVelocityReference = velRefWorld;
    input.comAccelerationReference.setZero();
    input.reference.resize(G1CentroidalNmpc::kNx, kHorizon);
    for (int k = 0; k < kHorizon; ++k)
    {
        input.reference.col(k).setZero();
        input.reference.col(k).head<3>() = comRef + static_cast<double>(k + 1) * mpcDt * velRefWorld;
        input.reference.col(k).segment<3>(3) = input.mass * velRefWorld;
    }
    return input;
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
        else if (arg == "--record-video" || arg == "--save-video")
        {
            options.recordVideo = true;
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
        else if (arg == "--shift-duration" && i + 1 < argc)
        {
            options.shiftDuration = std::max(1e-3, std::stod(argv[++i]));
        }
        else if ((arg == "--squat-target" || arg == "--com-z") && i + 1 < argc)
        {
            options.squatTargetComZ = std::stod(argv[++i]);
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
        else if ((arg == "--t-swing" || arg == "--single-support") && i + 1 < argc)
        {
            options.singleSupportTime = std::max(0.15, std::stod(argv[++i]));
        }
        else if ((arg == "--t-ds" || arg == "--double-support") && i + 1 < argc)
        {
            options.doubleSupportTime = std::max(0.0, std::stod(argv[++i]));
        }
        else if (arg == "--swing-height" && i + 1 < argc)
        {
            options.swingHeight = std::stod(argv[++i]);
        }
        else if (arg == "--mpc-dt" && i + 1 < argc)
        {
            options.mpcDt = std::max(0.02, std::stod(argv[++i]));
        }
        else if (arg == "--ff-scale" && i + 1 < argc)
        {
            options.ffScale = std::stod(argv[++i]);
        }
        else if (arg == "--print-hz" && i + 1 < argc)
        {
            options.printHz = std::max(0.05, std::stod(argv[++i]));
        }
        else if ((arg == "--steps" || arg == "--demo-steps") && i + 1 < argc)
        {
            options.demoSteps = std::max(1, std::stoi(argv[++i]));
        }
    }
    return options;
}

void printSummary(double time,
                  const DataBus &robotState,
                  const Eigen::Vector3d &comRef,
                  const Eigen::Vector3d &velRef,
                  int stepsDone,
                  bool walkComplete)
{
    std::cout << "\n======== G1 CD-NMPC + ALIP + WBC Walk  t="
              << std::fixed << std::setprecision(2) << time << " s ========\n";
    std::cout << "mode: " << (robotState.motionState == DataBus::Walk ? "walk" : "stand")
              << "   contact L/R: " << static_cast<int>(robotState.walk_left_contact)
              << " / " << static_cast<int>(robotState.walk_right_contact)
              << "   DS: " << static_cast<int>(robotState.walk_is_double_support)
              << "   steps: " << stepsDone
              << "   complete: " << static_cast<int>(walkComplete)
              << "   phi: " << std::setprecision(3) << robotState.phi << "\n";
    std::cout << "CoM cur/ref: [" << std::setprecision(4) << robotState.pCoM_W.x()
              << ", " << robotState.pCoM_W.y()
              << ", " << robotState.pCoM_W.z() << "] / ["
              << comRef.x() << ", " << comRef.y() << ", " << comRef.z() << "]"
              << "   v_ref: [" << velRef.x() << ", " << velRef.y() << ", " << velRef.z() << "]\n";
    std::cout << "base rpy: [" << robotState.base_rpy.x()
              << ", " << robotState.base_rpy.y()
              << ", " << robotState.base_rpy.z()
              << "]   nmpc status/iter/ms: " << robotState.centroidal_nmpc_status
              << " / " << robotState.centroidal_nmpc_nWSR
              << " / " << 1000.0 * robotState.centroidal_nmpc_cpuTime
              << "   wbc qp: " << robotState.qp_status << "\n";
    std::cout << "ALIP swing cur/final: [" << robotState.swingDesPosCur_W.transpose()
              << "] / [" << robotState.swingDesPosFinal_W.transpose() << "]\n";
    std::cout << "CD-NMPC Fr_ff L/R [Fx Fy Fz Mx My Mz]\n";
    std::cout << "  L:"
              << std::setw(9) << std::setprecision(3) << vectorValueOrZero(robotState.Fr_ff, 0)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 1)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 2)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 3)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 4)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 5) << "\n";
    std::cout << "  R:"
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 6)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 7)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 8)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 9)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 10)
              << std::setw(9) << vectorValueOrZero(robotState.Fr_ff, 11) << "\n";
    std::cout << "WBC solved Fr L/R Fz: "
              << vectorValueOrZero(robotState.wbc_FrRes, 2)
              << " / " << vectorValueOrZero(robotState.wbc_FrRes, 8) << "\n";
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
    const std::string videoRawPath = outputPathNearRecordScript("rgbRec.out");

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
    WBC_priority wbcStand(kinDyn.model_nv, 18, 22, 1.0, mj_model->opt.timestep);
    WBC_BruceWeighted wbcWalk(kinDyn.model_nv, 1.0, mj_model->opt.timestep);
    PVT_Ctr pvtCtr(mj_model->opt.timestep, jointConfig.c_str());
    GaitScheduler gaitScheduler(options.singleSupportTime, mj_model->opt.timestep);
    gaitScheduler.doubleSupportTime = options.doubleSupportTime;
    gaitScheduler.stepNumDes = options.demoSteps;
    FootPlacement footPlacement;
    JoyStickInterpreter jsInterp(mj_model->opt.timestep);
    G1CentroidalNmpc centroidalNmpc(kHorizon, options.mpcDt);
    centroidalNmpc.logSolve = options.nmpcLog;
    centroidalNmpc.logSolverIterations = options.nmpcSolverLog;

    const std::vector<double> standPoseStd = makeG1StandPose();
    const Eigen::VectorXd standPose =
        Eigen::Map<const Eigen::VectorXd>(standPoseStd.data(), standPoseStd.size());
    mjInterface.setMotorsPosition(standPoseStd);
    updateRobotState(mjInterface, kinDyn, robotState);
    pvtCtr.motor_pos_des_old = standPoseStd;

    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(mj_model->nq);
    qIniDes.block(7, 0, kinDyn.model_nv - 6, 1) = standPose;
    wbcStand.setQini(qIniDes, robotState.q);
    wbcWalk.setQini(qIniDes, robotState.q);

    const double simDt = mj_model->opt.timestep;
    const double modelMass = totalModelMass(mj_model);
    const double nominalFootForce = 0.5 * modelMass * 9.81;
    const Eigen::Vector3d initialBasePos = robotState.base_pos;
    const Eigen::Vector3d initialCom = robotState.pCoM_W;
    const double initialYaw = robotState.base_rpy.z();
    const double initialBaseHeight = robotState.base_pos.z();
    const Eigen::Vector3d initialFootCenter =
        0.5 * (robotState.fe_l_pos_W + robotState.fe_r_pos_W);
    const double groundZ = 0.5 * (robotState.fe_l_pos_W.z() + robotState.fe_r_pos_W.z());
    const double targetComZ = std::min(options.squatTargetComZ, initialCom.z());
    const double targetComHeight = targetComZ - groundZ;
    const double squatEndTime = options.standTime + options.squatDuration;
    const double walkStartTime = squatEndTime + options.shiftDuration;

    robotState.width_hips = 0.20;
    robotState.slop.setZero();
    footPlacement.kp_vx = 0.03;
    footPlacement.kp_vy = 0.035;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = options.swingHeight;
    footPlacement.legLength = targetComHeight;
    footPlacement.robotMass = modelMass;
    footPlacement.useAngularMomentumState = true;
    footPlacement.dt = simDt;
    jsInterp.setIniPos(initialBasePos.x(), initialBasePos.y(), initialBaseHeight, initialYaw);

    if (!options.headless)
    {
        uiController.iniGLFW();
        uiController.enableTracking();
        uiController.createWindow("G1 CD-NMPC + ALIP + WBC Walk", options.recordVideo, videoRawPath.c_str());
    }

    bool gaitStarted = false;
    bool walkingStarted = false;
    bool walkComplete = false;
    bool hasMpcSolution = false;
    bool completionComRefInitialized = false;
    Eigen::Vector3d completionSupportCenter = initialFootCenter;
    double nextPrintTime = 0.0;
    double lastCommandedVx = std::numeric_limits<double>::quiet_NaN();
    double lastCommandedVy = std::numeric_limits<double>::quiet_NaN();
    double lastCommandedYawRate = std::numeric_limits<double>::quiet_NaN();
    int iterationCounter = 0;
    Eigen::Vector3d mpcComRef = initialCom;
    Eigen::Matrix<double, 12, 1> lastMpcWrench =
        nominalWrench(true, true, modelMass);

    while ((options.headless || !glfwWindowShouldClose(uiController.window)) &&
           mj_data->time < options.duration)
    {
        const mjtNum simStart = mj_data->time;
        while ((options.headless || uiController.runSim) &&
               mj_data->time - simStart < 1.0 / 60.0 &&
               mj_data->time < options.duration)
        {
            mj_step(mj_model, mj_data);
            updateRobotState(mjInterface, kinDyn, robotState);
            const double time = mj_data->time;

            robotState.des_ddq = Eigen::VectorXd::Zero(mj_model->nv);
            robotState.des_dq = Eigen::VectorXd::Zero(mj_model->nv);
            robotState.des_delta_q = Eigen::VectorXd::Zero(mj_model->nv);

            const double squatPhase = smoothStep((time - options.standTime) / options.squatDuration);
            const double shiftPhase = smoothStep((time - squatEndTime) / options.shiftDuration);
            Eigen::Vector3d preWalkComRef = initialCom;
            preWalkComRef.z() = initialCom.z() + (targetComZ - initialCom.z()) * squatPhase;
            preWalkComRef.y() = initialCom.y() + kPreWalkStanceShiftY * shiftPhase;

            const bool walkingWindow = time >= walkStartTime;
            if (walkingWindow && !walkingStarted)
            {
                walkingStarted = true;
                gaitStarted = false;
                walkComplete = false;
                hasMpcSolution = false;
                gaitScheduler.stop();
                gaitScheduler.stepNumCur = 0;
                gaitScheduler.stepNumDes = options.demoSteps;
                jsInterp.setIniPos(robotState.q(0), robotState.q(1), initialBaseHeight, robotState.base_rpy.z());
                mpcComRef = preWalkComRef;
                completionComRefInitialized = false;
                lastCommandedVx = std::numeric_limits<double>::quiet_NaN();
                lastCommandedVy = std::numeric_limits<double>::quiet_NaN();
                lastCommandedYawRate = std::numeric_limits<double>::quiet_NaN();
                wbcWalk.pCoMDesInitialized = false;
            }

            Eigen::Vector3d velRefWorld = Eigen::Vector3d::Zero();
            const bool plannerActive = walkingWindow && !walkComplete;
            if (plannerActive)
            {
                if (!std::isfinite(lastCommandedYawRate) ||
                    std::abs(lastCommandedYawRate - options.yawRate) > 1e-9)
                {
                    jsInterp.setWzDesLPara(options.yawRate, 1.0);
                    lastCommandedYawRate = options.yawRate;
                }
                if (!std::isfinite(lastCommandedVx) ||
                    std::abs(lastCommandedVx - options.vx) > 1e-9)
                {
                    jsInterp.setVxDesLPara(options.vx, options.velocityRamp);
                    lastCommandedVx = options.vx;
                }
                if (!std::isfinite(lastCommandedVy) ||
                    std::abs(lastCommandedVy - options.vy) > 1e-9)
                {
                    jsInterp.setVyDesLPara(options.vy, options.velocityRamp);
                    lastCommandedVy = options.vy;
                }
                jsInterp.step();
                jsInterp.dataBusWrite(robotState);
                velRefWorld = robotState.js_vel_des;

                robotState.motionState = DataBus::Walk;
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
                clampDemoSwingFootPlan(robotState);

                const bool completedThisCycle =
                    gaitScheduler.stepNumCur >= options.demoSteps &&
                    robotState.walk_is_double_support;
                if (completedThisCycle && !walkComplete)
                {
                    hasMpcSolution = false;
                    completionComRefInitialized = false;
                }
                walkComplete = completedThisCycle;
            }
            else if (walkingWindow && walkComplete)
            {
                robotState.motionState = DataBus::Stand;
                robotState.walk_left_contact = true;
                robotState.walk_right_contact = true;
                robotState.walk_is_double_support = true;
                robotState.legState = DataBus::DSt;
                robotState.legStateNext = DataBus::DSt;
                robotState.phi = 0.0;
                robotState.walk_phase_time = 0.0;
                robotState.walk_time_to_impact = 0.0;
                velRefWorld.setZero();
                robotState.js_vel_des.setZero();
                robotState.js_omega_des.setZero();
                robotState.walk_desired_vx = 0.0;
                robotState.walk_desired_vy = 0.0;
                robotState.walk_target_yaw_rate = 0.0;
                if (robotState.walk_yd.size() >= 7)
                {
                    robotState.walk_yd.segment<3>(4).setZero();
                }
                if (robotState.walk_dyd.size() >= 7)
                {
                    robotState.walk_dyd.segment<3>(4).setZero();
                }
                if (robotState.walk_d2yd.size() >= 7)
                {
                    robotState.walk_d2yd.segment<3>(4).setZero();
                }
            }
            else
            {
                robotState.motionState = DataBus::Stand;
                gaitScheduler.stop();
                gaitStarted = false;
                robotState.legState = DataBus::DSt;
                robotState.legStateNext = DataBus::LSt;
                robotState.walk_stance_leg = DataBus::RSt;
                robotState.phi = 0.0;
                robotState.walk_is_double_support = true;
                robotState.walk_left_contact = true;
                robotState.walk_right_contact = true;
                robotState.walk_phase_time = 0.0;
                robotState.walk_time_to_impact = 0.0;
                robotState.tSwing = options.singleSupportTime;
                robotState.tDoubleSupport = options.doubleSupportTime;
                robotState.stance_fe_pos_cur_W = 0.5 * (robotState.fe_l_pos_W + robotState.fe_r_pos_W);
                robotState.stance_fe_rot_cur_W = robotState.fe_r_rot_W;
                velRefWorld.setZero();
                mpcComRef = preWalkComRef;
            }

            if (plannerActive)
            {
                mpcComRef.head<2>() += simDt * velRefWorld.head<2>();
                Eigen::Vector2d err = mpcComRef.head<2>() - robotState.pCoM_W.head<2>();
                if (err.norm() > 0.12)
                {
                    mpcComRef.head<2>() = robotState.pCoM_W.head<2>() + err.normalized() * 0.12;
                }
            }
            else if (walkingWindow && walkComplete)
            {
                if (!completionComRefInitialized)
                {
                    mpcComRef = robotState.pCoM_W;
                    completionSupportCenter = 0.5 * (robotState.fe_l_pos_W + robotState.fe_r_pos_W);
                    completionComRefInitialized = true;
                }
                Eigen::Vector3d target = completionSupportCenter + (initialCom - initialFootCenter);
                target.z() = targetComZ;
                Eigen::Vector3d delta = target - mpcComRef;
                const double xyStep = 0.03 * simDt;
                const double zStep = 0.02 * simDt;
                delta.x() = std::clamp(delta.x(), -xyStep, xyStep);
                delta.y() = std::clamp(delta.y(), -xyStep, xyStep);
                delta.z() = std::clamp(delta.z(), -zStep, zStep);
                mpcComRef += delta;
            }
            if (walkingWindow)
            {
                mpcComRef.z() = targetComZ;
            }

            robotState.base_rpy_des << 0.0, 0.0,
                (robotState.motionState == DataBus::Walk ? robotState.walk_target_yaw : initialYaw);
            robotState.base_pos_des = initialBasePos;
            robotState.base_pos_des.z() = initialBaseHeight + mpcComRef.z() - initialCom.z();
            robotState.base_vel_des = velRefWorld;
            robotState.base_omega_des << 0.0, 0.0,
                (robotState.motionState == DataBus::Walk ? robotState.walk_target_yaw_rate : 0.0);
            robotState.walk_target_com_height = targetComHeight;
            if (robotState.walk_yd.size() >= 1)
            {
                robotState.walk_yd(0) = targetComHeight;
            }
            setNominalWrench(robotState,
                             robotState.walk_left_contact,
                             robotState.walk_right_contact,
                             nominalFootForce);

            const bool holdDoubleSupport = walkingWindow && walkComplete;
            const std::array<Eigen::Vector3d, 2> currentFeet = {
                robotState.fe_l_pos_W,
                robotState.fe_r_pos_W};
            std::array<Eigen::Vector3d, 2> mpcFeet = currentFeet;
            if (robotState.motionState == DataBus::Walk)
            {
                if (!robotState.walk_left_contact)
                {
                    mpcFeet[0] = robotState.swingDesPosFinal_W;
                }
                if (!robotState.walk_right_contact)
                {
                    mpcFeet[1] = robotState.swingDesPosFinal_W;
                }
            }
            const Eigen::Matrix<int, Eigen::Dynamic, 2> contactTable =
                previewContactTable(robotState, options.mpcDt, holdDoubleSupport);
            const std::vector<std::array<Eigen::Vector3d, 2>> contactPositionsHorizon =
                previewContactPositions(robotState, currentFeet, mpcFeet, options.mpcDt, holdDoubleSupport);

            if (robotState.motionState == DataBus::Walk)
            {
                if (iterationCounter % kMpcSolveEveryIterations == 0 || !hasMpcSolution)
                {
                    const G1CentroidalNmpc::Input input =
                        buildWalkingMpcInput(robotState,
                                             mpcComRef,
                                             velRefWorld,
                                             contactTable,
                                             currentFeet,
                                             contactPositionsHorizon,
                                             modelMass,
                                             options.mpcDt);
                    if (centroidalNmpc.solve(input))
                    {
                        const Eigen::Matrix<double, 12, 1> fallback =
                            nominalWrench(robotState.walk_left_contact, robotState.walk_right_contact, modelMass);
                        lastMpcWrench = limitMpcWrenches(centroidalNmpc.firstWrenches(),
                                                         robotState.walk_left_contact,
                                                         robotState.walk_right_contact,
                                                         modelMass);
                        lastMpcWrench = sanitizeWrench(lastMpcWrench,
                                                       fallback,
                                                       robotState.walk_left_contact,
                                                       robotState.walk_right_contact);
                        centroidalNmpc.dataBusWrite(robotState);
                        robotState.centroidal_nmpc_com_pos_des = mpcComRef;
                        robotState.centroidal_nmpc_com_vel_des = velRefWorld;
                        robotState.centroidal_nmpc_com_acc_des.setZero();
                        hasMpcSolution = true;
                    }
                }
                if (hasMpcSolution)
                {
                    const Eigen::Matrix<double, 12, 1> fallback =
                        nominalWrench(robotState.walk_left_contact, robotState.walk_right_contact, modelMass);
                    const Eigen::Matrix<double, 12, 1> blended =
                        fallback + options.ffScale * (lastMpcWrench - fallback);
                    robotState.Fr_ff = sanitizeWrench(blended,
                                                      fallback,
                                                      robotState.walk_left_contact,
                                                      robotState.walk_right_contact);
                    robotState.centroidal_nmpc_Fr_des = robotState.Fr_ff;
                }
            }
            else
            {
                robotState.centroidal_nmpc_enabled = false;
                robotState.centroidal_nmpc_status = 0;
                robotState.centroidal_nmpc_nWSR = 0;
                robotState.centroidal_nmpc_cpuTime = 0.0;
                robotState.centroidal_nmpc_com_pos_des = mpcComRef;
                robotState.centroidal_nmpc_com_vel_des.setZero();
                robotState.centroidal_nmpc_com_acc_des.setZero();
            }

            if (robotState.motionState == DataBus::Walk)
            {
                robotState.des_delta_q.block<2, 1>(0, 0) =
                    velRefWorld.head<2>() * simDt;
                robotState.des_delta_q(5) = robotState.walk_target_yaw_rate * simDt;
                robotState.des_dq.block<2, 1>(0, 0) = velRefWorld.head<2>();
                robotState.des_dq(5) = robotState.walk_target_yaw_rate;
                const double kVel = 5.0;
                robotState.des_ddq.block<2, 1>(0, 0) =
                    kVel * (velRefWorld.head<2>() - robotState.dq.head<2>());
                robotState.des_ddq(5) = kVel * (robotState.walk_target_yaw_rate - robotState.dq(5));
            }

            if (robotState.motionState == DataBus::Stand)
            {
                wbcStand.pCoMDes = mpcComRef;
                wbcStand.pCoMDesInitialized = true;
                wbcStand.dataBusRead(robotState);
                wbcStand.computeDdq(kinDyn);
                wbcStand.computeTau();
                wbcStand.dataBusWrite(robotState);
            }
            else
            {
                wbcWalk.dataBusRead(robotState);
                wbcWalk.computeDdq(kinDyn);
                wbcWalk.computeTau();
                wbcWalk.dataBusWrite(robotState);
            }

            Eigen::VectorXd posDes = kinDyn.integrateDIY(robotState.q, robotState.wbc_delta_q_final);
            robotState.motors_pos_des = eigen2std(posDes.block(7, 0, kinDyn.model_nv - 6, 1));
            robotState.motors_vel_des = eigen2std(robotState.wbc_dq_final.block(6, 0, kinDyn.model_nv - 6, 1));
            robotState.motors_tor_des = eigen2std(robotState.wbc_tauJointRes);

            std::vector<bool> activeLegMotor(std::min(12, kinDyn.model_nv - 6), false);
            if (robotState.motionState == DataBus::Walk)
            {
                for (int motorId : robotState.wbc_active_motor_ids)
                {
                    if (motorId >= 0 && motorId < static_cast<int>(activeLegMotor.size()))
                    {
                        activeLegMotor[motorId] = true;
                    }
                }
                for (int i = 0; i < static_cast<int>(activeLegMotor.size()); ++i)
                {
                    if (!activeLegMotor[i])
                    {
                        robotState.motors_pos_des[i] = robotState.motors_pos_cur[i];
                        robotState.motors_vel_des[i] = 0.0;
                        robotState.motors_tor_des[i] = 0.0;
                    }
                }
            }
            for (int i = 12; i < kinDyn.model_nv - 6 && i < static_cast<int>(standPoseStd.size()); ++i)
            {
                robotState.motors_pos_des[i] = standPoseStd[i];
                robotState.motors_vel_des[i] = 0.0;
                robotState.motors_tor_des[i] = 0.0;
            }

            pvtCtr.dataBusRead(robotState);
            pvtCtr.calMotorsPVT();
            pvtCtr.dataBusWrite(robotState);
            if (robotState.motionState == DataBus::Walk)
            {
                for (int i = 0; i < static_cast<int>(activeLegMotor.size()); ++i)
                {
                    if (!activeLegMotor[i])
                    {
                        robotState.motors_tor_out[i] = 0.0;
                        robotState.motors_tor_cur[i] = 0.0;
                    }
                }
            }
            mjInterface.setMotorsTorque(robotState.motors_tor_out);

            if (options.print && time + 1e-12 >= nextPrintTime)
            {
                printSummary(time, robotState, mpcComRef, velRefWorld,
                             gaitScheduler.stepNumCur, walkComplete);
                nextPrintTime += 1.0 / options.printHz;
                if (nextPrintTime < time)
                {
                    nextPrintTime = time + 1.0 / options.printHz;
                }
            }

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
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);
    return walkComplete ? 0 : 2;
}

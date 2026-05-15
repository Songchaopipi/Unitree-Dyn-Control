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
#include "gait_scheduler.h"
#include "g1_centroidal_nmpc.h"
#include "joystick_interpreter.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"

namespace
{
constexpr int kTotalMotorDof = 29;
constexpr int kHorizon = 10;
constexpr int kMpcSolveEveryIterations = 5;
constexpr double kDefaultMpcSegmentDt = 0.04;
constexpr double kDefaultSingleSupportTime = 0.3;
constexpr double kDefaultDoubleSupportTime = 0.0;
constexpr double kGravity = 9.80665;
constexpr double kFeedforwardRampTime = 0.5;
constexpr double kDefaultPrintHz = 1.0;
constexpr double kPreWalkStanceShiftY = 0.035;
constexpr double kDoubleSupportTangentialLimit = 10.0;
constexpr double kSingleSupportTangentialLimit = 6.0;
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
constexpr int kLegMotorCount = 12;
constexpr int kWaistRollMotor = 13;
constexpr int kWaistPitchMotor = 14;
constexpr double kWaistRollKp = 260.0;
constexpr double kWaistRollKd = 18.0;
constexpr double kWaistPitchKp = 120.0;
constexpr double kWaistPitchKd = 8.0;
constexpr double kWaistRollLimit = 12.0;
constexpr double kWaistPitchLimit = 8.0;

struct RuntimeOptions
{
    bool headless{false};
    bool print{false};
    bool recordVideo{false};
    bool nmpcLog{false};
    bool nmpcSolverLog{false};
    double duration{12.0};
    double startTime{0.0};
    double standTime{1.0};
    double squatDuration{4.0};
    double squatHoldTime{1.0};
    double squatTargetComZ{0.65};
    double mpcSegmentDt{kDefaultMpcSegmentDt};
    double singleSupportTime{kDefaultSingleSupportTime};
    double doubleSupportTime{kDefaultDoubleSupportTime};
    double vx{0.1};
    double vy{0.0};
    double yawRate{0.0};
    double velocityRamp{2.0};
    double swingHeight{0.10};
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
    DataBus::LegState stanceLeg{DataBus::LSt};
    DataBus::LegState legState{DataBus::DSt};
    double phi{0.0};
    double timeToImpact{0.0};
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

Eigen::Matrix<int, Eigen::Dynamic, 2> previewContactTable(const DataBus &robotState, double segmentDt)
{
    Eigen::Matrix<int, Eigen::Dynamic, 2> table(kHorizon, 2);
    if (robotState.motionState != DataBus::Walk)
    {
        table.setOnes();
        return table;
    }

    for (int i = 0; i < kHorizon; ++i)
    {
        const GaitSample future = previewGaitSample(robotState, static_cast<double>(i) * segmentDt);
        table(i, 0) = future.leftContact ? 1 : 0;
        table(i, 1) = future.rightContact ? 1 : 0;
    }
    return table;
}

std::vector<std::array<Eigen::Vector3d, 2>>
previewContactPositions(const DataBus &robotState,
                        const std::array<Eigen::Vector3d, 2> &currentFootPositions,
                        const std::array<Eigen::Vector3d, 2> &nextFootPositions,
                        double segmentDt)
{
    std::vector<std::array<Eigen::Vector3d, 2>> positions(kHorizon, currentFootPositions);
    for (int k = 0; k < kHorizon; ++k)
    {
        const GaitSample future = previewGaitSample(robotState, static_cast<double>(k) * segmentDt);
        positions[k] = currentFootPositions;
        if (future.leftContact && !robotState.walk_left_contact)
        {
            positions[k][0] = nextFootPositions[0];
        }
        if (future.rightContact && !robotState.walk_right_contact)
        {
            positions[k][1] = nextFootPositions[1];
        }
    }
    return positions;
}

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

std::string outputPathNearRecordScript(const std::string &fileName)
{
    return firstExistingPath("../record/matlabReadDataScript.txt", "record/matlabReadDataScript.txt") ==
                   "../record/matlabReadDataScript.txt" ?
               "../record/" + fileName :
               "record/" + fileName;
}

void writeLowerBodyCsvHeader(std::ofstream &csv)
{
    csv << "time,"
        << "left_contact,right_contact,swing_leg,stance_leg,"
        << "swing_foot_rel_com_x,swing_foot_rel_com_y,swing_foot_rel_com_z,"
        << "stance_foot_rel_com_x,stance_foot_rel_com_y,stance_foot_rel_com_z,"
        << "lower_body_inertia_about_robot_com_x,"
        << "lower_body_inertia_about_robot_com_y,"
        << "lower_body_inertia_about_robot_com_z\n";
}

void writeLowerBodyCsvRow(std::ofstream &csv,
                          double time,
                          const DataBus &robotState,
                          bool leftContact,
                          bool rightContact,
                          const Pin_KinDyn::CompositeInertia &lowerBodyInertia)
{
    constexpr int kNoLeg = -1;
    constexpr int kLeftLeg = 0;
    constexpr int kRightLeg = 1;
    const double nan = std::numeric_limits<double>::quiet_NaN();

    int swingLeg = kNoLeg;
    int stanceLeg = kNoLeg;
    Eigen::Vector3d swingRelCom = Eigen::Vector3d::Constant(nan);
    Eigen::Vector3d stanceRelCom = Eigen::Vector3d::Zero();

    if (!leftContact && rightContact)
    {
        swingLeg = kLeftLeg;
        stanceLeg = kRightLeg;
        swingRelCom = robotState.fe_l_pos_W - robotState.pCoM_W;
        stanceRelCom = robotState.fe_r_pos_W - robotState.pCoM_W;
    }
    else if (leftContact && !rightContact)
    {
        swingLeg = kRightLeg;
        stanceLeg = kLeftLeg;
        swingRelCom = robotState.fe_r_pos_W - robotState.pCoM_W;
        stanceRelCom = robotState.fe_l_pos_W - robotState.pCoM_W;
    }
    else if (robotState.walk_stance_leg == DataBus::LSt)
    {
        stanceLeg = kLeftLeg;
        stanceRelCom = robotState.fe_l_pos_W - robotState.pCoM_W;
    }
    else if (robotState.walk_stance_leg == DataBus::RSt)
    {
        stanceLeg = kRightLeg;
        stanceRelCom = robotState.fe_r_pos_W - robotState.pCoM_W;
    }
    else
    {
        stanceRelCom = 0.5 * (robotState.fe_l_pos_W + robotState.fe_r_pos_W) - robotState.pCoM_W;
    }

    csv << std::setprecision(12)
        << time << ","
        << static_cast<int>(leftContact) << ","
        << static_cast<int>(rightContact) << ","
        << swingLeg << ","
        << stanceLeg << ","
        << swingRelCom.x() << ","
        << swingRelCom.y() << ","
        << swingRelCom.z() << ","
        << stanceRelCom.x() << ","
        << stanceRelCom.y() << ","
        << stanceRelCom.z() << ","
        << lowerBodyInertia.inertiaAboutReferenceWorld(0, 0) << ","
        << lowerBodyInertia.inertiaAboutReferenceWorld(1, 1) << ","
        << lowerBodyInertia.inertiaAboutReferenceWorld(2, 2) << "\n";
}

double smoothStep(double x)
{
    const double u = std::clamp(x, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

Eigen::Matrix<double, 6, 1> planeForceReferenceFromWrench(const Eigen::Matrix<double, 6, 1> &wrench)
{
    if (!wrench.allFinite())
    {
        return Eigen::Matrix<double, 6, 1>::Zero();
    }

    const Eigen::Matrix<double, 6, 6> H = g1_kin_dyn::planeForceToWrenchMap();
    Eigen::Matrix<double, 6, 1> planeForce = H.colPivHouseholderQr().solve(wrench);
    if (!planeForce.allFinite())
    {
        return g1_kin_dyn::distributeVerticalFootLoad(std::max(0.0, wrench.z()));
    }
    return planeForce;
}

Eigen::Matrix<double, 12, 1> shapedMpcPlaneForces(const Eigen::Matrix<double, 12, 1> &wrenchIn,
                                                  bool leftContact,
                                                  bool rightContact,
                                                  double mass)
{
    Eigen::Matrix<double, 12, 1> planeForces = Eigen::Matrix<double, 12, 1>::Zero();
    const int activeContacts = static_cast<int>(leftContact) + static_cast<int>(rightContact);
    if (activeContacts == 0)
    {
        return planeForces;
    }

    const double fzNom = mass * kGravity / static_cast<double>(activeContacts);
    const bool doubleSupport = leftContact && rightContact;
    const double tangentialLimit = doubleSupport ? kDoubleSupportTangentialLimit : kSingleSupportTangentialLimit;
    const double rollMomentLimit = doubleSupport ? kDoubleSupportRollMomentLimit : kSingleSupportRollMomentLimit;
    const double pitchMomentLimit = doubleSupport ? kDoubleSupportPitchMomentLimit : kSingleSupportPitchMomentLimit;
    const double yawMomentLimit = doubleSupport ? kDoubleSupportYawMomentLimit : kSingleSupportYawMomentLimit;
    const double fzMin = doubleSupport ? 20.0 : 0.92 * mass * kGravity;
    const double fzMax = doubleSupport ? 0.80 * mass * kGravity : 1.08 * mass * kGravity;

    for (int foot = 0; foot < 2; ++foot)
    {
        const bool active = foot == 0 ? leftContact : rightContact;
        if (!active)
        {
            continue;
        }

        Eigen::Matrix<double, 6, 1> wrench = wrenchIn.segment<6>(6 * foot);
        if (!wrench.allFinite())
        {
            wrench.setZero();
            wrench.z() = fzNom;
        }
        wrench.x() = std::clamp(wrench.x(), -tangentialLimit, tangentialLimit);
        wrench.y() = std::clamp(wrench.y(), -tangentialLimit, tangentialLimit);
        wrench.z() = std::clamp(wrench.z(), fzMin, fzMax);
        wrench(3) = std::clamp(wrench(3), -rollMomentLimit, rollMomentLimit);
        wrench(4) = std::clamp(wrench(4), -pitchMomentLimit, pitchMomentLimit);
        wrench(5) = std::clamp(wrench(5), -yawMomentLimit, yawMomentLimit);

        planeForces.segment<6>(6 * foot) = planeForceReferenceFromWrench(wrench);
    }

    return planeForces;
}

Eigen::Matrix<double, 12, 1> wrenchesFromPlaneForces(const Eigen::Matrix<double, 12, 1> &planeForces)
{
    Eigen::Matrix<double, 12, 1> wrenches = Eigen::Matrix<double, 12, 1>::Zero();
    const Eigen::Matrix<double, 6, 6> H = g1_kin_dyn::planeForceToWrenchMap();
    wrenches.segment<6>(0) = H * planeForces.segment<6>(0);
    wrenches.segment<6>(6) = H * planeForces.segment<6>(6);
    return wrenches;
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
    Eigen::Matrix<double, 12, 1> out = sanitizeWrench(wrenchIn, fallback, leftContact, rightContact);
    const int activeContacts = static_cast<int>(leftContact) + static_cast<int>(rightContact);
    if (activeContacts == 0)
    {
        return Eigen::Matrix<double, 12, 1>::Zero();
    }

    const bool doubleSupport = leftContact && rightContact;
    const double fzNom = mass * kGravity / static_cast<double>(activeContacts);
    const double tangentialLimit = doubleSupport ? kDoubleSupportTangentialLimit : kSingleSupportTangentialLimit;
    const double rollMomentLimit = doubleSupport ? kDoubleSupportRollMomentLimit : kSingleSupportRollMomentLimit;
    const double pitchMomentLimit = doubleSupport ? kDoubleSupportPitchMomentLimit : kSingleSupportPitchMomentLimit;
    const double yawMomentLimit = doubleSupport ? kDoubleSupportYawMomentLimit : kSingleSupportYawMomentLimit;
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
        if (doubleSupport && wrench.z() < 0.25 * fzNom)
        {
            wrench = fallback.segment<6>(6 * foot);
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

Eigen::Quaterniond quatFromDataBus(const DataBus &robotState)
{
    if (robotState.q.size() < 7)
    {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(robotState.q(6), robotState.q(3), robotState.q(4), robotState.q(5));
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

void tuneWalkingPvtGains(PVT_Ctr &pvtCtr)
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

Eigen::VectorXd contactTorqueFeedforward(const DataBus &robotState,
                                         const Eigen::Matrix<double, 12, 1> &planeForces,
                                         const Eigen::Vector2d &waistTorqueRp,
                                         double scale)
{
    Eigen::VectorXd tauMotor = Eigen::VectorXd::Zero(robotState.model_nv - 6);
    if (robotState.J_l_foot.rows() >= 6 && robotState.J_l_foot.cols() == robotState.model_nv)
    {
        tauMotor.head(kLegMotorCount / 2).noalias() -=
            robotState.J_l_foot.block(0, 6, 6, kLegMotorCount / 2).transpose() *
            planeForces.segment<6>(0);
    }
    if (robotState.J_r_foot.rows() >= 6 && robotState.J_r_foot.cols() == robotState.model_nv)
    {
        tauMotor.segment(kLegMotorCount / 2, kLegMotorCount / 2).noalias() -=
            robotState.J_r_foot.block(0, 12, 6, kLegMotorCount / 2).transpose() *
            planeForces.segment<6>(6);
    }

    tauMotor *= scale;
    if (tauMotor.size() > kWaistRollMotor)
    {
        tauMotor(kWaistRollMotor) += scale * waistTorqueRp.x();
    }
    if (tauMotor.size() > kWaistPitchMotor)
    {
        tauMotor(kWaistPitchMotor) += scale * waistTorqueRp.y();
    }
    for (int i = 0; i < tauMotor.size(); ++i)
    {
        const double limit = g1_kin_dyn::motorTorqueLimit(i);
        tauMotor(i) = std::clamp(tauMotor(i), -limit, limit);
    }
    return tauMotor;
}

Eigen::Vector2d waistRpStabilizingTorque(const DataBus &robotState)
{
    Eigen::Vector2d tau;
    tau.x() = -kWaistRollKp * robotState.base_rpy.x() - kWaistRollKd * robotState.base_omega_W.x();
    tau.y() = -kWaistPitchKp * robotState.base_rpy.y() - kWaistPitchKd * robotState.base_omega_W.y();
    tau.x() = std::clamp(tau.x(), -kWaistRollLimit, kWaistRollLimit);
    tau.y() = std::clamp(tau.y(), -kWaistPitchLimit, kWaistPitchLimit);
    return tau;
}

G1CentroidalNmpc::Input buildWalkingMpcInput(const DataBus &robotState,
                                      const Eigen::Vector3d &comDes,
                                      const Eigen::Vector3d &velDesWorld,
                                      double yawDes,
                                      double yawRateDes,
                                      const Eigen::Matrix<int, Eigen::Dynamic, 2> &contactTable,
                                      const std::array<Eigen::Vector3d, 2> &mpcContactPositions,
                                      const std::vector<std::array<Eigen::Vector3d, 2>> &mpcContactPositionsHorizon,
                                      double mass,
                                      double mpcSegmentDt)
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
    input.contactPositionWorld = mpcContactPositions;
    input.contactPositionWorldHorizon = mpcContactPositionsHorizon;
    input.reference.resize(G1CentroidalNmpc::kNx, kHorizon);
    input.contactTable = contactTable;
    input.contactActive = {{
        contactTable.rows() > 0 ? contactTable(0, 0) != 0 : robotState.walk_left_contact,
        contactTable.rows() > 0 ? contactTable(0, 1) != 0 : robotState.walk_right_contact}};
    input.comReference = comDes;
    input.comVelocityReference = velDesWorld;
    input.comAccelerationReference.setZero();

    for (int k = 0; k < kHorizon; ++k)
    {
        const Eigen::Vector3d pRef = comDes + static_cast<double>(k + 1) * mpcSegmentDt * velDesWorld;
        input.reference.col(k).setZero();
        input.reference.col(k).head<3>() = pRef;
        input.reference.col(k).segment<3>(3) = input.mass * velDesWorld;
    }
    return input;
}

void fillWalkBus(DataBus &robotState,
                 const Eigen::Vector3d &comDes,
                 const Eigen::Vector3d &velDesWorld,
                 double yawDes,
                 double yawRateDes,
                 bool leftContact,
                 bool rightContact,
                 double nominalFootForce,
                 bool walkingMode,
                 double targetComHeight)
{
    robotState.motionState = walkingMode ? DataBus::Walk : DataBus::Stand;
    robotState.walk_left_contact = leftContact;
    robotState.walk_right_contact = rightContact;
    robotState.walk_is_double_support = leftContact && rightContact;
    if (leftContact && !rightContact)
    {
        robotState.legState = DataBus::LSt;
        robotState.walk_stance_leg = DataBus::LSt;
        robotState.stance_fe_pos_cur_W = robotState.fe_l_pos_W;
        robotState.stance_fe_rot_cur_W = robotState.fe_l_rot_W;
    }
    else if (rightContact && !leftContact)
    {
        robotState.legState = DataBus::RSt;
        robotState.walk_stance_leg = DataBus::RSt;
        robotState.stance_fe_pos_cur_W = robotState.fe_r_pos_W;
        robotState.stance_fe_rot_cur_W = robotState.fe_r_rot_W;
    }
    else
    {
        robotState.legState = DataBus::DSt;
        if (!walkingMode)
        {
            robotState.walk_stance_leg = DataBus::DSt;
            robotState.stance_fe_pos_cur_W = 0.5 * (robotState.fe_l_pos_W + robotState.fe_r_pos_W);
            robotState.stance_fe_rot_cur_W = robotState.fe_l_rot_W;
        }
        else if (robotState.walk_stance_leg == DataBus::LSt)
        {
            robotState.stance_fe_pos_cur_W = robotState.fe_l_pos_W;
            robotState.stance_fe_rot_cur_W = robotState.fe_l_rot_W;
        }
        else
        {
            robotState.stance_fe_pos_cur_W = robotState.fe_r_pos_W;
            robotState.stance_fe_rot_cur_W = robotState.fe_r_rot_W;
        }
    }
    robotState.walk_target_yaw = yawDes;
    robotState.walk_target_yaw_rate = yawRateDes;
    robotState.walk_target_com_height = targetComHeight;
    robotState.desV_W = velDesWorld;
    robotState.js_vel_des = velDesWorld;
    robotState.js_omega_des << 0.0, 0.0, yawRateDes;
    robotState.base_rpy_des << 0.0, 0.0, yawDes;
    robotState.base_vel_des = velDesWorld;
    robotState.base_omega_des << 0.0, 0.0, yawRateDes;
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
                         const Eigen::Vector3d &velDesWorld,
                         const std::array<Eigen::Vector3d, 2> &footDesW,
                         const std::array<Eigen::Vector3d, 2> &mpcContactPositions,
                         const Eigen::Vector2d &waistTorqueRp,
                         const Pin_KinDyn::CompositeInertia &lowerBodyInertia,
                         const Eigen::Vector2d &swingPhase,
                         int stepsDone,
                         bool leftContact,
                         bool rightContact,
                         const Eigen::VectorXd &tauFeedforward)
{
    std::cout << "\n"
              << "======== G1 Aligator CD-NMPC + IK Walk  t="
              << std::fixed << std::setprecision(2) << time << " s ========\n";
    std::cout << "contact L/R: " << (leftContact ? 1 : 0) << " / " << (rightContact ? 1 : 0)
              << "   swing phase L/R: " << std::setprecision(3) << swingPhase.x()
              << " / " << swingPhase.y()
              << "   tSwing/tDS/tImpact: " << robotState.tSwing
              << " / " << robotState.tDoubleSupport
              << " / " << robotState.walk_time_to_impact
              << "   steps: " << stepsDone
              << "   v_des_W: [" << velDesWorld.x() << ", " << velDesWorld.y() << ", " << velDesWorld.z() << "]\n";
    std::cout << "CoM cur/des: [" << std::setprecision(4) << robotState.pCoM_W.x()
              << ", " << robotState.pCoM_W.y()
              << ", " << robotState.pCoM_W.z() << "] / ["
              << comDes.x() << ", " << comDes.y() << ", " << comDes.z() << "]"
              << "   base rpy: [" << robotState.base_rpy.x()
              << ", " << robotState.base_rpy.y()
              << ", " << robotState.base_rpy.z() << "]"
              << "   nmpc status: " << robotState.qpStatus_MPC << "\n";
    std::cout << "Swing current des L/R: [" << std::setprecision(4)
              << footDesW[0].x() << ", " << footDesW[0].y() << ", " << footDesW[0].z()
              << "] / [" << footDesW[1].x() << ", " << footDesW[1].y() << ", " << footDesW[1].z()
              << "]\n";
    std::cout << "Swing final/MPC foot L/R: [" << mpcContactPositions[0].x()
              << ", " << mpcContactPositions[0].y() << ", " << mpcContactPositions[0].z()
              << "] / [" << mpcContactPositions[1].x() << ", "
              << mpcContactPositions[1].y() << ", " << mpcContactPositions[1].z()
              << "]\n";

    std::cout << "\nNMPC desired GRF / wrench [Fx Fy Fz Mx My Mz]\n";
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

    std::cout << "Waist RP stabilization [roll pitch]: ["
              << std::setw(8) << waistTorqueRp.x() << ", "
              << std::setw(8) << waistTorqueRp.y() << "] Nm\n";
    std::cout << "Lower-body inertia diag [Ixx Iyy Izz] about robot COM: ["
              << std::setw(8) << lowerBodyInertia.inertiaAboutReferenceWorld(0, 0) << ", "
              << std::setw(8) << lowerBodyInertia.inertiaAboutReferenceWorld(1, 1) << ", "
              << std::setw(8) << lowerBodyInertia.inertiaAboutReferenceWorld(2, 2) << "] kg*m^2"
              << "   mass: " << lowerBodyInertia.mass << " kg\n";

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
        else if (arg == "--record-video" || arg == "--save-video")
        {
            options.recordVideo = true;
        }
        else if (arg == "--duration" && i + 1 < argc)
        {
            options.duration = std::stod(argv[++i]);
        }
        else if (arg == "--start" && i + 1 < argc)
        {
            options.startTime = std::stod(argv[++i]);
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
            options.mpcSegmentDt = std::max(0.02, std::stod(argv[++i]));
        }
        else if ((arg == "--t-swing" || arg == "--single-support") && i + 1 < argc)
        {
            options.singleSupportTime = std::max(0.12, std::stod(argv[++i]));
        }
        else if ((arg == "--t-ds" || arg == "--double-support") && i + 1 < argc)
        {
            options.doubleSupportTime = std::max(0.0, std::stod(argv[++i]));
        }
        else if (arg == "--squat-hold" && i + 1 < argc)
        {
            options.squatHoldTime = std::max(0.0, std::stod(argv[++i]));
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
            options.swingHeight = std::stod(argv[++i]);
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
            "../record/g1_cd_nmpc_ik_walk.log" :
            "record/g1_cd_nmpc_ik_walk.log";
    const std::string lowerBodyCsvPath = outputPathNearRecordScript("g1_cd_nmpc_ik_walk_lower_body.csv");
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
    PVT_Ctr pvtCtr(mj_model->opt.timestep, jointConfig.c_str());
    tuneWalkingPvtGains(pvtCtr);
    const double simDt = mj_model->opt.timestep;
    const double mpcSegmentDt = options.mpcSegmentDt;
    const double singleSupportTime = options.singleSupportTime;
    const double doubleSupportTime = options.doubleSupportTime;
    G1CentroidalNmpc centroidalNmpc(kHorizon, mpcSegmentDt);
    centroidalNmpc.logSolve = options.nmpcLog;
    centroidalNmpc.logSolverIterations = options.nmpcSolverLog;
    GaitScheduler gaitScheduler(singleSupportTime, simDt);
    gaitScheduler.doubleSupportTime = doubleSupportTime;
    FootPlacement footPlacement;
    JoyStickInterpreter jsInterp(simDt);
    DataLogger logger(logPath);
    std::ofstream lowerBodyCsv(lowerBodyCsvPath);
    if (!lowerBodyCsv.is_open())
    {
        std::cerr << "Failed to open lower-body CSV: " << lowerBodyCsvPath << std::endl;
        return 1;
    }
    writeLowerBodyCsvHeader(lowerBodyCsv);

    const std::vector<double> standPoseStd = makeG1StandPose();
    const Eigen::VectorXd standPose =
        Eigen::Map<const Eigen::VectorXd>(standPoseStd.data(), standPoseStd.size());
    const double modelMass = totalModelMass(mj_model);
    const double nominalFootForce = 0.5 * modelMass * 9.81;
    const double squatEndTime = options.standTime + options.squatDuration;
    const double walkStartTime = std::max(options.startTime,
                                          squatEndTime + options.squatHoldTime);

    mjInterface.setMotorsPosition(standPoseStd);
    updateRobotState(mjInterface, kinDyn, robotState);
    pvtCtr.motor_pos_des_old = standPoseStd;

    const Eigen::Vector3d initialBasePos = robotState.base_pos;
    const Eigen::Vector3d initialCom = robotState.pCoM_W;
    const Eigen::Vector3d baseMinusCom = initialBasePos - initialCom;
    const double initialYaw = robotState.base_rpy.z();
    const double targetComZ = std::min(options.squatTargetComZ, initialCom.z());
    const Eigen::Matrix3d initialBaseRot = robotState.base_rot;
    const double groundZ = 0.5 * (robotState.fe_l_pos_W.z() + robotState.fe_r_pos_W.z());
    const double targetComHeight = targetComZ - groundZ;
    const double initialBaseHeight = robotState.base_pos.z();
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
    const std::array<Eigen::Matrix3d, 2> initialFootRotW = {robotState.fe_l_rot_W, robotState.fe_r_rot_W};
    const Eigen::VectorXd standHoldPose =
        Eigen::Map<const Eigen::VectorXd>(robotState.motors_pos_cur.data(), robotState.model_nv - 6);
    const Eigen::VectorXd lockedUpperBodyPose = standHoldPose;
    jsInterp.setIniPos(initialBasePos.x(), initialBasePos.y(), initialBaseHeight, initialYaw);

    Eigen::VectorXd ikWalkPose = standPose;
    std::array<Eigen::Vector3d, 2> footHoldW = {robotState.fe_l_pos_W, robotState.fe_r_pos_W};

    logger.addIterm("simTime", 1);
    logger.addIterm("baseRpy", 3);
    logger.addIterm("basePos", 3);
    logger.addIterm("com", 3);
    logger.addIterm("comDes", 3);
    logger.addIterm("velDesWorld", 3);
    logger.addIterm("contactLR", 2);
    logger.addIterm("swingPhase", 2);
    logger.addIterm("ikStatus", 1);
    logger.addIterm("ikErrNorm", 1);
    logger.addIterm("Fr_ff", 12);
    logger.addIterm("waistTauRp", 2);
    logger.addIterm("lowerBodyMass", 1);
    logger.addIterm("lowerBodyInertiaDiagRobotCom", 3);
    logger.addIterm("tau_ff", 29);
    logger.addIterm("nmpcStatus", 1);
    logger.finishItermAdding();

    if (!options.headless)
    {
        uiController.iniGLFW();
        uiController.enableTracking();
        uiController.createWindow("G1 Aligator CD-NMPC + IK Walk", options.recordVideo, videoRawPath.c_str());
    }

    int iterationCounter = 0;
    bool walkingStarted = false;
    bool gaitStarted = false;
    bool hasMpcSolution = false;
    double nextPrintTime = 0.0;
    double lastCommandedVx = std::numeric_limits<double>::quiet_NaN();
    double lastCommandedVy = std::numeric_limits<double>::quiet_NaN();
    double lastCommandedYawRate = std::numeric_limits<double>::quiet_NaN();
    double yawDes = initialYaw;
    Eigen::Vector3d comDes = initialCom;
    Eigen::Vector3d worldComDesired = initialCom;
    Eigen::Matrix<double, 12, 1> lastMpcWrench = Eigen::Matrix<double, 12, 1>::Zero();
    lastMpcWrench << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0,
        0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, 12, 1> lastMpcPlaneForces = Eigen::Matrix<double, 12, 1>::Zero();
    lastMpcPlaneForces.segment<6>(0) = g1_kin_dyn::distributeVerticalFootLoad(nominalFootForce);
    lastMpcPlaneForces.segment<6>(6) = g1_kin_dyn::distributeVerticalFootLoad(nominalFootForce);
    Eigen::Vector2d lastWaistTorqueRp = Eigen::Vector2d::Zero();
    int ikStatus = 0;
    double ikErrNorm = 0.0;

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
            const double preWalkShiftPhase = smoothStep((time - squatEndTime) / std::max(1e-3, options.squatHoldTime));
            Eigen::Vector3d firstStanceComDes = initialCom;
            firstStanceComDes.y() = initialCom.y() + kPreWalkStanceShiftY;
            firstStanceComDes.z() = targetComZ;
            preWalkComDes.y() = initialCom.y() + (firstStanceComDes.y() - initialCom.y()) * preWalkShiftPhase;

            const bool walkingEnabled = time >= walkStartTime;
            if (walkingEnabled && !walkingStarted)
            {
                walkingStarted = true;
                worldComDesired = preWalkComDes;
                worldComDesired.head<2>() = initialCom.head<2>();
                worldComDesired.z() = targetComZ;
                yawDes = robotState.base_rpy.z();
                footHoldW = {robotState.fe_l_pos_W, robotState.fe_r_pos_W};
                gaitScheduler.stop();
                gaitScheduler.stepNumDes = std::numeric_limits<int>::max();
                gaitStarted = false;
                hasMpcSolution = false;
                lastMpcWrench << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0,
                    0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;
                lastMpcPlaneForces.setZero();
                lastMpcPlaneForces.segment<6>(0) = g1_kin_dyn::distributeVerticalFootLoad(nominalFootForce);
                lastMpcPlaneForces.segment<6>(6) = g1_kin_dyn::distributeVerticalFootLoad(nominalFootForce);
                jsInterp.setIniPos(robotState.q(0), robotState.q(1), initialBaseHeight, robotState.base_rpy.z());
                lastCommandedVx = std::numeric_limits<double>::quiet_NaN();
                lastCommandedVy = std::numeric_limits<double>::quiet_NaN();
                lastCommandedYawRate = std::numeric_limits<double>::quiet_NaN();
            }

            Eigen::Matrix<int, Eigen::Dynamic, 2> contactTable(kHorizon, 2);
            contactTable.setOnes();
            if (walkingEnabled)
            {
                if (!std::isfinite(lastCommandedYawRate) ||
                    std::abs(lastCommandedYawRate - options.yawRate) > 1e-9)
                {
                    jsInterp.setWzDesLPara(options.yawRate, 1.0);
                    lastCommandedYawRate = options.yawRate;
                }
                if (!std::isfinite(lastCommandedVx) || std::abs(lastCommandedVx - options.vx) > 1e-9)
                {
                    jsInterp.setVxDesLPara(options.vx, std::max(1e-3, options.velocityRamp));
                    lastCommandedVx = options.vx;
                }
                if (!std::isfinite(lastCommandedVy) || std::abs(lastCommandedVy - options.vy) > 1e-9)
                {
                    jsInterp.setVyDesLPara(options.vy, std::max(1e-3, options.velocityRamp));
                    lastCommandedVy = options.vy;
                }
                jsInterp.step();
                jsInterp.dataBusWrite(robotState);
            }
            else
            {
                lastCommandedVx = std::numeric_limits<double>::quiet_NaN();
                lastCommandedVy = std::numeric_limits<double>::quiet_NaN();
                lastCommandedYawRate = std::numeric_limits<double>::quiet_NaN();
                jsInterp.setIniPos(robotState.q(0), robotState.q(1), initialBaseHeight, robotState.base_rpy.z());
                robotState.js_pos_des << initialBasePos.x(), initialBasePos.y(), initialBaseHeight;
                robotState.js_vel_des.setZero();
                robotState.js_omega_des.setZero();
                robotState.base_pos_des << initialBasePos.x(), initialBasePos.y(), initialBaseHeight;
                robotState.base_vel_des.setZero();
                robotState.base_omega_des.setZero();
                robotState.base_rpy_des << 0.0, 0.0, initialYaw;
            }

            Eigen::Vector3d velDesWorld = robotState.js_vel_des;
            const double yawRateDes = robotState.js_omega_des.z();
            yawDes = walkingEnabled ? robotState.base_rpy_des.z() : initialYaw;

            if (walkingEnabled)
            {
                if (!gaitStarted)
                {
                    gaitScheduler.start();
                    gaitStarted = true;
                }
                robotState.motionState = DataBus::Walk;
                gaitScheduler.dataBusRead(robotState);
                gaitScheduler.step();
                gaitScheduler.dataBusWrite(robotState);

                footPlacement.dataBusRead(robotState);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(robotState);

                worldComDesired.head<2>() += velDesWorld.head<2>() * simDt;
                worldComDesired.z() = targetComZ;
                comDes = worldComDesired;
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
                robotState.tSwing = singleSupportTime;
                robotState.tDoubleSupport = doubleSupportTime;
                robotState.stance_fe_pos_cur_W = 0.5 * (robotState.fe_l_pos_W + robotState.fe_r_pos_W);
                robotState.stance_fe_rot_cur_W = robotState.fe_r_rot_W;
                robotState.walk_target_yaw = initialYaw;
                robotState.walk_target_yaw_rate = 0.0;
                robotState.walk_target_com_height = targetComHeight;
                worldComDesired = preWalkComDes;
                comDes = preWalkComDes;
                yawDes = initialYaw;
            }

            std::array<Eigen::Vector3d, 2> footDesW = footHoldW;
            std::array<Eigen::Vector3d, 2> mpcContactPositions = footHoldW;
            if (walkingEnabled)
            {
                if (!robotState.walk_left_contact)
                {
                    footDesW[0] = robotState.swingDesPosCur_W;
                    mpcContactPositions[0] = robotState.swingDesPosFinal_W;
                    footHoldW[1] = robotState.fe_r_pos_W;
                    footHoldW[0] = robotState.fe_l_pos_W;
                }
                else if (!robotState.walk_right_contact)
                {
                    footDesW[1] = robotState.swingDesPosCur_W;
                    mpcContactPositions[1] = robotState.swingDesPosFinal_W;
                    footHoldW[0] = robotState.fe_l_pos_W;
                    footHoldW[1] = robotState.fe_r_pos_W;
                }
                else
                {
                    footHoldW[0] = robotState.fe_l_pos_W;
                    footHoldW[1] = robotState.fe_r_pos_W;
                }
            }
            else
            {
                footHoldW[0] = robotState.fe_l_pos_W;
                footHoldW[1] = robotState.fe_r_pos_W;
                footDesW = footHoldW;
                mpcContactPositions = footHoldW;
            }

            const bool leftContact = robotState.walk_left_contact;
            const bool rightContact = robotState.walk_right_contact;
            const Eigen::Vector2d swingPhase = swingPhaseFromRobotState(robotState);
            const Pin_KinDyn::CompositeInertia lowerBodyInertia =
                kinDyn.computeLowerBodyInertia(robotState.pCoM_W);
            writeLowerBodyCsvRow(lowerBodyCsv, mj_data->time, robotState,
                                 leftContact, rightContact, lowerBodyInertia);
            contactTable = previewContactTable(robotState, mpcSegmentDt);
            const std::array<Eigen::Vector3d, 2> measuredFootPositions = {
                robotState.fe_l_pos_W,
                robotState.fe_r_pos_W};
            const std::vector<std::array<Eigen::Vector3d, 2>> mpcContactPositionsHorizon =
                previewContactPositions(robotState, measuredFootPositions, mpcContactPositions, mpcSegmentDt);

            fillWalkBus(robotState, comDes, velDesWorld, yawDes, yawRateDes,
                        leftContact, rightContact, nominalFootForce, walkingEnabled, targetComHeight);

            if (!walkingEnabled)
            {
                lastMpcWrench << 0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0,
                    0.0, 0.0, nominalFootForce, 0.0, 0.0, 0.0;
                lastMpcPlaneForces.setZero();
                lastMpcPlaneForces.segment<6>(0) = g1_kin_dyn::distributeVerticalFootLoad(nominalFootForce);
                lastMpcPlaneForces.segment<6>(6) = g1_kin_dyn::distributeVerticalFootLoad(nominalFootForce);
                robotState.qpStatus_MPC = 0;
                hasMpcSolution = true;
            }
            else if (iterationCounter % kMpcSolveEveryIterations == 0)
            {
                const G1CentroidalNmpc::Input mpcInput =
                    buildWalkingMpcInput(robotState, comDes, velDesWorld, yawDes, yawRateDes,
                                         contactTable, mpcContactPositions, mpcContactPositionsHorizon,
                                         modelMass, mpcSegmentDt);
                if (centroidalNmpc.solve(mpcInput))
                {
                    lastMpcWrench = limitMpcWrenches(centroidalNmpc.firstWrenches(),
                                                     leftContact, rightContact, modelMass);
                    lastMpcWrench = sanitizeWrench(lastMpcWrench,
                                                   nominalWrench(leftContact, rightContact, modelMass),
                                                   leftContact, rightContact);
                    lastMpcPlaneForces = shapedMpcPlaneForces(lastMpcWrench, leftContact, rightContact, modelMass);
                    centroidalNmpc.dataBusWrite(robotState);
                    hasMpcSolution = true;
                }
                robotState.qpStatus_MPC = centroidalNmpc.status();
            }
            if (hasMpcSolution)
            {
                robotState.Fr_ff = lastMpcWrench;
            }

            const Eigen::Matrix3d yawDelta = Rz3(yawDes - initialYaw);
            const Eigen::Matrix3d leftFootRotWDes = yawDelta * initialFootRotW[0];
            const Eigen::Matrix3d rightFootRotWDes = yawDelta * initialFootRotW[1];
            Eigen::Vector3d basePosForIk = walkingEnabled ? robotState.base_pos : initialBasePos + (comDes - initialCom);
            const double baseZDes = comDes.z() + baseMinusCom.z();
            const double comZCorrection =
                std::clamp(comDes.z() - robotState.pCoM_W.z(), -0.04, 0.04);
            basePosForIk.z() = baseZDes + comZCorrection;
            const Eigen::Matrix3d baseRotForIk = yawDelta * initialBaseRot;
            const Eigen::Vector3d leftFootPosLDes = baseRotForIk.transpose() * (footDesW[0] - basePosForIk);
            const Eigen::Vector3d rightFootPosLDes = baseRotForIk.transpose() * (footDesW[1] - basePosForIk);
            const Eigen::Matrix3d leftFootRotLDes = baseRotForIk.transpose() * leftFootRotWDes;
            const Eigen::Matrix3d rightFootRotLDes = baseRotForIk.transpose() * rightFootRotWDes;
            Pin_KinDyn::IkRes ikRes =
                kinDyn.computeInK_Leg(leftFootRotLDes, leftFootPosLDes, rightFootRotLDes, rightFootPosLDes);
            ikStatus = ikRes.status;
            ikErrNorm = ikRes.err.norm();
            if (ikRes.err.norm() < 8e-2 && ikRes.jointPosRes.size() >= 12)
            {
                ikWalkPose.head(kLegMotorCount) = ikRes.jointPosRes.head(kLegMotorCount);
            }
            ikWalkPose.tail(ikWalkPose.size() - kLegMotorCount) =
                lockedUpperBodyPose.tail(lockedUpperBodyPose.size() - kLegMotorCount);

            const double ikRamp = std::clamp(mj_data->time / 0.5, 0.0, 1.0);
            const Eigen::VectorXd motorPosDes = (1.0 - ikRamp) * standHoldPose + ikRamp * ikWalkPose;
            robotState.motors_pos_des = eigen2std(motorPosDes);
            robotState.motors_vel_des.assign(robotState.model_nv - 6, 0.0);
            lastWaistTorqueRp = waistRpStabilizingTorque(robotState);
            const double ffRamp = std::clamp(mj_data->time / kFeedforwardRampTime, 0.0, 1.0);
            const Eigen::VectorXd tauFeedforward =
                contactTorqueFeedforward(robotState, lastMpcPlaneForces, lastWaistTorqueRp,
                                         ffRamp * options.ffScale);
            robotState.motors_tor_des = eigen2std(tauFeedforward);

            pvtCtr.dataBusRead(robotState);
            pvtCtr.calMotorsPVT(0.02);
            pvtCtr.dataBusWrite(robotState);
            mjInterface.setMotorsTorque(robotState.motors_tor_out);

            if (options.print && mj_data->time + 1e-12 >= nextPrintTime)
            {
                printRuntimeSummary(mj_data->time, robotState, comDes, velDesWorld,
                                    footDesW, mpcContactPositions, lastWaistTorqueRp,
                                    lowerBodyInertia,
                                    swingPhase,
                                    gaitScheduler.stepNumCur,
                                    leftContact, rightContact, tauFeedforward);
                nextPrintTime += 1.0 / options.printHz;
                if (nextPrintTime < mj_data->time)
                {
                    nextPrintTime = mj_data->time + 1.0 / options.printHz;
                }
            }

            logger.startNewLine();
            logger.recItermData("simTime", mj_data->time);
            logger.recItermData("baseRpy", robotState.base_rpy);
            logger.recItermData("basePos", robotState.base_pos);
            logger.recItermData("com", robotState.pCoM_W);
            logger.recItermData("comDes", comDes);
            logger.recItermData("velDesWorld", velDesWorld);
            logger.recItermData("contactLR", (Eigen::Vector2d() << static_cast<double>(leftContact),
                                               static_cast<double>(rightContact))
                                                  .finished());
            logger.recItermData("swingPhase", swingPhase);
            logger.recItermData("ikStatus", static_cast<double>(ikStatus));
            logger.recItermData("ikErrNorm", ikErrNorm);
            logger.recItermData("Fr_ff", robotState.Fr_ff);
            logger.recItermData("waistTauRp", lastWaistTorqueRp);
            logger.recItermData("lowerBodyMass", lowerBodyInertia.mass);
            logger.recItermData("lowerBodyInertiaDiagRobotCom", lowerBodyInertia.inertiaAboutReferenceWorld.diagonal());
            logger.recItermData("tau_ff", tauFeedforward);
            logger.recItermData("nmpcStatus", static_cast<double>(robotState.qpStatus_MPC));
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

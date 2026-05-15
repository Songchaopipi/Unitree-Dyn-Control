/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "foot_placement.h"
#include "bezier_1D.h"
#include "useful_math.h"
#include <algorithm>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

namespace
{
Eigen::Vector2d SolveDlqrGain2x1(const Eigen::Matrix2d &A, const Eigen::Vector2d &B)
{
    const Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
    const double R = 20.0;
    Eigen::Matrix2d G = B * (1.0 / R) * B.transpose();
    Eigen::Matrix2d H = Q;
    Eigen::Matrix2d HOld = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d AIter = A;

    for (int i = 0; i < 1000; ++i)
    {
        HOld = H;
        Eigen::Matrix2d invW = (Eigen::Matrix2d::Identity() + G * H).inverse();
        Eigen::Matrix2d V1 = invW * AIter;
        Eigen::Matrix2d V2 = G * invW;
        G = G + AIter * V2 * AIter.transpose();
        H = H + V1.transpose() * H * AIter;
        AIter = AIter * V1;
        if ((H - HOld).norm() <= 1e-8 * std::max(1.0, H.norm()))
        {
            break;
        }
    }

    const double denom = B.transpose() * H * B + R;
    return -(B.transpose() * H * A / denom).transpose();
}

struct HlipAxis
{
    Eigen::Matrix2d Astep;
    Eigen::Vector2d Bstep;
    Eigen::Vector2d K;
    Eigen::Matrix2d Ass;
};

HlipAxis MakeHlipAxis(double zNom, double singleSupportTime, double doubleSupportTime, bool useAngularMomentum)
{
    HlipAxis axis;
    if (useAngularMomentum)
    {
        axis.Ass << 0.0, 1.0 / zNom,
            9.81, 0.0;
    }
    else
    {
        axis.Ass << 0.0, 1.0,
            9.81 / zNom, 0.0;
    }
    Eigen::Matrix2d Ads;
    if (useAngularMomentum)
    {
        Ads << 0.0, 1.0 / zNom,
            0.0, 0.0;
    }
    else
    {
        Ads << 0.0, 1.0,
            0.0, 0.0;
    }
    const Eigen::Vector2d stepInputMap(-1.0, 0.0);
    axis.Bstep = (singleSupportTime * axis.Ass).exp() * stepInputMap;
    axis.Astep = (singleSupportTime * axis.Ass).exp() * (doubleSupportTime * Ads).exp();
    axis.K = SolveDlqrGain2x1(axis.Astep, axis.Bstep);
    return axis;
}

Eigen::Vector2d PeriodOneTarget(const HlipAxis &axis, double nominalStep)
{
    return (Eigen::Matrix2d::Identity() - axis.Astep).inverse() * axis.Bstep * nominalStep;
}

Eigen::Vector2d PeriodTwoTarget(const HlipAxis &axis, double nominalStepThis, double nominalStepNext)
{
    return (Eigen::Matrix2d::Identity() - axis.Astep * axis.Astep).inverse() *
           (axis.Astep * axis.Bstep * nominalStepThis + axis.Bstep * nominalStepNext);
}

Eigen::Vector2d FilterPivotMomentum(double dt,
                                    const Eigen::Vector2d &measurement,
                                    const Eigen::Vector2d &input,
                                    Eigen::Vector2d &state,
                                    Eigen::Matrix2d &covariance,
                                    bool &initialized)
{
    const double positionVar = 1.0;
    const double measuredMomentumVar = 0.1;
    if (!initialized)
    {
        covariance = measuredMomentumVar * Eigen::Matrix2d::Identity();
        state = measurement;
        initialized = true;
    }

    const Eigen::Matrix2d measurementCov = measuredMomentumVar * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d processCov =
        std::pow(dt * 9.81 * positionVar, 2.0) * Eigen::Matrix2d::Identity();
    const Eigen::Vector2d prior = state + input;
    covariance = covariance + processCov;

    const Eigen::Matrix2d innovationCov = covariance + measurementCov;
    const Eigen::Matrix2d gain = covariance * innovationCov.inverse();
    state = prior + gain * (measurement - prior);
    covariance = (Eigen::Matrix2d::Identity() - gain) * covariance;
    return state;
}
} // namespace

void FootPlacement::dataBusRead(DataBus &robotState)
{
    posStart_W = robotState.swingStartPos_W;
    desV_W = robotState.js_vel_des;
    desWz_W = robotState.js_omega_des(2);
    curV_W = robotState.dq.block<3, 1>(0, 0);
    phi = robotState.phi;
    hipPos_W = robotState.posHip_W;
    STPos_W = robotState.posST_W;
    base_pos = robotState.base_pos;
    com_pos_W = robotState.pCoM_W;
    com_vel_W = robotState.Jcom_W * robotState.dq;
    base_rpy = robotState.base_rpy;
    centroidalMomentum_W = robotState.dyn_Ag * robotState.dq;
    tSwing = robotState.tSwing;
    theta0 = robotState.theta0;
    yawCur = robotState.rpy[2];
    omegaZ_W = robotState.base_omega_W(2);
    hip_width = robotState.width_hips;
    legState = robotState.legState;
    stanceLeg = robotState.walk_stance_leg;
    doubleSupportTime = robotState.tDoubleSupport;
}

void FootPlacement::dataBusWrite(DataBus &robotState)
{
    robotState.swingDesPosCur_W << pDesCur[0], pDesCur[1], pDesCur[2];
    robotState.swingDesPosFinal_W = posDes_W;
    robotState.swing_fe_rpy_des_W << 0, 0, robotState.base_rpy_des(2); // WARNING! ThetaZ!
    robotState.swing_fe_pos_des_W << pDesCur[0], pDesCur[1], pDesCur[2];
    robotState.swing_fe_vel_des_W = swingVelDes_W;
    robotState.swing_fe_acc_des_W = swingAccDes_W;
    robotState.walk_yd = yd;
    robotState.walk_dyd = dyd;
    robotState.walk_d2yd = d2yd;
    robotState.walk_target_yaw = targetYaw;
    robotState.walk_target_yaw_rate = targetYawRate;
    robotState.walk_target_com_height = yd(0);
    robotState.walk_desired_vx = desV_W.dot(Rz3(targetYaw).col(0));
    robotState.walk_desired_vy = desV_W.dot(Rz3(targetYaw).col(1));
}
void FootPlacement::getSwingPos()
{
    const bool inSingleSupport = (legState == DataBus::LSt || legState == DataBus::RSt);
    const bool isNewStep = inSingleSupport && (legState != lastLegState);
    if (isNewStep)
    {
        lastLegState = legState;
        pivotMomentumInitialized = false;
    }

    if (!targetYawInitialized)
    {
        targetYaw = yawCur;
        targetYawInitialized = true;
    }
    targetYawRate = desWz_W;
    targetYaw += targetYawRate * dt;

    const double zNom = std::max(0.45, legLength);
    const double tRemain = std::max(0.0, (1.0 - phi) * tSwing);
    const double stepWidthNominal = std::max(0.20, hip_width);

    if (!inSingleSupport)
    {
        yd.setZero(10);
        dyd.setZero(10);
        d2yd.setZero(10);
        if (referenceInitialized)
        {
            yd = referenceAtStepStart;
        }
        yd(0) = zNom;
        pDesCur[0] = posStart_W.x();
        pDesCur[1] = posStart_W.y();
        pDesCur[2] = posStart_W.z();
        posDes_W = posStart_W;
        swingVelDes_W.setZero();
        swingAccDes_W.setZero();
        return;
    }

    Eigen::Matrix3d Rtarget = Rz3(targetYaw);
    Eigen::Matrix3d RtargetT = Rtarget.transpose();
    Eigen::Vector3d comRel = RtargetT * (com_pos_W - STPos_W);
    Eigen::Vector3d comVel = RtargetT * com_vel_W;
    Eigen::Vector3d normalizedAngularMomentum = Eigen::Vector3d::Zero();
    if (robotMass > 1e-6 && centroidalMomentum_W.size() >= 6)
    {
        normalizedAngularMomentum = RtargetT * (centroidalMomentum_W.tail<3>() / robotMass);
    }

    const Eigen::Vector3d orbitalComponent = comRel.cross(comVel);
    Eigen::Vector3d pivotAngularMomentum;
    pivotAngularMomentum << -normalizedAngularMomentum.x() - orbitalComponent.x(),
        normalizedAngularMomentum.y() + orbitalComponent.y(),
        normalizedAngularMomentum.z() + orbitalComponent.z();

    Eigen::Vector2d pivotInput;
    pivotInput << dt * 9.81 * comRel.y(), dt * 9.81 * comRel.x();
    Eigen::Vector2d filteredPivot = FilterPivotMomentum(dt,
                                                        pivotAngularMomentum.head<2>(),
                                                        pivotInput,
                                                        pivotMomentumFiltered,
                                                        pivotMomentumP,
                                                        pivotMomentumInitialized);
    pivotAngularMomentum.x() = filteredPivot.x();
    pivotAngularMomentum.y() = filteredPivot.y();

    Eigen::Vector4d reducedState;
    if (useAngularMomentumState)
    {
        reducedState << comRel.x(),
            pivotAngularMomentum.y(),
            comRel.y(),
            pivotAngularMomentum.x();
    }
    else
    {
        reducedState << comRel.x(),
            comVel.x(),
            comRel.y(),
            comVel.y();
    }

    if (isNewStep)
    {
        referenceAtStepStart.setZero(10);
        referenceAtStepStart(0) = com_pos_W.z() - STPos_W.z();
        referenceAtStepStart.segment<3>(4) = posStart_W - STPos_W;
        referenceInitialized = true;

        const Eigen::VectorXd swingHorizontalBlend =
            (Eigen::VectorXd(5) << 0.0, 0.0, 1.0, 1.0, 1.0).finished();
        const Eigen::VectorXd comHeightBlend =
            (Eigen::VectorXd(6) << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0).finished();
        comHeightCurve = (Eigen::VectorXd::Ones(comHeightBlend.size()) - comHeightBlend) *
                             referenceAtStepStart(0) +
                         comHeightBlend * zNom;

        const double clippedYawDelta = std::clamp(targetYawRate * tSwing, -0.2, 0.2);
        const double targetHipYaw = 0.5 * clippedYawDelta;
        stanceHipYawCurve = (Eigen::VectorXd::Ones(swingHorizontalBlend.size()) - swingHorizontalBlend) *
                                referenceAtStepStart(1) +
                            swingHorizontalBlend * targetHipYaw;
        swingHipYawCurve = (Eigen::VectorXd::Ones(swingHorizontalBlend.size()) - swingHorizontalBlend) *
                               referenceAtStepStart(7) -
                           swingHorizontalBlend * targetHipYaw;

        const double touchdownOvershoot = -0.01;
        swingZCurve << referenceAtStepStart(6),
            referenceAtStepStart(6),
            stepHeight / 3.0,
            stepHeight,
            stepHeight,
            stepHeight / 2.0,
            0.0,
            0.0,
            touchdownOvershoot;
    }

    Eigen::Vector2d stepLocalXY = PlanHlipFootstep(reducedState, tRemain, zNom,
                                                   tSwing, doubleSupportTime,
                                                   stepWidthNominal);
    Eigen::Vector3d stepInBaseYaw = Rz3(targetYaw - yawCur) * Eigen::Vector3d(stepLocalXY.x(), stepLocalXY.y(), 0.0);
    const double lateralStepMin = 0.10;
    const double lateralStepMax = 0.60;
    if (legState == DataBus::LSt)
    {
        stepInBaseYaw.y() = std::clamp(stepInBaseYaw.y(), -lateralStepMax, -lateralStepMin);
    }
    else
    {
        stepInBaseYaw.y() = std::clamp(stepInBaseYaw.y(), lateralStepMin, lateralStepMax);
    }
    const Eigen::Vector3d plannedStep_W = Rz3(yawCur) * stepInBaseYaw;
    lockedFinalTarget_W = STPos_W + plannedStep_W;
    lockedFinalTarget_W(2) = STPos_W(2) - 0.01;
    lastFinalTargetY = lockedFinalTarget_W(1);
    posDes_W = lockedFinalTarget_W;

    const Eigen::VectorXd swingHorizontalBlend =
        (Eigen::VectorXd(5) << 0.0, 0.0, 1.0, 1.0, 1.0).finished();
    swingXCurve = (Eigen::VectorXd::Ones(swingHorizontalBlend.size()) - swingHorizontalBlend) *
                      referenceAtStepStart(4) +
                  swingHorizontalBlend * plannedStep_W.x();
    swingYCurve = (Eigen::VectorXd::Ones(swingHorizontalBlend.size()) - swingHorizontalBlend) *
                      referenceAtStepStart(5) +
                  swingHorizontalBlend * plannedStep_W.y();

    const double sPhi = std::clamp(phi, 0.0, 1.0);
    const double invSwing = 1.0 / std::max(1e-3, tSwing);
    yd.setZero(10);
    dyd.setZero(10);
    d2yd.setZero(10);

    auto writeBezierReference = [&](const Eigen::VectorXd &coeff, int index)
    {
        std::vector<double> coeffStd(coeff.data(), coeff.data() + coeff.size());
        yd(index) = Bezier(coeffStd, sPhi);
        dyd(index) = BezierD1(coeffStd, sPhi) * invSwing;
        d2yd(index) = BezierD2(coeffStd, sPhi) * invSwing * invSwing;
    };

    writeBezierReference(comHeightCurve, 0);
    writeBezierReference(stanceHipYawCurve, 1);
    writeBezierReference(swingXCurve, 4);
    writeBezierReference(swingYCurve, 5);
    writeBezierReference(swingZCurve, 6);
    writeBezierReference(swingHipYawCurve, 7);

    const Eigen::Vector3d swingRelDes = yd.segment<3>(4);
    pDesCur[0] = STPos_W.x() + swingRelDes.x();
    pDesCur[1] = STPos_W.y() + swingRelDes.y();
    pDesCur[2] = STPos_W.z() + swingRelDes.z();

    swingVelDes_W = dyd.segment<3>(4);
    swingAccDes_W = d2yd.segment<3>(4);

}

double FootPlacement::Bezier(const std::vector<double> &coeff, double s) const
{
    Bezier_1D curve;
    curve.P = coeff;
    return curve.getOut(std::clamp(s, 0.0, 1.0));
}

double FootPlacement::BezierD1(const std::vector<double> &coeff, double s) const
{
    if (coeff.size() < 2)
    {
        return 0.0;
    }
    const double order = static_cast<double>(coeff.size() - 1);
    std::vector<double> derivative(coeff.size() - 1, 0.0);
    for (std::size_t i = 0; i + 1 < coeff.size(); ++i)
    {
        derivative[i] = order * (coeff[i + 1] - coeff[i]);
    }
    return Bezier(derivative, s);
}

double FootPlacement::BezierD2(const std::vector<double> &coeff, double s) const
{
    if (coeff.size() < 3)
    {
        return 0.0;
    }
    const double order = static_cast<double>(coeff.size() - 1);
    const double secondOrder = static_cast<double>(coeff.size() - 2);
    std::vector<double> derivative2(coeff.size() - 2, 0.0);
    for (std::size_t i = 0; i + 2 < coeff.size(); ++i)
    {
        derivative2[i] = order * secondOrder * (coeff[i + 2] - 2.0 * coeff[i + 1] + coeff[i]);
    }
    return Bezier(derivative2, s);
}

Eigen::Vector2d FootPlacement::PlanHlipFootstep(const Eigen::Vector4d &reducedState,
                                                double tRemain,
                                                double zNom,
                                                double singleSupportTimeIn,
                                                double doubleSupportTimeIn,
                                                double stepWidthNominal) const
{
    const double singleSupportTime = std::max(1e-3, singleSupportTimeIn);
    const double doubleSupportTime = std::max(0.0, doubleSupportTimeIn);
    const HlipAxis sagittal = MakeHlipAxis(zNom, singleSupportTime, doubleSupportTime, useAngularMomentumState);
    const HlipAxis lateral = MakeHlipAxis(zNom, singleSupportTime, doubleSupportTime, useAngularMomentumState);

    const Eigen::Vector2d sagittalPreImpact = (std::max(0.0, tRemain) * sagittal.Ass).exp() * reducedState.head<2>();
    const Eigen::Vector2d lateralPreImpact = (std::max(0.0, tRemain) * lateral.Ass).exp() * reducedState.tail<2>();

    const double desiredVx = desV_W.dot(Rz3(targetYaw).col(0));
    const double desiredVy = desV_W.dot(Rz3(targetYaw).col(1));
    const double sagittalNominalStep = desiredVx * singleSupportTime;
    const Eigen::Vector2d sagittalTarget = PeriodOneTarget(sagittal, sagittalNominalStep);

    const bool leftStance = (legState == DataBus::LSt);
    const double leftNominalStep = -stepWidthNominal;
    const double rightNominalStep = 2.0 * desiredVy * singleSupportTime - leftNominalStep;
    const Eigen::Vector2d lateralTarget = leftStance
                                              ? PeriodTwoTarget(lateral, leftNominalStep, rightNominalStep)
                                              : PeriodTwoTarget(lateral, rightNominalStep, leftNominalStep);
    const double lateralNominalStep = leftStance ? leftNominalStep : rightNominalStep;

    Eigen::Vector2d plannedStep;
    plannedStep.x() = sagittal.K.dot(sagittalPreImpact - sagittalTarget) + sagittalNominalStep;
    plannedStep.y() = lateral.K.dot(lateralPreImpact - lateralTarget) + lateralNominalStep;
    return plannedStep;
}

double FootPlacement::Trajectory(double phase, double hei, double len)
{
    Bezier_1D Bswpid;
    double para0 = 5, para1 = 3;
    for (int i = 0; i < para0; i++)
    {
        Bswpid.P.push_back(0.0);
    }
    for (int i = 0; i < para1; i++)
    {
        Bswpid.P.push_back(1.0);
    }

    double output;
    if (phi < phase)
    {
        output = hei * Bswpid.getOut(phi / phase);
    }
    else
    {
        double s = Bswpid.getOut((1.4 - phi) / (1.4 - phase));
        if (s > 0)
        {
            output = hei * s + len * (1.0 - s);
        }
        else
        {
            output = len;
        }
    }
    return output;
}

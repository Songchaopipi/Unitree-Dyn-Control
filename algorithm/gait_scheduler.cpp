/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "gait_scheduler.h"

#include <algorithm>

namespace
{
DataBus::LegState OppositeStance(DataBus::LegState stance)
{
    return stance == DataBus::LSt ? DataBus::RSt : DataBus::LSt;
}
}

GaitScheduler::GaitScheduler(double tSwingIn, double dtIn)
{
    singleSupportTime = tSwingIn;
    tSwing = singleSupportTime;
    doubleSupportTime = 0.0;
    dt = dtIn;
    phi = 0;
    isIni = false;
	firstleg=DataBus::RSt;
    legState=DataBus::DSt;
    legStateNext=OppositeStance(firstleg);
    stanceLeg = firstleg;
    motionState=DataBus::Stand;
    enableNextStep= false;
    touchDown = false;
    supportPhase = SupportPhase::DoubleSupport;
    phaseTime = 0.0;
}

void GaitScheduler::dataBusRead(const DataBus &robotState)
{
    model_nv = robotState.model_nv;
    torJoint = Eigen::VectorXd::Zero(model_nv - 6);
    for (int i = 0; i < model_nv - 6; i++)
    {
        torJoint[i] = robotState.motors_tor_cur[i];
    }
    dyn_M = robotState.dyn_M;
    dyn_Non = robotState.dyn_Non;
    J_l = robotState.J_l;
    dJ_l = robotState.dJ_l;
    J_r = robotState.J_r;
    dJ_r = robotState.dJ_r;
    Fz_L_m = robotState.fL[2];
    Fz_R_m = robotState.fR[2];
    hip_l_pos_W = robotState.hip_l_pos_W;
    hip_r_pos_W = robotState.hip_r_pos_W;
    fe_r_pos_W = robotState.fe_r_pos_W;
    fe_l_pos_W = robotState.fe_l_pos_W;
    fe_l_rot_W = robotState.fe_l_rot_W;
    fe_r_rot_W = robotState.fe_r_rot_W;
    dq = robotState.dq;
    motionState = robotState.motionState;
}

void GaitScheduler::dataBusWrite(DataBus &robotState)
{
    robotState.tSwing = singleSupportTime;
    robotState.tDoubleSupport = doubleSupportTime;
    robotState.swingStartPos_W = swingStartPos_W;
    robotState.stanceDesPos_W = stanceStartPos_W;
    robotState.posHip_W = posHip_W;
    robotState.posST_W = posST_W;
    robotState.theta0 = theta0;
    robotState.legState = legState;
    robotState.legStateNext = legStateNext;
    robotState.walk_stance_leg = stanceLeg;
    robotState.phi = phi;
    robotState.walk_is_double_support = (supportPhase == SupportPhase::DoubleSupport);
    robotState.walk_left_contact = (supportPhase == SupportPhase::DoubleSupport) || (stanceLeg == DataBus::LSt);
    robotState.walk_right_contact = (supportPhase == SupportPhase::DoubleSupport) || (stanceLeg == DataBus::RSt);
    robotState.walk_phase_time = phaseTime;
    robotState.walk_time_to_impact =
        (supportPhase == SupportPhase::SingleSupport) ? std::max(0.0, singleSupportTime - phaseTime) : 0.0;
    robotState.FL_est = FLest;
    robotState.FR_est = FRest;
    if (stanceLeg == DataBus::LSt)
    {
        robotState.stance_fe_pos_cur_W = fe_l_pos_W;
        robotState.stance_fe_rot_cur_W = fe_l_rot_W;
    }
    else {
        robotState.stance_fe_pos_cur_W=fe_r_pos_W;
        robotState.stance_fe_rot_cur_W=fe_r_rot_W;
    }
    robotState.motionState = motionState;
}

void GaitScheduler::step()
{
    Eigen::VectorXd tauAll;
    tauAll = Eigen::VectorXd::Zero(model_nv);
    tauAll.block(6, 0, model_nv - 6, 1) = torJoint;
    if (dyn_M.size() > 0)
    {
        FLest = -pseudoInv_SVD(J_l * dyn_M.inverse() * J_l.transpose()) * (J_l * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_l * dq);
        FRest = -pseudoInv_SVD(J_r * dyn_M.inverse() * J_r.transpose()) * (J_r * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_r * dq);
    }
    else
    {
        FLest = Eigen::VectorXd::Zero(6);
        FRest = Eigen::VectorXd::Zero(6);
    }

    if (motionState == DataBus::Stand)
    {
        phi = 0;
        phaseTime = 0.0;
        isIni = false;
        enableNextStep = false;
        stepNumCur=0;
        legState = DataBus::DSt;
        stanceLeg = firstleg;
        legStateNext = OppositeStance(stanceLeg);
        supportPhase = SupportPhase::DoubleSupport;
    }
    else if (motionState == DataBus::Walk)
    {
        enableNextStep = true;
        if (!isIni && start_walk)
        {
            isIni = true;
            supportPhase = SupportPhase::DoubleSupport;
            legState = DataBus::DSt;
            stanceLeg = firstleg;
            legStateNext = OppositeStance(stanceLeg);
            phaseTime = 0.0;
        }

        phaseTime += dt;
        bool transitioned = true;
        while (transitioned)
        {
            transitioned = false;
            if (supportPhase == SupportPhase::DoubleSupport && phaseTime >= doubleSupportTime)
            {
                transitioned = enterSingleSupport();
            }
            if (supportPhase == SupportPhase::SingleSupport && phaseTime >= singleSupportTime)
            {
                const bool readyForDoubleSupport =
                    swingFootReadyForSwitch() || phaseTime >= singleSupportTime + 0.4;
                if (readyForDoubleSupport)
                {
                    enterDoubleSupport();
                    transitioned = true;
                }
            }
        }

        phi = (supportPhase == SupportPhase::SingleSupport)
                  ? std::clamp(phaseTime / std::max(1e-6, singleSupportTime), 0.0, 1.0)
                  : 0.0;
    }
    else if (motionState == DataBus::Walk2Stand)
    {
        phaseTime += dt;
        phi = std::clamp(phaseTime / std::max(1e-6, singleSupportTime), 0.0, 1.0);
        if (supportPhase == SupportPhase::SingleSupport && swingFootReadyForSwitch())
        {
            enterDoubleSupport();
            motionState = DataBus::Stand;
        }
    }

    if (enableNextStep)
        touchDown = false;

    if (!enableNextStep)
    {
        if (legState == DataBus::LSt && FRest[2] >= 200)
        {
            touchDown = true;
            stepNumCur++;
			legState = DataBus::DSt;
        }
        if (legState == DataBus::RSt && FLest[2] >= 200)
        {
            touchDown = true;
            stepNumCur++;
			legState = DataBus::DSt;
        }
    }

    if (phi >= 1)
    {
        phi = 1;
    }
    updateKinematicOutputs();

}

void GaitScheduler::start(){
	start_walk = true;
}

void GaitScheduler::stop()
{
    start_walk = false;
    motionState = DataBus::Stand;
    supportPhase = SupportPhase::DoubleSupport;
    legState = DataBus::DSt;
    legStateNext = OppositeStance(firstleg);
    stanceLeg = firstleg;
    phi = 0.0;
    phaseTime = 0.0;
    isIni = false;
}

bool GaitScheduler::enterSingleSupport()
{
    if (stepNumCur >= stepNumDes)
    {
        phaseTime = std::min(phaseTime, doubleSupportTime);
        return false;
    }
    supportPhase = SupportPhase::SingleSupport;
    phaseTime = 0.0;
    legState = OppositeStance(stanceLeg);
    stanceLeg = legState;
    if (legState == DataBus::LSt)
    {
        swingStartPos_W = fe_r_pos_W;
        stanceStartPos_W = fe_l_pos_W;
        legStateNext = DataBus::RSt;
    }
    else
    {
        swingStartPos_W = fe_l_pos_W;
        stanceStartPos_W = fe_r_pos_W;
        legStateNext = DataBus::LSt;
    }
    return true;
}

void GaitScheduler::enterDoubleSupport()
{
    supportPhase = SupportPhase::DoubleSupport;
    phaseTime = 0.0;
    stepNumCur++;
    legStateNext = OppositeStance(stanceLeg);
    legState = DataBus::DSt;
}

void GaitScheduler::updateKinematicOutputs()
{
    if (stanceLeg == DataBus::LSt)
    {
        posHip_W = hip_r_pos_W;
        posST_W = fe_l_pos_W;
        stanceStartPos_W = fe_l_pos_W;
        theta0 = -3.1415 * 0.5;
        if (supportPhase == SupportPhase::SingleSupport && motionState == DataBus::Walk2Stand)
        {
            legStateNext = DataBus::DSt;
        }
        else
        {
            legStateNext = DataBus::RSt;
        }
    }
    else
    {
        posHip_W = hip_l_pos_W;
        posST_W = fe_r_pos_W;
        stanceStartPos_W = fe_r_pos_W;
        theta0 = 3.1415 * 0.5;
        if (supportPhase == SupportPhase::SingleSupport && motionState == DataBus::Walk2Stand)
        {
            legStateNext = DataBus::DSt;
        }
        else
        {
            legStateNext = DataBus::LSt;
        }
    }

    if (supportPhase == SupportPhase::DoubleSupport && motionState == DataBus::Stand)
    {
        posHip_W = 0.5 * (hip_l_pos_W + hip_r_pos_W);
        posST_W = stanceLeg == DataBus::LSt ? fe_l_pos_W : fe_r_pos_W;
    }
    if (supportPhase == SupportPhase::DoubleSupport)
    {
        swingStartPos_W = stanceLeg == DataBus::LSt ? fe_r_pos_W : fe_l_pos_W;
    }
}

bool GaitScheduler::swingFootReadyForSwitch() const
{
    if (legState == DataBus::LSt)
    {
        const double dz = fe_r_pos_W(2) - fe_l_pos_W(2);
        return dz < 0.03 || Fz_R_m > FzThrehold;
    }
    if (legState == DataBus::RSt)
    {
        const double dz = fe_l_pos_W(2) - fe_r_pos_W(2);
        return dz < 0.03 || Fz_L_m > FzThrehold;
    }
    return true;
}

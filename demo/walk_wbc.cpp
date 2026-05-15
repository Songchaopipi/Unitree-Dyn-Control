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
#include <cstdio>
#include <iostream>
#include <limits>
#include <string>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "data_logger.h"
#include "wbc_priority.h"
#include "wbc_bruce_weighted.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include "StateEst.h"

namespace
{
constexpr int kTotalMotorDof = 29;
constexpr bool kUseStateEstimatorInMujoco = false;
constexpr int kLeftAnklePitchMotorId = 4;
constexpr int kLeftAnkleRollMotorId = 5;
constexpr int kRightAnklePitchMotorId = 10;
constexpr int kRightAnkleRollMotorId = 11;

double totalModelMass(const mjModel *model)
{
    double mass = 0.0;
    for (int i = 0; i < model->nbody; ++i)
    {
        mass += model->body_mass[i];
    }
    return mass;
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

double vectorValueOrZero(const Eigen::VectorXd &value, int index)
{
    return (index >= 0 && index < value.size()) ? value(index) : 0.0;
}
}

// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel* mj_model = mj_loadXML("../models/g1/g1_29_withsensor.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);

//************************
// main function
int main(int argc, const char** argv)
{
    bool enableWalking = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--walk")
        {
            enableWalking = true;
        }
    }

    // ini classes
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    Pin_KinDyn kinDynSolver("../models/g1/g1_29_withsensor.urdf"); // kinematics and dynamics solver
    DataBus RobotState(kinDynSolver.model_nv); // data bus
    WBC_priority WBC_stand(kinDynSolver.model_nv, 18, 22, 1.0, mj_model->opt.timestep);
    WBC_BruceWeighted WBC_bruce(kinDynSolver.model_nv, 1.0, mj_model->opt.timestep);
    GaitScheduler gaitScheduler(0.3, mj_model->opt.timestep); // gait scheduler
    PVT_Ctr pvtCtr(mj_model->opt.timestep,"../common/joint_ctrl_config.json");// PVT joint control
    FootPlacement footPlacement; // foot-placement planner
    JoyStickInterpreter jsInterp(mj_model->opt.timestep); // desired baselink velocity generator
    DataLogger logger("../record/datalog.log"); // data logger
    StateEst StateModule(mj_model->opt.timestep);

    // variables ini
    double stand_legLength = 0.65; // desired COM height above the stance foot for G1
    double  xv_des = 0.1;  // desired velocity in x direction for G1 walking
    const double walkComHeight = 0.65;
    const double nominalFootForce = totalModelMass(mj_model) * 9.81 * 0.5;

    RobotState.width_hips = 0.20;
    footPlacement.kp_vx = 0.03;
    footPlacement.kp_vy = 0.035;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = 0.10;
    footPlacement.legLength = walkComHeight;
    footPlacement.robotMass = totalModelMass(mj_model);
    footPlacement.useAngularMomentumState = true;
    footPlacement.dt = mj_model->opt.timestep;
    int model_nv=kinDynSolver.model_nv;
    const auto g1StandPose = makeG1StandPose();
    mj_interface.setMotorsPosition(g1StandPose);
    mj_interface.dataBusWrite(RobotState);
    const Eigen::Vector3d g1BasePosDes = RobotState.base_pos;
    const double g1YawDes = RobotState.base_rpy(2);
    const double g1BaseHeightDes = RobotState.base_pos(2);
    jsInterp.setIniPos(g1BasePosDes(0), g1BasePosDes(1), g1BaseHeightDes, g1YawDes);

    // ini position and posture for foot-end and hand
    std::vector<double> motors_pos_cur(model_nv-6,0);
    std::vector<double> motors_vel_des(model_nv-6,0);
    std::vector<double> motors_vel_cur(model_nv-6,0);
    std::vector<double> motors_tau_des(model_nv-6,0);
    std::vector<double> motors_tau_cur(model_nv-6,0);
    Eigen::VectorXd qMotorIni = Eigen::Map<const Eigen::VectorXd>(g1StandPose.data(), g1StandPose.size());
    Eigen::VectorXd qIniDes=Eigen::VectorXd::Zero(mj_model->nq,1);
    qIniDes.block(7, 0, model_nv - 6, 1) = qMotorIni;
    WBC_stand.setQini(qIniDes, RobotState.q);
    WBC_bruce.setQini(qIniDes, RobotState.q);

    // register variable name for data logger
    logger.addIterm("simTime", 1);
    logger.addIterm("motors_pos_cur",model_nv-6);
    logger.addIterm("motors_vel_cur",model_nv-6);
    logger.addIterm("rpy",3);
    logger.addIterm("fL",3);
    logger.addIterm("fR",3);
    logger.addIterm("basePos",3);
    logger.addIterm("baseLinVel",3);
    logger.addIterm("baseAcc",3);
    logger.addIterm("baseAngVel",3);
    logger.addIterm("legState", 1);
    logger.addIterm("phi", 1);
    logger.addIterm("swingFinal", 3);
    logger.addIterm("swingCur", 3);
    logger.addIterm("stancePos", 3);
    logger.addIterm("wbcFr", 12);
    logger.addIterm("qpStatus", 1);
    logger.finishItermAdding();

    /// ----------------- sim Loop ---------------
    double simEndTime=30;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    double startShiftTime=2.0;
    double startSteppingTime=3.0;
    double startWalkingTime=3.0;
    Eigen::Vector3d pCoMStandDes = Eigen::Vector3d::Zero();
    bool pCoMStandDesInitialized = false;
    bool gaitStarted = false;
    enum class PipelineMode
    {
        Null,
        Standing,
        Walking
    };
    PipelineMode currentMode = PipelineMode::Null;
    double lastCommandedVx = std::numeric_limits<double>::quiet_NaN();

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false);

    while( !glfwWindowShouldClose(uiController.window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        simstart=mj_data->time;
        while( mj_data->time - simstart < 1.0/60.0 && uiController.runSim) // press "1" to pause and resume, "2" to step the simulation
        {
            mj_step(mj_model, mj_data);

            simTime=mj_data->time;
            printf("-------------%.3f s------------\n",simTime);
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);

            RobotState.motionState = DataBus::Stand;

            // update kinematics and dynamics info
            kinDynSolver.dataBusRead(RobotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.dataBusWrite(RobotState);

            if (kUseStateEstimatorInMujoco && simTime > 1 && StateModule.flag_init)
            {
                std::cout << "init state module" << std::endl;
                StateModule.init(RobotState);
            }

            if (kUseStateEstimatorInMujoco && !StateModule.flag_init)
            {
                StateModule.set(RobotState);
                StateModule.update();
                StateModule.get(RobotState);

                // StateEst updates base velocity/angular velocity. Recompute
                // kinematics/dynamics so planner and WBC consume one coherent
                // current-cycle state, matching the RoMoCo pipeline ordering.
                kinDynSolver.dataBusRead(RobotState);
                kinDynSolver.computeJ_dJ();
                kinDynSolver.computeDyn();
                kinDynSolver.dataBusWrite(RobotState);
            }

            if (!pCoMStandDesInitialized)
            {
                pCoMStandDes = RobotState.pCoM_W;
                WBC_bruce.pCoMDes = pCoMStandDes;
                WBC_bruce.pCoMDesInitialized = true;
                pCoMStandDesInitialized = true;
            }

            if (false)
            {
                StateModule.setF(RobotState);
                StateModule.updateF();
                StateModule.getF(RobotState);
            }

            const bool readyToWalk =
                !enableWalking || simTime >= startWalkingTime ||
                std::abs(RobotState.pCoM_W(1) - pCoMStandDes(1)) >= 0.03;
            const bool requestWalkingMode = enableWalking && simTime >= startShiftTime && readyToWalk;
            const PipelineMode requestedMode = requestWalkingMode ? PipelineMode::Walking : PipelineMode::Standing;
            if (requestedMode != currentMode)
            {
                currentMode = requestedMode;
                if (currentMode == PipelineMode::Walking)
                {
                    gaitScheduler.stop();
                    gaitStarted = false;
                    WBC_bruce.pCoMDesInitialized = false;
                }
                else
                {
                    gaitScheduler.stop();
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), g1BaseHeightDes, RobotState.base_rpy(2));
                }
            }

            if (currentMode == PipelineMode::Walking) {
                jsInterp.setWzDesLPara(0, 1);
                if (!std::isfinite(lastCommandedVx) || std::abs(lastCommandedVx - xv_des) > 1e-9)
                {
                    jsInterp.setVxDesLPara(xv_des, 2.0);
                    lastCommandedVx = xv_des;
                }
            } else {
                lastCommandedVx = std::numeric_limits<double>::quiet_NaN();
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), g1BaseHeightDes, RobotState.base_rpy(2));
                gaitStarted = false;
            }

            const bool plannerActive = currentMode == PipelineMode::Walking && simTime >= startSteppingTime;
            if (plannerActive) {
                jsInterp.step();
                jsInterp.dataBusWrite(RobotState); // only pos x, pos y, pos_z, theta z, vel x, vel y , omega z are rewrote.
                RobotState.motionState = DataBus::Walk;
                if (!gaitStarted)
                {
                    gaitScheduler.start();
                    gaitStarted = true;
                }

                gaitScheduler.dataBusRead(RobotState);
                gaitScheduler.step();
                gaitScheduler.dataBusWrite(RobotState);

                footPlacement.dataBusRead(RobotState);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(RobotState);
            }
            else {
                RobotState.motionState = DataBus::Stand;
                RobotState.legState = DataBus::DSt;
                RobotState.walk_stance_leg = DataBus::RSt;
                RobotState.legStateNext = DataBus::LSt;
                RobotState.phi = 0.0;
                RobotState.walk_is_double_support = true;
                RobotState.walk_left_contact = true;
                RobotState.walk_right_contact = true;
                RobotState.walk_time_to_impact = 0.0;
                RobotState.stance_fe_pos_cur_W = RobotState.fe_r_pos_W;
                RobotState.stance_fe_rot_cur_W = RobotState.fe_r_rot_W;
            }

            // ------------- Contact/reference pipeline ------------
            RobotState.des_ddq = Eigen::VectorXd::Zero(mj_model->nv);
            RobotState.des_dq = Eigen::VectorXd::Zero(mj_model->nv);
            RobotState.des_delta_q = Eigen::VectorXd::Zero(mj_model->nv);
            const double firstStanceYOffset = 0.04;
            const double shiftDen = std::max(1e-3, startSteppingTime - startShiftTime);
            const double shiftPhase = std::clamp((simTime - startShiftTime) / shiftDen, 0.0, 1.0);
            const double shiftBlend = shiftPhase * shiftPhase * (3.0 - 2.0 * shiftPhase);
            if (RobotState.motionState == DataBus::Stand)
            {
                RobotState.base_rpy_des << 0.0, 0.0, g1YawDes;
                RobotState.base_pos_des = g1BasePosDes;
                WBC_bruce.pCoMDes = pCoMStandDes;
                if (enableWalking)
                {
                    const double footZ = 0.5 * (RobotState.fe_l_pos_W(2) + RobotState.fe_r_pos_W(2));
                    const double walkReadyCoMZ = footZ + walkComHeight;
                    WBC_bruce.pCoMDes(1) += firstStanceYOffset * shiftBlend;
                    WBC_bruce.pCoMDes(2) = (1.0 - shiftBlend) * pCoMStandDes(2) + shiftBlend * walkReadyCoMZ;
                    RobotState.base_pos_des(2) = g1BaseHeightDes + WBC_bruce.pCoMDes(2) - pCoMStandDes(2);
                }
                RobotState.walk_target_yaw = RobotState.base_rpy_des(2);
                RobotState.walk_target_yaw_rate = 0.0;
                const double supportZ = 0.5 * (RobotState.fe_l_pos_W.z() + RobotState.fe_r_pos_W.z());
                RobotState.walk_target_com_height = WBC_bruce.pCoMDes.z() - supportZ;
                RobotState.walk_yd(0) = RobotState.walk_target_com_height;
                RobotState.Fr_ff << 0, 0, nominalFootForce, 0, 0, 0,
                                    0, 0, nominalFootForce, 0, 0, 0;
            }
            else
            {
                RobotState.base_rpy_des << 0.0, 0.0, RobotState.walk_target_yaw;
                RobotState.base_pos_des = g1BasePosDes;
                const double targetCoMHeight =
                    (RobotState.walk_yd.size() >= 1) ? RobotState.walk_yd(0) : walkComHeight;
                const double targetCoMZ = RobotState.stance_fe_pos_cur_W.z() + targetCoMHeight;
                RobotState.base_pos_des(2) = g1BaseHeightDes + targetCoMZ - pCoMStandDes(2);
                RobotState.walk_target_com_height = targetCoMHeight;
                if (RobotState.walk_is_double_support)
                {
                    RobotState.Fr_ff << 0, 0, nominalFootForce, 0, 0, 0,
                                        0, 0, nominalFootForce, 0, 0, 0;
                }
                else if (RobotState.walk_left_contact)
                {
                    RobotState.Fr_ff << 0, 0, 2.0 * nominalFootForce, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0;
                }
                else
                {
                    RobotState.Fr_ff << 0, 0, 0, 0, 0, 0,
                                        0, 0, 2.0 * nominalFootForce, 0, 0, 0;
                }
            }

            if (currentMode == PipelineMode::Walking) {
                RobotState.des_delta_q.block<2, 1>(0, 0) << jsInterp.vx_W * mj_model->opt.timestep, jsInterp.vy_W * mj_model->opt.timestep;
                RobotState.des_delta_q(5) = jsInterp.wz_L * mj_model->opt.timestep;
                RobotState.des_dq.block<2, 1>(0, 0) << jsInterp.vx_W, jsInterp.vy_W;
                RobotState.des_dq(5) = jsInterp.wz_L;

                double k = 5;
                RobotState.des_ddq.block<2, 1>(0, 0) << k * (jsInterp.vx_W - RobotState.dq(0)), k * (jsInterp.vy_W -
                                                                                                     RobotState.dq(1));
                RobotState.des_ddq(5) = k * (jsInterp.wz_L - RobotState.dq(5));
            }


            // WBC Calculation
            if (RobotState.motionState == DataBus::Stand)
            {
                WBC_stand.pCoMDes = WBC_bruce.pCoMDes;
                WBC_stand.pCoMDesInitialized = true;
                WBC_stand.dataBusRead(RobotState);
                WBC_stand.computeDdq(kinDynSolver);
                WBC_stand.computeTau();
                WBC_stand.dataBusWrite(RobotState);
            }
            else
            {
                WBC_bruce.dataBusRead(RobotState);
                WBC_bruce.computeDdq(kinDynSolver);
                WBC_bruce.computeTau();
                WBC_bruce.dataBusWrite(RobotState);
            }

            // get the final joint command
            Eigen::VectorXd pos_des=kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
            RobotState.motors_pos_des = eigen2std(pos_des.block(7,0, model_nv-6,1));
            RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final.block(6,0, model_nv-6,1));
            RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
            std::vector<bool> activeLegMotor(std::min(12, model_nv - 6), false);
            if (RobotState.motionState == DataBus::Walk)
            {
                for (int motorId : RobotState.wbc_active_motor_ids)
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
                        RobotState.motors_pos_des[i] = RobotState.motors_pos_cur[i];
                        RobotState.motors_vel_des[i] = 0.0;
                        RobotState.motors_tor_des[i] = 0.0;
                    }
                }
            }
            for (int i = 12; i < model_nv - 6 && i < static_cast<int>(g1StandPose.size()); ++i)
            {
                RobotState.motors_pos_des[i] = g1StandPose[i];
                RobotState.motors_vel_des[i] = 0.0;
                RobotState.motors_tor_des[i] = 0.0;
            }

            pvtCtr.dataBusRead(RobotState);
            {
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);
            if (RobotState.motionState == DataBus::Walk)
            {
                for (int i = 0; i < static_cast<int>(activeLegMotor.size()); ++i)
                {
                    if (!activeLegMotor[i])
                    {
                        RobotState.motors_tor_out[i] = 0.0;
                        RobotState.motors_tor_cur[i] = 0.0;
                    }
                }
            }

            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            logger.startNewLine();
            logger.recItermData("simTime", simTime);
            logger.recItermData("motors_pos_cur",RobotState.motors_pos_cur);
            logger.recItermData("motors_vel_cur",RobotState.motors_vel_cur);
            logger.recItermData("rpy",RobotState.rpy);
            logger.recItermData("fL",RobotState.fL);
            logger.recItermData("fR",RobotState.fR);
            logger.recItermData("basePos",RobotState.basePos);
            logger.recItermData("baseLinVel",RobotState.baseLinVel);
            logger.recItermData("baseAcc",RobotState.baseAcc);
            logger.recItermData("baseAngVel",RobotState.baseAngVel);
            logger.recItermData("legState", static_cast<double>(RobotState.legState));
            logger.recItermData("phi", RobotState.phi);
            logger.recItermData("swingFinal", RobotState.swingDesPosFinal_W);
            logger.recItermData("swingCur", RobotState.swingDesPosCur_W);
            logger.recItermData("stancePos", RobotState.stance_fe_pos_cur_W);
            logger.recItermData("wbcFr", RobotState.wbc_FrRes);
            logger.recItermData("qpStatus", static_cast<double>(RobotState.qp_status));
            logger.finishLine();

            printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
            printf("gps=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);
            printf("vel=[%.5f, %.5f, %.5f]\n", RobotState.baseLinVel[0], RobotState.baseLinVel[1], RobotState.baseLinVel[2]);
            printf("wbcAnkleTau[Nm]: L_pitch=%.5f, L_roll=%.5f, R_pitch=%.5f, R_roll=%.5f\n",
                   vectorValueOrZero(RobotState.wbc_tauJointRes, kLeftAnklePitchMotorId),
                   vectorValueOrZero(RobotState.wbc_tauJointRes, kLeftAnkleRollMotorId),
                   vectorValueOrZero(RobotState.wbc_tauJointRes, kRightAnklePitchMotorId),
                   vectorValueOrZero(RobotState.wbc_tauJointRes, kRightAnkleRollMotorId));
            printf("wbcDesiredGRF_Fr_ff[L/R:Fx,Fy,Fz,Mx,My,Mz]=[%.5f, %.5f, %.5f, %.5f, %.5f, %.5f] / [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]\n",
                   vectorValueOrZero(RobotState.Fr_ff, 0),
                   vectorValueOrZero(RobotState.Fr_ff, 1),
                   vectorValueOrZero(RobotState.Fr_ff, 2),
                   vectorValueOrZero(RobotState.Fr_ff, 3),
                   vectorValueOrZero(RobotState.Fr_ff, 4),
                   vectorValueOrZero(RobotState.Fr_ff, 5),
                   vectorValueOrZero(RobotState.Fr_ff, 6),
                   vectorValueOrZero(RobotState.Fr_ff, 7),
                   vectorValueOrZero(RobotState.Fr_ff, 8),
                   vectorValueOrZero(RobotState.Fr_ff, 9),
                   vectorValueOrZero(RobotState.Fr_ff, 10),
                   vectorValueOrZero(RobotState.Fr_ff, 11));
            printf("wbcSolvedGRF[L/R:Fx,Fy,Fz,Mx,My,Mz]=[%.5f, %.5f, %.5f, %.5f, %.5f, %.5f] / [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]\n",
                   vectorValueOrZero(RobotState.wbc_FrRes, 0),
                   vectorValueOrZero(RobotState.wbc_FrRes, 1),
                   vectorValueOrZero(RobotState.wbc_FrRes, 2),
                   vectorValueOrZero(RobotState.wbc_FrRes, 3),
                   vectorValueOrZero(RobotState.wbc_FrRes, 4),
                   vectorValueOrZero(RobotState.wbc_FrRes, 5),
                   vectorValueOrZero(RobotState.wbc_FrRes, 6),
                   vectorValueOrZero(RobotState.wbc_FrRes, 7),
                   vectorValueOrZero(RobotState.wbc_FrRes, 8),
                   vectorValueOrZero(RobotState.wbc_FrRes, 9),
                   vectorValueOrZero(RobotState.wbc_FrRes, 10),
                   vectorValueOrZero(RobotState.wbc_FrRes, 11));
        }

        if (mj_data->time>=simEndTime)
        {
            break;
        }

        uiController.updateScene();
    }

//    // free visualization storage
    uiController.Close();

    return 0;
}

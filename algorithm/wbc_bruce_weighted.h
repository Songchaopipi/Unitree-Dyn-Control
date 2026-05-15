/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#pragma once

#include "qpOASES.hpp"
#include <Eigen/Dense>
#include <vector>
#include "data_bus.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"

class WBC_BruceWeighted
{
public:
    int model_nv;
    double miu{1.0};
    double f_z_low{100.0}, f_z_upp{1400.0};
    double foot_l_front{0.1}, foot_l_back{-0.04}, foot_half_width{0.025}, foot_yaw_friction{0.046};
    Eigen::Vector3d pCoMDes;
    bool pCoMDesInitialized{false};
    double desiredVx{0.0};
    double desiredVy{0.0};
    double targetYaw{0.0};
    double targetYawRate{0.0};
    double targetComHeight{0.65};

    WBC_BruceWeighted(int model_nv_In, double miu_In, double dt);

    void setQini(const Eigen::VectorXd &qIniDes, const Eigen::VectorXd &qIniCur);
    void dataBusRead(const DataBus &robotState);
    void computeDdq(Pin_KinDyn &pinKinDynIn);
    void computeTau();
    void dataBusWrite(DataBus &robotState);

private:
    struct AccTask
    {
        Eigen::MatrixXd J;
        Eigen::VectorXd dJdq;
        Eigen::VectorXd desiredAcc;
        Eigen::VectorXd weight;
    };

    double timeStep{0.001};
    int qpStatus{0};
    qpOASES::int_t nWSR{100};
    qpOASES::real_t cpu_time{0.001};

    DataBus::LegState legStateCur{DataBus::DSt};
    DataBus::LegState stanceLegCur{DataBus::LSt};
    DataBus::MotionState motionStateCur{DataBus::Stand};
    bool isDoubleSupportCur{true};
    bool leftContactCur{true};
    bool rightContactCur{true};

    Eigen::VectorXd q, dq;
    Eigen::VectorXd qIniDes, qIniCur;
    Eigen::VectorXd des_ddq, des_dq, des_delta_q;
    Eigen::VectorXd Fr_ff;
    Eigen::VectorXd delta_q_final_kin, dq_final_kin, ddq_final_kin;
    Eigen::VectorXd tauJointRes, eigen_ddq_Opt, eigen_fr_Opt;

    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non;
    Eigen::MatrixXd J_base, dJ_base, Jcom;
    Eigen::Vector3d dJcom;
    Eigen::MatrixXd J_l, J_r, dJ_l, dJ_r;
    Eigen::MatrixXd Jc, dJc, Jsw, dJsw;
    Eigen::MatrixXd Jfe, dJfe, Jfe_foot, dJfe_foot, Jc_foot;

    Eigen::Vector3d base_pos, base_pos_des;
    Eigen::Vector3d base_rpy_cur, base_rpy_des;
    Eigen::Matrix3d base_rot;
    Eigen::Vector3d pCoMCur;
    Eigen::Vector3d pCoMRefWorld{Eigen::Vector3d::Zero()};
    Eigen::Vector3d vCoMRefWorld{Eigen::Vector3d::Zero()};
    Eigen::Vector3d aCoMRefWorld{Eigen::Vector3d::Zero()};
    Eigen::Vector3d centroidalComPosDes{Eigen::Vector3d::Zero()};
    Eigen::Vector3d centroidalComVelDes{Eigen::Vector3d::Zero()};
    Eigen::Vector3d centroidalComAccDes{Eigen::Vector3d::Zero()};
    bool comReferenceInitialized{false};
    bool centroidalNmpcEnabled{false};
    Eigen::Vector3d fe_l_pos_cur_W, fe_r_pos_cur_W, fe_pos_sw_W;
    Eigen::Matrix3d fe_l_rot_cur_W, fe_r_rot_cur_W, fe_rot_sw_W;
    Eigen::Vector3d swing_fe_pos_des_W, swing_fe_vel_des_W, swing_fe_acc_des_W, swing_fe_rpy_des_W;
    Eigen::Vector3d stance_fe_pos_cur_W, stanceDesPos_W;
    Eigen::VectorXd walk_yd, walk_dyd, walk_d2yd;
    std::vector<int> activeMotorIds;

    bool isLeftStance() const;
    bool isDoubleSupport() const;
    void updateActiveMotorIds();
    bool isMotorActive(int motorId) const;
    void updateComReference();
    void buildContact(Eigen::MatrixXd &Jh, Eigen::VectorXd &dJhdq, Eigen::VectorXd &forceRef) const;
    void buildBruceAccelerationTasks(std::vector<AccTask> &tasks) const;
    void buildBruceIkTasks(Eigen::MatrixXd &J, Eigen::VectorXd &err, Eigen::VectorXd &dx, Eigen::VectorXd &weight) const;
    Eigen::VectorXd solveDampedLeastSquares(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, double damping) const;
    void copyEigenToReal(qpOASES::real_t *target, const Eigen::MatrixXd &source) const;
    void copyEigenToReal(qpOASES::real_t *target, const Eigen::VectorXd &source) const;
};

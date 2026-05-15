/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "MJ_interface.h"

MJ_Interface::MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn)
{
    mj_model = mj_modelIn;
    mj_data = mj_dataIn;
    timeStep = mj_model->opt.timestep;
    jointNum = JointName.size();
    jntId_qpos.assign(jointNum, 0);
    jntId_qvel.assign(jointNum, 0);
    jntId_dctl.assign(jointNum, 0);
    motor_pos.assign(jointNum, 0);
    motor_vel.assign(jointNum, 0);
    motor_pos_Old.assign(jointNum, 0);
    for (int i = 0; i < jointNum; i++)
    {
        int tmpId = mj_name2id(mj_model, mjOBJ_JOINT, JointName[i].c_str());
        if (tmpId == -1)
        {
            std::cerr << JointName[i] << " not found in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_qpos[i] = mj_model->jnt_qposadr[tmpId];
        jntId_qvel[i] = mj_model->jnt_dofadr[tmpId];
        std::string motorName = JointName[i];
        tmpId = mj_name2id(mj_model, mjOBJ_ACTUATOR, motorName.c_str());
        if (tmpId == -1)
        {
            std::cerr << motorName << " not found in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_dctl[i] = tmpId;
    }
    //    int adr = m->sensor_adr[sensorId];
    //    int dim = m->sensor_dim[sensorId];
    //    mjtNum sensor_data[dim];
    //    mju_copy(sensor_data, &d->sensordata[adr], dim);
    baseBodyId = mj_name2id(mj_model, mjOBJ_BODY, baseName.c_str());
    floorGeomId = mj_name2id(mj_model, mjOBJ_GEOM, "floor");
    leftFootBodyId = mj_name2id(mj_model, mjOBJ_BODY, "left_ankle_roll_link");
    rightFootBodyId = mj_name2id(mj_model, mjOBJ_BODY, "right_ankle_roll_link");
    orientataionSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, orientationSensorName.c_str());
    velSensorId = velSensorName.empty() ? -1 : mj_name2id(mj_model, mjOBJ_SENSOR, velSensorName.c_str());
    gyroSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, gyroSensorName.c_str());
    accSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, accSensorName.c_str());
    if (baseBodyId == -1 || floorGeomId == -1 || leftFootBodyId == -1 || rightFootBodyId == -1 ||
        orientataionSensorId == -1 || gyroSensorId == -1 || accSensorId == -1)
    {
        std::cerr << "G1 MuJoCo base, foot contact body, floor geom or IMU sensors were not found in the XML file!" << std::endl;
        std::terminate();
    }
}

void MJ_Interface::updateSensorValues()
{
    for (int i = 0; i < jointNum; i++)
    {
        motor_pos_Old[i] = motor_pos[i];
        motor_pos[i] = mj_data->qpos[jntId_qpos[i]];
        motor_vel[i] = mj_data->qvel[jntId_qvel[i]];
    }
    for (int i = 0; i < 4; i++)
        baseQuat[i] = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId] + i];
    double tmp = baseQuat[0];
    baseQuat[0] = baseQuat[1];
    baseQuat[1] = baseQuat[2];
    baseQuat[2] = baseQuat[3];
    baseQuat[3] = tmp;

    rpy[0] = atan2(2 * (baseQuat[3] * baseQuat[0] + baseQuat[1] * baseQuat[2]), 1 - 2 * (baseQuat[0] * baseQuat[0] + baseQuat[1] * baseQuat[1]));
    rpy[1] = asin(2 * (baseQuat[3] * baseQuat[1] - baseQuat[0] * baseQuat[2]));
    rpy[2] = atan2(2 * (baseQuat[3] * baseQuat[2] + baseQuat[0] * baseQuat[1]), 1 - 2 * (baseQuat[1] * baseQuat[1] + baseQuat[2] * baseQuat[2]));

    if ((rpy[2] - yaw_simgle) > 3.1415926*0.5)
        yaw_N -= 1.0;
    else if ((rpy[2] - yaw_simgle) < -3.1415926*0.5)
        yaw_N += 1.0;

    yaw_simgle=rpy[2];
    rpy[2]=yaw_simgle + yaw_N*2.0*3.1415926;


    for (int i = 0; i < 3; i++)
    {
        double posOld = basePos[i];
        basePos[i] = mj_data->xpos[3 * baseBodyId + i];
        baseAcc[i] = mj_data->sensordata[mj_model->sensor_adr[accSensorId] + i];
        baseAngVel[i] = mj_data->sensordata[mj_model->sensor_adr[gyroSensorId] + i];
        baseLinVel[i] = (basePos[i] - posOld) / (mj_model->opt.timestep);
        f3d[i][0] = 0.0;
        f3d[i][1] = 0.0;
    }

    for (int i = 0; i < mj_data->ncon; ++i)
    {
        const mjContact &contact = mj_data->contact[i];
        int footCol = -1;
        int sign = 1;
        for (int side = 0; side < 2; ++side)
        {
            const int geomId = contact.geom[side];
            const int otherGeomId = contact.geom[1 - side];
            const int bodyId = mj_model->geom_bodyid[geomId];
            if (otherGeomId == floorGeomId && bodyId == leftFootBodyId)
            {
                footCol = 0;
                sign = (side == 0) ? 1 : -1;
            }
            else if (otherGeomId == floorGeomId && bodyId == rightFootBodyId)
            {
                footCol = 1;
                sign = (side == 0) ? 1 : -1;
            }
        }
        if (footCol < 0)
        {
            continue;
        }

        mjtNum contactForceLocal[6];
        mj_contactForce(mj_model, mj_data, i, contactForceLocal);
        Eigen::Vector3d forceLocal(contactForceLocal[0], contactForceLocal[1], contactForceLocal[2]);
        Eigen::Matrix3d contactFrame;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                contactFrame(r, c) = contact.frame[3 * r + c];
            }
        }
        const Eigen::Vector3d forceWorld = -sign * contactFrame.transpose() * forceLocal;
        for (int k = 0; k < 3; ++k)
        {
            f3d[k][footCol] += forceWorld(k);
        }
    }
}

void MJ_Interface::setMotorsTorque(std::vector<double> &tauIn)
{
    for (int i = 0; i < jointNum; i++)
        mj_data->ctrl[jntId_dctl[i]] = tauIn.at(i);
}

void MJ_Interface::setMotorsPosition(const std::vector<double> &qIn)
{
    for (int i = 0; i < jointNum && i < static_cast<int>(qIn.size()); i++)
    {
        mj_data->qpos[jntId_qpos[i]] = qIn.at(i);
    }
    for (int i = 0; i < mj_model->nv; ++i)
    {
        mj_data->qvel[i] = 0.0;
    }
    mj_forward(mj_model, mj_data);
    updateSensorValues();
}

void MJ_Interface::dataBusWrite(DataBus &busIn)
{
    busIn.motors_pos_cur = motor_pos;
    busIn.motors_vel_cur = motor_vel;
    busIn.rpy[0] = rpy[0];
    busIn.rpy[1] = rpy[1];
    busIn.rpy[2] = rpy[2];
    busIn.fL[0] = f3d[0][0];
    busIn.fL[1] = f3d[1][0];
    busIn.fL[2] = f3d[2][0];
    busIn.fR[0] = f3d[0][1];
    busIn.fR[1] = f3d[1][1];
    busIn.fR[2] = f3d[2][1];
    busIn.basePos[0] = basePos[0];
    busIn.basePos[1] = basePos[1];
    busIn.basePos[2] = basePos[2];
    busIn.baseLinVel[0] = baseLinVel[0];
    busIn.baseLinVel[1] = baseLinVel[1];
    busIn.baseLinVel[2] = baseLinVel[2];
    busIn.baseAcc[0] = baseAcc[0];
    busIn.baseAcc[1] = baseAcc[1];
    busIn.baseAcc[2] = baseAcc[2];
    busIn.baseAngVel[0] = baseAngVel[0];
    busIn.baseAngVel[1] = baseAngVel[1];
    busIn.baseAngVel[2] = baseAngVel[2];
    busIn.updateQ();
}

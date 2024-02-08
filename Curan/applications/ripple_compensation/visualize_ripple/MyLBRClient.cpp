/**
DISCLAIMER OF WARRANTY
The Software is provided "AS IS" and "WITH ALL FAULTS,"
without warranty of any kind, including without limitation the warranties
of merchantability, fitness for a particular purpose and non-infringement.
KUKA makes no warranty that the Software is free of defects or is suitable
for any particular purpose. In no event shall KUKA be responsible for loss
or damages arising from the installation or use of the Software,
including but not limited to any indirect, punitive, special, incidental
or consequential damages of any character including, without limitation,
damages for loss of goodwill, work stoppage, computer failure or malfunction,
or any and all other commercial damages or losses.
The entire risk to the quality and performance of the Software is not borne by KUKA.
Should the Software prove defective, KUKA is not liable for the entire cost
of any service and repair.
COPYRIGHT
All Rights Reserved
Copyright (C)  2014-2015
KUKA Roboter GmbH
Augsburg, Germany
This material is the exclusive property of KUKA Roboter GmbH and must be returned
to KUKA Roboter GmbH immediately upon request.
This material and the information illustrated or contained herein may not be used,
reproduced, stored in a retrieval system, or transmitted in whole
or in part in any way - electronic, mechanical, photocopying, recording,
or otherwise, without the prior written consent of KUKA Roboter GmbH.
\file
\version {1.9}
*/

#include "MyLBRClient.h"
#include <chrono>
#include <fstream>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#ifndef NCoef
#define NCoef 1
#endif

//******************************************************************************
MyLBRClient::MyLBRClient(std::shared_ptr<SharedState> in_shared_state) : shared_state{in_shared_state} {

    // Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
    kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here

    robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
    iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

    // Initialize Limits
    myIIWALimits = RobotLimits();
    myIIWALimits.qMax[0] = 163 * M_PI / 180;
    myIIWALimits.qMax[1] = 113 * M_PI / 180;
    myIIWALimits.qMax[2] = 163 * M_PI / 180;
    myIIWALimits.qMax[3] = 115 * M_PI / 180;
    myIIWALimits.qMax[4] = 160 * M_PI / 180;
    myIIWALimits.qMax[5] = 110 * M_PI / 180;
    myIIWALimits.qMax[6] = 165 * M_PI / 180;

    myIIWALimits.qMin[0] = -163 * M_PI / 180;
    myIIWALimits.qMin[1] = -113 * M_PI / 180;
    myIIWALimits.qMin[2] = -163 * M_PI / 180;
    myIIWALimits.qMin[3] = -115 * M_PI / 180;
    myIIWALimits.qMin[4] = -160 * M_PI / 180;
    myIIWALimits.qMin[5] = -110 * M_PI / 180;
    myIIWALimits.qMin[6] = -165 * M_PI / 180;

    myIIWALimits.qDotMax[0] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[1] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[2] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[3] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[4] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[5] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[6] = 185 * M_PI / 180;

    myIIWALimits.qDotMin[0] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[1] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[2] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[3] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[4] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[5] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[6] = -185 * M_PI / 180;

    myIIWALimits.qDotDotMax[0] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[1] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[2] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[3] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[4] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[5] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[6] = 340 * M_PI / 180;

    myIIWALimits.qDotDotMin[0] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[1] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[2] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[3] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[4] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[5] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[6] = -340 * M_PI / 180;

    // Attach tool
    toolMass = 0.0;                                                                     // No tool for now
    toolCOM = Vector3d::Zero(3, 1);
    toolInertia = Matrix3d::Zero(3, 3);
    myTool = new ToolData(toolMass, toolCOM, toolInertia);
    //-(change_1 )myLBR->attachToolToRobotModel(myTool);
    robot->attachToolToRobotModel(myTool);

    _qInitial[0] = 0.0 * M_PI / 180;
    _qInitial[1] = -90.0 * M_PI / 180;
    _qInitial[2] = 0.0 * M_PI / 180;
    _qInitial[3] = 0.0 * M_PI / 180;
    _qInitial[4] = 0.0 * M_PI / 180;
    _qInitial[5] = 0.0 * M_PI / 180;
    _qInitial[6] = 0.0 * M_PI / 180;

    for (int i = 0; i < LBR_N_JOINTS; i++) {
        _qCurr[i] = _qInitial[i];
        _qOld[i] = _qInitial[i];
        _qApplied[i] = _qInitial[i];
        _torques[i] = 0.0;
        _measured_torques[i] = 0.0;
    }

    measured_torque = VectorNd::Zero(LBR_N_JOINTS, 1);
    tau_command = VectorNd::Zero(LBR_N_JOINTS, 1);
    tau_command_filtered = VectorNd::Zero(LBR_N_JOINTS, 1);
    tau_previous = VectorNd::Zero(LBR_N_JOINTS, 1);
    tau_prev_prev = VectorNd::Zero(LBR_N_JOINTS, 1);

    // Positions and orientations and Jacobian
    p_0_cur = Vector3d::Zero(3, 1);
    R_0_7 = Matrix3d::Zero(3, 3);
    J = MatrixNd::Zero(6, LBR_N_JOINTS);
}

//******************************************************************************
MyLBRClient::~MyLBRClient() {
}

//******************************************************************************
void MyLBRClient::onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState) {
    LBRClient::onStateChange(oldState, newState);
    // react on state change events
    switch (newState)
    {
    case KUKA::FRI::MONITORING_WAIT:
    {
        break;
    }
    case KUKA::FRI::MONITORING_READY:
    {
        sampleTime = robotState().getSampleTime();
        break;
    }
    case KUKA::FRI::COMMANDING_WAIT:
    {
        break;
    }
    case KUKA::FRI::COMMANDING_ACTIVE:
    {
        break;
    }
    default:
    {
        break;
    }
    }
}

//******************************************************************************
void MyLBRClient::monitor() {

    // Copied from FRIClient.cpp
    robotCommand().setJointPosition(robotState().getCommandedJointPosition());

    // Copy measured joint positions (radians) to _qcurr, which is a double
    memcpy(_qCurr, robotState().getMeasuredJointPosition(), LBR_N_JOINTS * sizeof(double));
    shared_state->robot_state.store(robotState());
    shared_state->is_initialized.store(true);

}
//******************************************************************************

void MyLBRClient::waitForCommand()
{
    // If we want to command torques, we have to command them all the time; even in
    // waitForCommand(). This has to be done due to consistency checks. In this state it is
    // only necessary, that some torque vlaues are sent. The LBR does not take the
    // specific value into account.

    if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
        robotCommand().setTorque(_torques);
        robotCommand().setJointPosition(robotState().getIpoJointPosition());            // Just overlaying same position
    }
    shared_state->robot_state.store(robotState());
    shared_state->is_initialized.store(true);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//******************************************************************************
void MyLBRClient::command() {
    // Get robot measurements
    memcpy(_qOld, _qCurr, LBR_N_JOINTS * sizeof(double));
    memcpy(_qCurr, robotState().getMeasuredJointPosition(), LBR_N_JOINTS * sizeof(double));
    memcpy(_measured_torques, robotState().getMeasuredTorque(), LBR_N_JOINTS * sizeof(double));

    static bool copy_current_vals_into_qapplied = true;
    if(copy_current_vals_into_qapplied)
        for (int i = 0; i < LBR_N_JOINTS; i++) 
            _qApplied[i] = _qCurr[i]; 
    copy_current_vals_into_qapplied = false;

    shared_state->robot_state.store(robotState());
    shared_state->is_initialized.store(true);
    for (int i = 0; i < LBR_N_JOINTS; i++) {
        iiwa->q[i] = _qCurr[i];
        measured_torque[i] = _measured_torques[i];
    }
    for (int i = 0; i < LBR_N_JOINTS; i++) {
        iiwa->qDot[i] = (_qCurr[i] - _qOld[i]) / sampleTime;
    }
    robot->getMassMatrix(iiwa->M, iiwa->q);
    iiwa->M(6, 6) = 45 * iiwa->M(6, 6);                                       // Correct mass of last body to avoid large accelerations
    iiwa->Minv = iiwa->M.inverse();
    robot->getCoriolisAndGravityVector(iiwa->c, iiwa->g, iiwa->q, iiwa->qDot);
    robot->getWorldCoordinates(p_0_cur, iiwa->q, pointPosition, 7);              // 3x1 position of flange (body = 7), expressed in base coordinates
    robot->getRotationMatrix(R_0_7, iiwa->q, LBR_N_JOINTS);                                // 3x3 rotation matrix of flange, expressed in base coordinates
    // Limit torques to stop at the robot's joint limits.
	//VectorNd SJSTorque = addConstraints(torqueCommand, 0.005);
    //for now we use a damping torque but we would like to use the state-derivative torque
    Eigen::VectorXd torqueCommand = Eigen::VectorXd::Zero(LBR_N_JOINTS,1);
    static bool positive = true;
    double last_joint_torque = (positive) ? 3 : -3 ;
    torqueCommand[LBR_N_JOINTS-1] = last_joint_torque;
    VectorNd SJSTorque = addConstraints(torqueCommand, 0.005); 

    if(sgn(SJSTorque[LBR_N_JOINTS-1])!=sgn(torqueCommand[LBR_N_JOINTS-1])) // if the sign is different we need to change the direction of the actuation;
        positive = !positive;
    
    for (int i = 0; i < LBR_N_JOINTS; i++) 
        _torques[i] = SJSTorque[i];

    _qApplied[LBR_N_JOINTS-1] = _qCurr[LBR_N_JOINTS-1]; 

    if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
        robotCommand().setJointPosition(_qApplied);
        robotCommand().setTorque(_torques);
    }

    tau_prev_prev = tau_previous;
    tau_previous = tau_command;

    currentTime = currentTime + sampleTime;
}

VectorNd MyLBRClient::addConstraints(const VectorNd& tauStack, double dt)
{
    VectorNd dt2 = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd dtvar = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd qDownBar = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd qTopBar = VectorNd::Zero(LBR_N_JOINTS, 1);

    VectorNd qDotMaxFromQ = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd qDotMinFromQ = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd qDotMaxFormQDotDot = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd qDotMinFormQDotDot = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd vMaxVector = Vector3d::Zero(3);
    VectorNd vMinVector = Vector3d::Zero(3);
    VectorNd qDotMaxFinal = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd qDotMinFinal = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd aMaxqDot = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd aMinqDot = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd aMaxQ = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd aMinQ = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd aMaxVector = Vector3d::Zero(3);
    VectorNd aMinVector = Vector3d::Zero(3);
    VectorNd qDotDotMaxFinal = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd qDotDotMinFinal = VectorNd::Zero(LBR_N_JOINTS, 1);
    MatrixNd Iden = MatrixNd::Identity(LBR_N_JOINTS, LBR_N_JOINTS);
    VectorNd TauBar = VectorNd::Zero(LBR_N_JOINTS, 1);
    VectorNd qDotDotGot = VectorNd::Zero(LBR_N_JOINTS, 1);
    MatrixNd Js = MatrixNd::Zero(3, LBR_N_JOINTS);

    double lowestdtFactor = 10;

    qDownBar = iiwa->q - myIIWALimits.qMin;
    qTopBar = myIIWALimits.qMax - iiwa->q;
    dtvar[0] = 3 * dt;
    dtvar[1] = 3 * dt;
    dtvar[2] = 2 * dt;
    dtvar[3] = 3 * dt;
    dtvar[4] = dt;
    dtvar[5] = dt;
    dtvar[6] = dt;

    for (int i = 0; i < LBR_N_JOINTS; i++)
    {
        //dt2[i] = (lowestdtFactor + (sqrt(lowestdtFactor)*sqrt(10*180/M_PI)))*dt;
        dt2[i] = dtvar[i];
        if (qDownBar[i] < 10 * M_PI / 180)
        {

            if (qDownBar[i] < 0)
                qDownBar[i] = 0;

            dt2[i] = (lowestdtFactor + (sqrt(lowestdtFactor) * sqrt(qDownBar[i] * 180 / M_PI))) * dtvar[i];

            if (dt2[i] < lowestdtFactor * dtvar[i])
                dt2[i] = lowestdtFactor * dtvar[i];
        }
        if (qTopBar[i] < 10 * M_PI / 180)
        {

            if (qTopBar[i] < 0)
                qTopBar[i] = 0;

            dt2[i] = (lowestdtFactor + (sqrt(lowestdtFactor) * sqrt(qTopBar[i] * 180 / M_PI))) * dtvar[i];
            if (dt2[i] < lowestdtFactor * dtvar[i])
                dt2[i] = lowestdtFactor * dtvar[i];

        }


        qDotMaxFromQ[i] = (myIIWALimits.qMax[i] - iiwa->q[i]) / dt2[i];
        qDotMinFromQ[i] = (myIIWALimits.qMin[i] - iiwa->q[i]) / dt2[i];
        qDotMaxFormQDotDot[i] = sqrt(2 * myIIWALimits.qDotDotMax[i] * (myIIWALimits.qMax[i] - iiwa->q[i]));
        qDotMinFormQDotDot[i] = -sqrt(2 * myIIWALimits.qDotDotMax[i] * (iiwa->q[i] - myIIWALimits.qMin[i]));

        if (myIIWALimits.qMax[i] - iiwa->q[i] < 0)
            qDotMaxFormQDotDot[i] = 1000000;

        if (iiwa->q[i] - myIIWALimits.qMin[i] < 0)
            qDotMinFormQDotDot[i] = -1000000;

        vMaxVector = Vector3d(myIIWALimits.qDotMax[i], qDotMaxFromQ[i], qDotMaxFormQDotDot[i]);
        //qDotMaxFinal[i] = getMinValue(vMaxVector);
        qDotMaxFinal[i] = vMaxVector.minCoeff();


        vMinVector = Vector3d(myIIWALimits.qDotMin[i], qDotMinFromQ[i], qDotMinFormQDotDot[i]);
        //qDotMinFinal[i] = getMaxValue(vMinVector);
        qDotMinFinal[i] = vMinVector.maxCoeff();

        aMaxqDot[i] = (qDotMaxFinal[i] - iiwa->qDot[i]) / dtvar[i];
        aMinqDot[i] = (qDotMinFinal[i] - iiwa->qDot[i]) / dtvar[i];

        aMaxQ[i] = 2 * (myIIWALimits.qMax[i] - iiwa->q[i] - iiwa->qDot[i] * dt2[i]) / pow(dt2[i], 2);
        aMinQ[i] = 2 * (myIIWALimits.qMin[i] - iiwa->q[i] - iiwa->qDot[i] * dt2[i]) / pow(dt2[i], 2);

        aMaxVector = Vector3d(aMaxQ[i], aMaxqDot[i], 10000000);
        //qDotDotMaxFinal[i] = getMinValue(aMaxVector);
        qDotDotMaxFinal[i] = aMaxVector.minCoeff();
        aMinVector = Vector3d(aMinQ[i], aMinqDot[i], -10000000);
        //qDotDotMinFinal[i] = getMaxValue(aMinVector);
        qDotDotMinFinal[i] = aMinVector.maxCoeff();

        if (qDotDotMaxFinal[i] < qDotDotMinFinal[i])
        {
            vMaxVector = Vector3d(INFINITY, qDotMaxFromQ[i], qDotMaxFormQDotDot[i]);
            //qDotMaxFinal[i] = getMinValue(vMaxVector);
            qDotMaxFinal[i] = vMaxVector.minCoeff();

            vMinVector = Vector3d(-INFINITY, qDotMinFromQ[i], qDotMinFormQDotDot[i]);
            //qDotMinFinal[i] = getMaxValue(vMinVector);
            qDotMinFinal[i] = vMinVector.maxCoeff();

            aMaxqDot[i] = (qDotMaxFinal[i] - iiwa->qDot[i]) / dtvar[i];
            aMinqDot[i] = (qDotMinFinal[i] - iiwa->qDot[i]) / dtvar[i];

            aMaxVector = Vector3d(aMaxQ[i], aMaxqDot[i], 10000000);
            //qDotDotMaxFinal[i] = getMinValue(aMaxVector);
            qDotDotMaxFinal[i] = aMaxVector.minCoeff();
            aMinVector = Vector3d(aMinQ[i], aMinqDot[i], -10000000);
            //qDotDotMinFinal[i] = getMaxValue(aMinVector);
            qDotDotMinFinal[i] = aMinVector.maxCoeff();
        }
    }


    VectorNd qDotDotS = VectorNd::Zero(LBR_N_JOINTS);
    VectorNd tauS = VectorNd::Zero(LBR_N_JOINTS);
    MatrixNd Psat = Iden;
    bool LimitedExceeded = true;
    bool CreateTaskSat = false;
    int NumSatJoints = 0;
    VectorNd theMostCriticalOld = VectorNd::Zero(LBR_N_JOINTS);
    theMostCriticalOld.conservativeResize(1);
    theMostCriticalOld[0] = 100;
    bool isThere = false;
    int iO = 0;
    int cycle = 0;
    while (LimitedExceeded == true)
    {
        LimitedExceeded = false;
        if (CreateTaskSat == true)
        {
            Js.conservativeResize(NumSatJoints, LBR_N_JOINTS);
            for (int i = 0; i < NumSatJoints; i++)
            {
                for (int k = 0; k < LBR_N_JOINTS; k++)
                {
                    Js(i, k) = 0;
                }
                Js(i, (int)theMostCriticalOld[i]) = 1;
            }

            MatrixNd LambdaSatInv = Js * iiwa->Minv * Js.transpose();
            MatrixNd LambdaSatInv_aux = LambdaSatInv * LambdaSatInv.transpose();
            MatrixNd LambdaSat_aux = LambdaSatInv_aux.inverse();
            MatrixNd LambdaSat = LambdaSatInv.transpose() * LambdaSat_aux;

            MatrixNd JsatBar = iiwa->Minv * Js.transpose() * LambdaSat;
            Psat = Iden - Js.transpose() * JsatBar.transpose();
            VectorNd xDotDot_s = Js * qDotDotS;
            tauS = Js.transpose() * (LambdaSat * xDotDot_s);
        }

        TauBar = tauS + Psat * tauStack;
        qDotDotGot = iiwa->Minv * (TauBar); // it should -g -c

        isThere = false;
        for (int i = 0; i < LBR_N_JOINTS; i++)
        {
            if ((qDotDotMaxFinal[i] + 0.001 < qDotDotGot[i]) || (qDotDotGot[i] < qDotDotMinFinal[i] - 0.001))
            {
                LimitedExceeded = true;
                CreateTaskSat = true;

                for (int k = 0; k < theMostCriticalOld.size(); k++)
                {
                    if (i == theMostCriticalOld[k])
                    {
                        isThere = true;
                    }
                }
                if (isThere == false)
                {

                    theMostCriticalOld.conservativeResize(iO + 1);
                    theMostCriticalOld[iO] = i;
                    iO += 1;
                }
            }
        }

        if (LimitedExceeded == true)
        {
            NumSatJoints = iO;
            theMostCriticalOld.conservativeResize(iO);
            cycle += 1;
            if (cycle > 8)
                LimitedExceeded = false;

            for (int i = 0; i < theMostCriticalOld.size(); i++)
            {
                int jM = theMostCriticalOld[i];

                if (qDotDotGot[jM] > qDotDotMaxFinal[jM])
                    qDotDotS[jM] = qDotDotMaxFinal[jM];

                if (qDotDotGot[jM] < qDotDotMinFinal[jM])
                    qDotDotS[jM] = qDotDotMinFinal[jM];
            }
        }
    }

    VectorNd SJSTorque = TauBar;
    return SJSTorque;
}

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
#include "utils/Reader.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#ifndef NCoef
#define NCoef 1
#endif


Eigen::MatrixXd getLambdaLeastSquares(const Eigen::MatrixXd &M, const Eigen::MatrixXd& J, const double& k)
{
    Eigen::MatrixXd Iden = Eigen::MatrixXd::Identity(J.rows(), J.rows());
    Eigen::MatrixXd Lambda_Inv = J * M.inverse() * J.transpose() + (k*k)*Iden;
    Eigen::MatrixXd Lambda = Lambda_Inv.inverse();
    return Lambda;
}

void computeLinVelToJointTorqueCmd(const double& sample_time, const Eigen::MatrixXd &jacobian, const Eigen::MatrixXd & massMatrix , const Eigen::Vector3d& posCurr , const Eigen::Matrix3d& R_0_E ,const Eigen::VectorXd &qDot, const Eigen::Vector3d &desLinVelocity, const Eigen::Matrix3d &desRotation,Eigen::VectorXd &jointTorqueCommand, Eigen::MatrixXd &nullSpace)
{
	// Operational Space Control (OSC)

	// Compute goal position based on desired velocity.
	Eigen::Vector3d posDes  = posCurr + desLinVelocity * sample_time;

	// Set matrices for current robot configuration.
	const Eigen::MatrixXd jacobianPos = jacobian.block(0,0,3, NUMBER_OF_JOINTS);
	const Eigen::MatrixXd jacobianRot = jacobian.block(3,0,3, NUMBER_OF_JOINTS);
	Eigen::MatrixXd lambda    = getLambdaLeastSquares(massMatrix,jacobian   ,0.3);
	Eigen::MatrixXd lambdaPos = getLambdaLeastSquares(massMatrix,jacobianPos,0.3);
	Eigen::MatrixXd lambdaRot = getLambdaLeastSquares(massMatrix,jacobianRot,0.3);

	// ############################################################################################
	// Compute positional part.
	// ############################################################################################

	auto maxCartSpeed = 0.3;
    const double stiffness = 400;
	const double damping   = 40;
	Eigen::Vector3d posErr  = posDes - posCurr;
	Eigen::Vector3d velCurr = jacobianPos * qDot;
	Eigen::Vector3d velDes  = desLinVelocity + (stiffness / damping) * posErr;
	// Limit velocity to maxCartSpeed.
	velDes = velDes.norm() == 0.0
		? velDes * 0.0
		: velDes * std::min(1.0, maxCartSpeed / velDes.norm());
	// Compute positional force command.
	Eigen::Vector3d forcePos = damping * (velDes - velCurr);

	// ############################################################################################
	// Compute rotation part.
	// ############################################################################################

	const double maxRotSpeed = 2.0;
    const double angularStiffness = 800;
    const double angularDamping   = 40;
	Eigen::Matrix3d R_E_Ed = R_0_E.transpose() * desRotation;
	// Convert rotation error in eef frame to axis angle representation.
    Eigen::AngleAxisd E_AxisAngle(R_E_Ed);
    Eigen::Vector3d E_u = E_AxisAngle.axis();
    double angleErr     = E_AxisAngle.angle();
	// Convert axis to base frame.
    Eigen::Vector3d O_u = R_0_E * E_u;
	// Compute rotational error. (Scaled axis angle)
	Eigen::Vector3d rotErr     = O_u * angleErr;
	Eigen::Vector3d rotVelCurr = jacobianRot * qDot;
	Eigen::Vector3d rotVelDes  = (angularStiffness / angularDamping) * rotErr; // + 0 desired 
	// Limit rotational velocity to maxRotSpeed.
	rotVelDes = rotVelDes.norm() == 0.0
		? rotVelDes * 0.0
		: rotVelDes * std::min(1.0, maxRotSpeed / rotVelDes.norm());
	// Compute rotational force command.
	Eigen::Vector3d forceRot = angularDamping * (rotVelDes - rotVelCurr);

	// ############################################################################################
	// Compute positional and rotational nullspace.
	// ############################################################################################
	
	Eigen::MatrixXd I       = Eigen::MatrixXd::Identity(NUMBER_OF_JOINTS, NUMBER_OF_JOINTS);
	Eigen::MatrixXd jbarPos = massMatrix.inverse() * jacobianPos.transpose() * lambdaPos;
	Eigen::MatrixXd jbarRot = massMatrix.inverse() * jacobianRot.transpose() * lambdaRot;
	Eigen::MatrixXd nullSpaceTranslation = I - jacobianPos.transpose() * jbarPos.transpose();
	Eigen::MatrixXd nullSpaceRotation    = I - jacobianRot.transpose() * jbarRot.transpose();

	// Compute positional and rotation torque from force commands.
	Eigen::VectorXd torquePos = jacobianPos.transpose() * lambdaPos * forcePos;
	Eigen::VectorXd torqueRot = jacobianRot.transpose() * lambdaRot * forceRot;

	// Set torque command.
	jointTorqueCommand = torquePos + nullSpaceTranslation * torqueRot; // Prioritize positional torques. 
	// Set nullspace.
	nullSpace = nullSpaceTranslation * nullSpaceRotation;
}

//******************************************************************************
MyLBRClient::MyLBRClient(std::shared_ptr<SharedState> in_shared_state,const std::string& model_file,const std::string& transform_file) : shared_state{in_shared_state} {

    {
        std::ifstream modelfile{model_file};
        modelfile >> model;
    }

    {   
        std::cout << "arrived here! filepath:\n" << transform_file << std::endl;
	    std::ifstream transformfile{transform_file};
        if(transformfile.is_open())
            std::cout << "partial success\n";
	    nlohmann::json calibration_data = nlohmann::json::parse(transformfile);

        auto rotationstring = calibration_data["rotation"];
        std::cout << "partial success 1\n";
        std::stringstream s;
        s <<  rotationstring;
        auto translationstring = calibration_data["translation"];
        std::cout << "partial success 2\n";
        
        transformation_to_model_coordinates.rotation = curan::utilities::convert_matrix(s);
        s = std::stringstream{};
        s <<  translationstring;
        transformation_to_model_coordinates.translation = curan::utilities::convert_matrix(s);
    }

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
    myIIWALimits.qDotMax[6] = 155 * M_PI / 180;

    myIIWALimits.qDotMin[0] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[1] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[2] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[3] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[4] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[5] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[6] = -155 * M_PI / 180;

    myIIWALimits.qDotDotMax[0] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[1] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[2] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[3] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[4] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[5] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[6] = 300 * M_PI / 180;

    myIIWALimits.qDotDotMin[0] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[1] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[2] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[3] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[4] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[5] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[6] = -300 * M_PI / 180;

    // Attach tool
    toolMass = 0.0;                                                                     // No tool for now
    toolCOM = Vector3d::Zero(3, 1);
    toolInertia = Matrix3d::Zero(3, 3);
    myTool = new ToolData(toolMass, toolCOM, toolInertia);
    //-(change_1 )myLBR->attachToolToRobotModel(myTool);
    robot->attachToolToRobotModel(myTool);

    _qInitial[0] = 0.0 * M_PI / 180;
    _qInitial[1] = 0.0 * M_PI / 180;
    _qInitial[2] = 0.0 * M_PI / 180;
    _qInitial[3] = 0.0 * M_PI / 180;
    _qInitial[4] = 0.0 * M_PI / 180;
    _qInitial[5] = 0.0 * M_PI / 180;
    _qInitial[6] = 0.0 * M_PI / 180;

    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        _qCurr[i] = _qInitial[i];
        _qOld[i] = _qInitial[i];
        _qApplied[i] = 0.0;
        _torques[i] = 0.0;
        _measured_torques[i] = 0.0;
    }

    measured_torque = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    tau_command = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    tau_command_filtered = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    tau_previous = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    tau_prev_prev = VectorNd::Zero(NUMBER_OF_JOINTS, 1);

    // Positions and orientations and Jacobian
    p_0_cur = Vector3d::Zero(3, 1);
    R_0_7 = Matrix3d::Zero(3, 3);
    J = MatrixNd::Zero(6, NUMBER_OF_JOINTS);
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
    memcpy(_qCurr, robotState().getMeasuredJointPosition(), NUMBER_OF_JOINTS * sizeof(double));
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

VectorNd MyLBRClient::addConstraints(const VectorNd& tauStack, double dt)
{
    VectorNd dt2 = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd dtvar = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd qDownBar = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd qTopBar = VectorNd::Zero(NUMBER_OF_JOINTS, 1);

    VectorNd qDotMaxFromQ = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd qDotMinFromQ = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd qDotMaxFormQDotDot = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd qDotMinFormQDotDot = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd vMaxVector = Vector3d::Zero(3);
    VectorNd vMinVector = Vector3d::Zero(3);
    VectorNd qDotMaxFinal = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd qDotMinFinal = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd aMaxqDot = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd aMinqDot = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd aMaxQ = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd aMinQ = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd aMaxVector = Vector3d::Zero(3);
    VectorNd aMinVector = Vector3d::Zero(3);
    VectorNd qDotDotMaxFinal = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd qDotDotMinFinal = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    MatrixNd Iden = MatrixNd::Identity(NUMBER_OF_JOINTS, NUMBER_OF_JOINTS);
    VectorNd TauBar = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    VectorNd qDotDotGot = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
    MatrixNd Js = MatrixNd::Zero(3, NUMBER_OF_JOINTS);

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

    for (int i = 0; i < NUMBER_OF_JOINTS; i++)
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


    VectorNd qDotDotS = VectorNd::Zero(NUMBER_OF_JOINTS);
    VectorNd tauS = VectorNd::Zero(NUMBER_OF_JOINTS);
    MatrixNd Psat = Iden;
    bool LimitedExceeded = true;
    bool CreateTaskSat = false;
    int NumSatJoints = 0;
    VectorNd theMostCriticalOld = VectorNd::Zero(NUMBER_OF_JOINTS);
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
            Js.conservativeResize(NumSatJoints, NUMBER_OF_JOINTS);
            for (int i = 0; i < NumSatJoints; i++)
            {
                for (int k = 0; k < NUMBER_OF_JOINTS; k++)
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
        for (int i = 0; i < NUMBER_OF_JOINTS; i++)
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

//******************************************************************************
void MyLBRClient::command() {
    // Get robot measurements
    memcpy(_qOld, _qCurr, NUMBER_OF_JOINTS * sizeof(double));
    memcpy(_qCurr, robotState().getMeasuredJointPosition(), NUMBER_OF_JOINTS * sizeof(double));
    memcpy(_measured_torques, robotState().getMeasuredTorque(), NUMBER_OF_JOINTS * sizeof(double));

    shared_state->robot_state.store(robotState());
    
    shared_state->is_initialized.store(true);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        iiwa->q[i] = _qCurr[i];
        measured_torque[i] = _measured_torques[i];
    }
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        iiwa->qDot[i] = (_qCurr[i] - _qOld[i]) / sampleTime;
    }
    robot->getMassMatrix(iiwa->M, iiwa->q);
    iiwa->M(6, 6) = 45 * iiwa->M(6, 6);                                       // Correct mass of last body to avoid large accelerations
    iiwa->Minv = iiwa->M.inverse();
    robot->getCoriolisAndGravityVector(iiwa->c, iiwa->g, iiwa->q, iiwa->qDot);
    robot->getWorldCoordinates(p_0_cur, iiwa->q, pointPosition, 7);              // 3x1 position of flange (body = 7), expressed in base coordinates
    robot->getRotationMatrix(R_0_7, iiwa->q, NUMBER_OF_JOINTS);                                // 3x3 rotation matrix of flange, expressed in base coordinates
    
    //  Geometrical Jacobian matrix (Siciliano: Modelling, Planning and Control)
    robot->getJacobian(J, iiwa->q, pointPosition, 7);                         // Jacobian matrix, wrt. point on flange (pointPosition), expressed in base coordinates
    
    // This is a normal damper that removes energy from all joints (makes it easier to mvoe the robot in free space)
    //this adds the joint limits to the robot control (it takes our control commands and it shapes it to avoid torque limits)
    //VectorNd SJSTorque = addConstraints(torques, 0.005);
    
    
    Eigen::Vector3d transformed_position = transformation_to_model_coordinates.rotation*p_0_cur + transformation_to_model_coordinates.translation;
    Eigen::Vector2d limit_cycle_portion = transformed_position.block(0,0,2,1);

    auto in_plane_velocity = model.likeliest(limit_cycle_portion);

    Eigen::Vector3d desired_velocity_in_plane;
    desired_velocity_in_plane << in_plane_velocity(0) , in_plane_velocity(1) , -transformed_position(2);

    Eigen::Vector3d velocity_in_world_coordinates = transformation_to_model_coordinates.rotation.transpose()*desired_velocity_in_plane;
    std::array<double,3> computed_velocity;
    computed_velocity[0] = velocity_in_world_coordinates(0,0);
    computed_velocity[1] = velocity_in_world_coordinates(1,0);
    computed_velocity[2] = velocity_in_world_coordinates(2,0);
    shared_state->velocity.store(computed_velocity);

    //Eigen::Vector3d velocity_in_world_coordinates = Eigen::Vector3d::Zero(); 

    Eigen::Matrix3d desired_orientation = Eigen::Matrix3d::Identity();
    desired_orientation << -0.0185 , 0.0007 , -0.9998 ,
                            0.3030 , 0.9530 , -0.0049 ,
                            0.9528 , -0.3030 , -0.0179;
                            
    //Eigen::VectorXd torqueLinTask = Eigen::VectorXd::Zero(NUMBER_OF_JOINTS);
    //Eigen::MatrixXd nullSpace = Eigen::VectorXd::Zero(NUMBER_OF_JOINTS,NUMBER_OF_JOINTS);

    //computeLinVelToJointTorqueCmd(robotState().getSampleTime(),J,iiwa->M,p_0_cur,R_0_7, iiwa->qDot,velocity_in_world_coordinates, desired_orientation,torqueLinTask, nullSpace); // Fixed rotation for now.
    
	// Apply damping torques in nullspace.
	const double dampingQ = 5;
    //Eigen::MatrixXd clamperQ = Eigen::MatrixXd::Zero(NUMBER_OF_JOINTS,NUMBER_OF_JOINTS);
    //clamperQ(0,0) = 10;
    Eigen::VectorXd dampingTorque = -iiwa->M * dampingQ * iiwa->qDot;
    //Eigen::VectorXd firstJointClamperTorque = -clamperQ*iiwa->q;
	//Eigen::MatrixXd torqueCommand = torqueLinTask + nullSpace*(dampingTorque+firstJointClamperTorque);
    Eigen::MatrixXd torqueCommand = dampingTorque;
		
	// Limit torques to stop at the robot's joint limits.
	//VectorNd SJSTorque = addConstraints(torqueCommand, 0.005);
    //for now we use a damping torque but we would like to use the state-derivative torque
    VectorNd SJSTorque = addConstraints(torqueCommand, 0.005); 
	//this->jointTorqueCommand[6] = 20*this->jointTorqueCommand[6];
    
    std::array<double,NUMBER_OF_JOINTS> local_copy_of_torques;
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        _qApplied[i] = _qCurr[i] + 0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime);
        _torques[i] = SJSTorque[i];
        local_copy_of_torques[i] = torqueCommand(i,0);
    }

    shared_state->joint_torques.store(local_copy_of_torques);

    if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
        robotCommand().setJointPosition(_qApplied);
        robotCommand().setTorque(_torques);
    }

    tau_prev_prev = tau_previous;
    tau_previous = tau_command;

    currentTime = currentTime + sampleTime;
}

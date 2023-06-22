#include <chrono>

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

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#ifndef NCoef
#define NCoef 1
#endif

//******************************************************************************
MyLBRClient::MyLBRClient(std::shared_ptr<SharedRobotState> shared_state) : shared_stat{shared_state}{
	// Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
	kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here
	
	robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

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

	shared_stat->write(robotState());
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
	shared_stat->write(robotState());
}

//******************************************************************************
void MyLBRClient::command() {
	shared_stat->write(robotState());
	// Get robot measurements
	memcpy(_qOld, _qCurr, NUMBER_OF_JOINTS * sizeof(double));
	memcpy(_qCurr, robotState().getMeasuredJointPosition(), NUMBER_OF_JOINTS * sizeof(double));
	memcpy(_measured_torques, robotState().getExternalTorque(), NUMBER_OF_JOINTS * sizeof(double));

	for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		iiwa->q[i] = _qCurr[i];
		measured_torque[i] = _measured_torques[i];
	}
	for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		iiwa->qDot[i] = (_qCurr[i] - _qOld[i]) / sampleTime;
	}
	robot->getMassMatrix(iiwa->M,iiwa->q);
	iiwa->M(6,6) = 45 * iiwa->M(6,6);                                       // Correct mass of last body to avoid large accelerations
	iiwa->Minv = iiwa->M.inverse();
	robot->getCoriolisAndGravityVector(iiwa->c,iiwa->g,iiwa->q,iiwa->qDot);
	robot->getWorldCoordinates(p_0_cur,iiwa->q,pointPosition,7);              // 3x1 position of flange (body = 7), expressed in base coordinates
	robot->getRotationMatrix(R_0_7,iiwa->q,NUMBER_OF_JOINTS);                                // 3x3 rotation matrix of flange, expressed in base coordinates
    
    auto Jacobian = J = MatrixNd::Zero(6, NUMBER_OF_JOINTS);
    robot->getJacobian(Jacobian,iiwa->q,pointPosition,NUMBER_OF_JOINTS);

    auto forces = (Jacobian*Jacobian.transpose()).inverse()*Jacobian*measured_torque;

	for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		_qApplied[i] = _qCurr[i];
		_torques[i] = 0.0;
	}

	if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
		robotCommand().setJointPosition(_qApplied);
		robotCommand().setTorque(_torques);
	}

	tau_prev_prev = tau_previous;
	tau_previous = tau_command;

	currentTime = currentTime + sampleTime;
}

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

#ifndef _KUKA_FRI_MY_LBR_CLIENT_H
#define _KUKA_FRI_MY_LBR_CLIENT_H

#include "friLBRClient.h"
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include <memory>
#include <atomic>

struct SharedState {
    std::atomic<KUKA::FRI::LBRState> robot_state;
};

/**
 * \brief Template client implementation.
 */
class MyLBRClient : public KUKA::FRI::LBRClient
{

public:

    struct RobotLimits
    {
        VectorNd qMin;
        VectorNd qMax;
        VectorNd qDotMin;
        VectorNd qDotMax;
        VectorNd qDotDotMax;
        VectorNd qDotDotMin;

        RobotLimits()
        {
            qMin = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
            qMax = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
            qDotMin = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
            qDotMax = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
            qDotDotMax = VectorNd::Zero(NUMBER_OF_JOINTS, 1);
            qDotDotMin = VectorNd::Zero(NUMBER_OF_JOINTS, 1);

        }
    };


    /**
    * \brief Constructor.
    */
    MyLBRClient(std::shared_ptr<SharedState> shared_state);

    /**
    * \brief Destructor.
    */
    ~MyLBRClient();

    /**
    * \brief Callback for FRI state changes.
    *
    * @param oldState
    * @param newState
    */
    virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);

    /**
    * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
    *
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void monitor();

    /**
    * \brief Callback for the FRI session state 'Commanding Wait'.
    *
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void waitForCommand();

    /**
    * \brief Callback for the FRI state 'Commanding Active'.
    *
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void command();

private:
    VectorNd addConstraints(const VectorNd& tauStack, double dt);

    std::unique_ptr<kuka::Robot> robot;
    std::unique_ptr<RobotParameters> iiwa;
    std::shared_ptr<SharedState> shared_state;
    RobotLimits myIIWALimits;

    double toolMass;
    Vector3d toolCOM;
    Matrix3d toolInertia;
    ToolData* myTool;
    //Vector3d pointPosition = Vector3d(0,0,0.071); // Point on center of flange for MF-Touch
    Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric

    double _qInitial[NUMBER_OF_JOINTS];
    double _qCurr[NUMBER_OF_JOINTS];
    double _qOld[NUMBER_OF_JOINTS];
    double _qApplied[NUMBER_OF_JOINTS];
    double _torques[NUMBER_OF_JOINTS];
    double _measured_torques[NUMBER_OF_JOINTS];

    VectorNd measured_torque;
    VectorNd tau_command;
    VectorNd tau_command_filtered;
    VectorNd tau_previous;
    VectorNd tau_prev_prev;

    MatrixNd J; //upper: translational; lower: rotational
    Vector3d p_0_cur;
    Matrix3d R_0_7;

    double sampleTime = 0;
    double currentTime = 0;
};

#endif // _KUKA_FRI_MY_LBR_CLIENT_H
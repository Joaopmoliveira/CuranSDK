#ifndef _KUKA_FRI_MY_LBR_CLIENT_H
#define _KUKA_FRI_MY_LBR_CLIENT_H

#include "friLBRClient.h"
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include <memory>
#include "FilterRipple.h"

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
            qMin = VectorNd::Zero(NUMBER_OF_JOINTS,1);
            qMax = VectorNd::Zero(NUMBER_OF_JOINTS,1);
            qDotMin = VectorNd::Zero(NUMBER_OF_JOINTS,1);
            qDotMax = VectorNd::Zero(NUMBER_OF_JOINTS,1);
            qDotDotMax = VectorNd::Zero(NUMBER_OF_JOINTS,1);
            qDotDotMin = VectorNd::Zero(NUMBER_OF_JOINTS,1);

        }
    };


    /**
    * \brief Constructor.
    */
    MyLBRClient();

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

    FilterData data_freq;
    FilterProperties properties_freq{ 5.0,320.0 };
    
    std::unique_ptr<kuka::Robot> robot;
    std::unique_ptr<RobotParameters> iiwa;
    RobotLimits myIIWALimits;

    double toolMass;
    Vector3d toolCOM;
    Matrix3d toolInertia;
    ToolData *myTool;
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

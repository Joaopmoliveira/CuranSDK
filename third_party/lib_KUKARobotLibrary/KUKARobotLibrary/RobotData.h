#ifndef ROBOT_DATA_20150427
#define ROBOT_DATA_20150427

#include <rbdl/rbdl.h> 
#include "Robot.h"

class __HIDE_CLASS__ RobotData
{
	public:
		RobotData(int whatRobot);
		RobotData(int whatRobot, unsigned int numLinks, RigidBodyDynamics::Math::VectorNd& linkLengths, RigidBodyDynamics::Math::VectorNd& linkMasses);
		~RobotData();

	private: 
		void initializeVariables(unsigned int numAxes);
		
	public:
		double* mass;
		RigidBodyDynamics::Math::Vector3d* com;
		RigidBodyDynamics::Math::Matrix3d* inertia;

		RigidBodyDynamics::Math::Vector3d* axisDirection;
		RigidBodyDynamics::Math::Vector3d* axisOrigin;

	private: 
		unsigned int numAxes;

};

 /* LIST OF CURRENTLY IMPLEMENTED ROBOTS (whatRobot)
 1: n-DOF snake-robot (for testing n=3)
 2: 2-DOF robot (double pendulum), rotation about y
 3: sample Robot according to http://rbdl.bitbucket.org/dd/d4c/_simple_example.html
 4: LBR4
 7: LBR iiwa 7kg
 8: Dennis YouBot 
 14: LBR iiwa 14kg
 */




#endif
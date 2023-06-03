#ifndef ROBOTPARAMETERS_H
#define ROBOTPARAMETERS_H
/*
#include "rbdl\rbdl.h"
*/

#include <rbdl/rbdl.h> //JL, uncomment this

#ifndef NUMBER_OF_JOINTS 
#define NUMBER_OF_JOINTS 7
#endif

using namespace RigidBodyDynamics::Math;

/**
*	\brief	This structure provides a framework to collect the data that belongs to a robot into the RobotParameters data type. 
*/

struct RobotParameters{

	VectorNd q;
	VectorNd qDot;
	VectorNd c;
	VectorNd g;
	
	MatrixNd M;
	MatrixNd Minv;

	MatrixNd xRobotPoints;
	MatrixNd xDotRobotPoints;
	MatrixNd jacobiansOfRobotPoints;

	RobotParameters()
	{
		q = VectorNd::Zero(NUMBER_OF_JOINTS,1);
		qDot = VectorNd::Zero(NUMBER_OF_JOINTS,1);
		c = VectorNd::Zero(NUMBER_OF_JOINTS,1);
		g = VectorNd::Zero(NUMBER_OF_JOINTS,1);
	
		M = MatrixNd::Zero(NUMBER_OF_JOINTS, NUMBER_OF_JOINTS);
		Minv = MatrixNd::Zero(NUMBER_OF_JOINTS, NUMBER_OF_JOINTS);

		xRobotPoints = MatrixNd::Zero(NUMBER_OF_JOINTS, 3);
		xDotRobotPoints = MatrixNd::Zero(NUMBER_OF_JOINTS, 3);
		jacobiansOfRobotPoints = MatrixNd::Zero(NUMBER_OF_JOINTS * 3, NUMBER_OF_JOINTS); //Only in the translation components
	}
};


#endif

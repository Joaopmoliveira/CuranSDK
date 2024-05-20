#ifndef ROBOTPARAMETERS_H
#define ROBOTPARAMETERS_H
/*
#include "rbdl\rbdl.h"
*/

#include <rbdl/rbdl.h> //JL, uncomment this

constexpr size_t LBR_N_JOINTS = 7;

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
		q = VectorNd::Zero(LBR_N_JOINTS,1);
		qDot = VectorNd::Zero(LBR_N_JOINTS,1);
		c = VectorNd::Zero(LBR_N_JOINTS,1);
		g = VectorNd::Zero(LBR_N_JOINTS,1);
	
		M = MatrixNd::Zero(LBR_N_JOINTS, LBR_N_JOINTS);
		Minv = MatrixNd::Zero(LBR_N_JOINTS, LBR_N_JOINTS);

		xRobotPoints = MatrixNd::Zero(LBR_N_JOINTS, 3);
		xDotRobotPoints = MatrixNd::Zero(LBR_N_JOINTS, 3);
		jacobiansOfRobotPoints = MatrixNd::Zero(LBR_N_JOINTS * 3, LBR_N_JOINTS); //Only in the translation components
	}
};


#endif

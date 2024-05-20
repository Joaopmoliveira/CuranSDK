#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>

#include <rbdl/rbdl.h>
#include "Robot.h"



using namespace RigidBodyDynamics::Math;
using namespace std;


int main (int argc, char** argv)
{
	unsigned int noDofs = 0;

	cout << "\nWelcome to the amazing snake-configurator! \n\r";
	cout << "\nPlease enter the number of Dofs:\n\r";
	cin >> noDofs;
	
	VectorNd Q = VectorNd::Zero (noDofs);   
	VectorNd QDot = VectorNd::Zero (noDofs);
	VectorNd QDotDot = VectorNd::Zero (noDofs);

	VectorNd linkLengths = VectorNd::Zero (noDofs);
	VectorNd linkMasses = VectorNd::Zero (noDofs);

	MatrixNd M = MatrixNd::Zero(noDofs,noDofs);
	VectorNd g = VectorNd::Zero(noDofs);
	VectorNd c = VectorNd::Zero(noDofs);
	MatrixNd J = MatrixNd::Zero(6, noDofs);

	// get lengths
	for (int i=0; i<noDofs; i++)
	{
		cout << "\nlinkLength element " << i << ":\n\r";
		cin >> linkLengths[i];
	}

	// get masses
	for (int i=0; i<noDofs; i++)
	{
		cout << "\nlinkMass element " << i << ":\n\r";
		cin >> linkMasses[i];
	}

	// get Q
	for (int i=0; i<noDofs; i++)
	{
		cout << "\nQ " << i << ":\n\r";
		cin >> Q[i];
	}

	// get QDot
	for (int i=0; i<noDofs; i++)
	{
		cout << "\nQDot " << i << ":\n\r";
		cin >> QDot[i];
	}

	// get QDotDot
	for (int i=0; i<noDofs; i++)
	{
		cout << "\nQDotDot " << i << ":\n\r";
		cin >> QDotDot[i];
	}

	cout << "\nConfiguration complete! \r\n";

	kuka::Robot::robotName myName(kuka::Robot::Snake);
	kuka::Robot myRobot(myName, noDofs, linkLengths, linkMasses);

	cout << "\nRobot built! \r\n";

	myRobot.updateKinematics(Q,QDot,QDotDot);
	
	
	// Get the mass matrix
	myRobot.getMassMatrix(M, Q);
	cout << "\nM = \n" << M << "\n"; 

	
	// Get the vector of velocity-dependent and generalized gravity forces
	myRobot.getCoriolisAndGravityVector(c, g, Q, QDot);
	cout << " \nc = \n" << c << "\n" << " \ng  = \n" << g << "\n";

	

	// Get the Jacobian of any point on any body
	unsigned int bodyIndex = noDofs;
	//Vector3d pointPosition = Vector3d(0, 0, 0.0314); // flange center iiwa
	Vector3d pointPosition = Vector3d(linkLengths[noDofs], 0, 0); // endeffector snake-robot
	myRobot.getJacobian(J, Q, pointPosition, bodyIndex);
	cout << "\nJ = \n" << J << "\n"; 

	// Get the world coordinates of any point on any body
	//Vector3d x = Vector3d::Zero(3, 1);
	//bodyIndex = 7;
	//pointPosition = Vector3d(0, 0, 0.0314); // flange center
	//pointPosition = Vector3d(0, 0, 0);
	//myRobot.getWorldCoordinates(x, Q, pointPosition, bodyIndex);
	//std::cout << "\n x = \n\r " << x << "\n"; 


	// Get the rotation matrix of any body
	//Matrix3d R = Matrix3d::Zero(3, 3);
	//bodyIndex = 7;
	//myRobot.getRotationMatrix(R, Q, bodyIndex);
	//std::cout << "\n R = \n\r " << R << "\n"; 

	int a;
	cin>>a;

	return 0;
	
}


#include "Robot.h"
#include <iostream>
#include "RobotModel.h"
#include "ToolData.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace kuka;


Robot::Robot(robotName name)
{

	rbdl_check_api_version (RBDL_API_VERSION);   

	//robotModel = new Model();
	switch(name)
	{
		case 0:
			this->robotModel = RobotModel::createModelLBR4();
			robot_dof = 7;
			break;
		case 1:
			this->robotModel = RobotModel::createModelIiwa7kg();	
			robot_dof = 7;
			break;
		case 2:
			this->robotModel = RobotModel::createModelIiwa14kg();	
			robot_dof = 7;
			break;
		case 3:
			this->robotModel = RobotModel::createModelYouBot();
			robot_dof = 8;
			break;
		default:
			std::cerr << "Unknown robot type: no model! \n";
	}	
}

Robot::Robot(robotName name, unsigned int numLinks, VectorNd& linkLengths, VectorNd& linkMasses)
{
	rbdl_check_api_version (RBDL_API_VERSION);   

	//robotModel = new Model();
	switch(name)
	{
		case 4:
			this->robotModel = RobotModel::createModelSnake(numLinks, linkLengths, linkMasses);
			robot_dof = numLinks;
			break;
		default:
			std::cerr << "Unknown robot type: no model! \n";
	}	
}


Robot::~Robot()
{
	delete this->robotModel;
}



int Robot::getDOF()
{
	//
	return robot_dof;
}

void Robot::getMassMatrix(MatrixNd& M, VectorNd& Q)
{	
	// Get the mass matrix
	CompositeRigidBodyAlgorithm(*robotModel->model(), Q, M, false);
	//std::cout << "\n M = \n" <<  M << std::endl; 
}


void Robot::getCoriolisAndGravityVector(VectorNd& c, VectorNd& g, VectorNd& Q, VectorNd& QDot)
{
	// Get the vector of generalized gravity forces
	InverseDynamics(*robotModel->model(), Q, VectorNd::Zero(this->robotModel->model()->dof_count), VectorNd::Zero(this->robotModel->model()->dof_count), g);

	// Get the vector of velocity-dependent forces (coriolis etc.)
	InverseDynamics(*robotModel->model(), Q, QDot, VectorNd::Zero(this->robotModel->model()->dof_count), c);
	c = c-g;
}

void Robot::getJacobian(MatrixNd& J, VectorNd& Q, Vector3d pointPosition, unsigned int bodyIndex)
{
	// Get the Jacobian of any point on any body

	MatrixNd J0 = MatrixNd::Zero(6, this->robotModel->model()->dof_count);
	MatrixNd JP;
	
	CalcBodySpatialJacobian(*robotModel->model(), Q, bodyIndex, J0, false);
	
	// Doing my clumsy transforms
	MatrixNd X1 = MatrixNd::Identity(6,6);
	MatrixNd X2 = MatrixNd::Identity(6,6);
	MatrixNd R_7_0 = CalcBodyWorldOrientation(*robotModel->model(), Q, bodyIndex, false);
	
	Vector3d r = pointPosition;
	
	X1.block(0,0,3,3) = R_7_0.transpose();
	X1.block(3,3,3,3) = R_7_0.transpose();
	

	X2(4,0) = -r[2];
	X2(5,0) = +r[1];
	X2(3,1) = +r[2];
	X2(5,1) = -r[0];
	X2(3,2) = -r[1];
	X2(4,2) = +r[0];
	

	JP = X1*X2*J0;
	J.block(0,0,3, this->robotModel->model()->dof_count) = JP.block(3,0,3, this->robotModel->model()->dof_count);
	J.block(3,0,3, this->robotModel->model()->dof_count) = JP.block(0,0,3, this->robotModel->model()->dof_count);
	

}

void Robot::getWorldCoordinates(RigidBodyDynamics::Math::Vector3d& x, RigidBodyDynamics::Math::VectorNd& Q, RigidBodyDynamics::Math::Vector3d pointPosition, unsigned int bodyIndex)
{
       x = CalcBodyToBaseCoordinates(*robotModel->model(), Q, bodyIndex, pointPosition, false);
}

void Robot::getRotationMatrix(RigidBodyDynamics::Math::Matrix3d& R_0_i, RigidBodyDynamics::Math::VectorNd& Q, unsigned int bodyIndex)
{
       R_0_i = CalcBodyWorldOrientation(*robotModel->model(), Q, bodyIndex, false); // returns R_i_0
	   R_0_i.transposeInPlace();	// Do not use "R_0_i.transpose();" - it does not overwrite R_0_i
}

void Robot::updateKinematics(RigidBodyDynamics::Math::VectorNd& Q,RigidBodyDynamics::Math::VectorNd& QDot,RigidBodyDynamics::Math::VectorNd& QDotDot)
{
	UpdateKinematics(*robotModel->model(),Q,QDot,QDotDot);
}

void Robot::attachToolToRobotModel(ToolData* toolData)
{
	Body bodyTool = Body(toolData->mass, toolData->com, toolData->inertia);
	Joint jointTool = Joint(JointTypeFixed);
	this->robotModel->model()->AppendBody(Xtrans(toolData->axisOrigin), jointTool, bodyTool, "tool");
}



 
	
	

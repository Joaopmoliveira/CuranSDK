#include "tasks.h"

int Tasks::gravityToggle = 1;

Tasks::Tasks()
{
	force_star = VectorNd::Zero(7,1);
	force_tilde = VectorNd::Zero(7,1);
	force_tilde_i = VectorNd::Zero(7,1);
	p = VectorNd::Zero(7,1);
	torque = VectorNd::Zero(7,1);
	h = VectorNd::Zero(7,1);

	jacobian = MatrixNd::Zero(7, 7);
	jacobianTranspose = MatrixNd::Zero(7, 7);
	jbar = MatrixNd::Zero(7, 7);
	nullspace = MatrixNd::Identity(7, 7);
	lambda = MatrixNd::Zero(7, 7);
	lambdaInverse = MatrixNd::Zero(7, 7);
	
	I = MatrixNd::Identity(7, 7);	
	constraintNullspace = I;
}

Tasks::~Tasks()
{
}

void Tasks::getInverseJacobian(MatrixNd& jbar, MatrixNd& jTranspose, MatrixNd& lambda)
{
	jbar = iiwa14->Minv * jTranspose * lambda;
}

void Tasks::getNullspace(MatrixNd& nullspace, MatrixNd& jTranspose, MatrixNd& jbar)
{
        nullspace = (I - jTranspose * jbar.transpose());
}

void Tasks::setGravityToggle(int setReset)
{
	if(setReset==1)
		gravityToggle=1;
	else
		gravityToggle=0;
}

VectorNd& Tasks::getTorque()
{
	return torque;
}

MatrixNd& Tasks::returnNullspaceTranspose()
{
	return nullspace;
}

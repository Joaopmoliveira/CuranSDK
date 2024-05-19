#include "taskCreator.h"
#include <iomanip>
TaskCreator::TaskCreator(RobotParameters* robotParam, int taskDim)
{
	//conservativeResize preserves old values while resizing.
	jacobian.conservativeResize(taskDim, 7);
	jacobianTranspose.conservativeResize(7, taskDim);

	jbar.conservativeResize(7, taskDim);

	force_star.conservativeResize(taskDim,1);
	force_tilde.conservativeResize(taskDim,1);
	force_tilde_i.conservativeResize(taskDim,1);

	p.conservativeResize(taskDim,1);

	lambda.conservativeResize(taskDim, taskDim);
	lambdaInverse.conservativeResize(taskDim, taskDim);

	h.conservativeResize(taskDim, 1);

	//torque and nullspace are already 7x7
	//zeroVector = VectorNd::Zero(taskDim, 1);
	//vectorOfOnes = VectorNd::Ones(taskDim, 1);

	iiwa14 = robotParam;
}

TaskCreator::~TaskCreator()
{
}

void TaskCreator::calcTorqueAndNullspace()
{
	torque = jacobianTranspose * lambda * force_tilde_i;

	getInverseJacobian(jbar,jacobianTranspose,lambda);
	getNullspace(nullspace,jacobianTranspose,jbar);
}

void TaskCreator::setJacobian(MatrixNd& j)
{
	jacobian = j;
	jacobianTranspose = jacobian.transpose();
}

MatrixNd& TaskCreator::getJacobian()
{
	return jacobian;
}

MatrixNd& TaskCreator::getJacobianTranspose()
{
	return jacobianTranspose;
}

void TaskCreator::setForce(VectorNd& f, VectorNd& actParameter, VectorNd& torque_star_excluding_task )
{
	force_star = f;

	for (int i=0; i < f.rows(); i++)
	{
		h[i] = actParameter[i] ;		/*! Because actParameter can be > h */
	}
	
	if (gravityToggle==1)
	{
		p = jacobian * iiwa14->Minv * iiwa14->g;
		force_tilde = force_star + p;
	}
	else
	{
		force_tilde = force_star;
	}

	for (int i=0; i< f.rows(); i++)
	{
		if ( h[i] != 1.0)
		{
			force_tilde_i[i] = h[i] * force_tilde[i] + (1 - h[i]) * jacobian.row(i) * iiwa14->Minv * torque_star_excluding_task;
		}
		else
		{
			force_tilde_i[i] = force_tilde[i];
		}
	}
}
 
VectorNd& TaskCreator::getForce()
{
	return force_tilde_i;
}

void TaskCreator::setLambda(MatrixNd& l)
{
	lambda = l;
}

MatrixNd& TaskCreator::getLambda()
{
	return lambda;
}

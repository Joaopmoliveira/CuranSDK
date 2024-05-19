#ifndef TASKCREATOR_H
#define TASKCREATOR_H

#include "tasks.h"
#include "rbdl/rbdl.h"

using namespace RigidBodyDynamics::Math;

/**
* \brief	This class provides functionality for the developer to use define operational space tasks in his application. It inherits the class 'Tasks'. 
*/
class TaskCreator : public Tasks
{
public:
	/**
	* \brief Constructor
	*
	@param robotParam A pointer to the robot type which is used.
	@param taskDim An integer value that defines the dimension that the task exists in.
	*/
	TaskCreator(RobotParameters* robotParam, int taskDim);

	/**
	* \brief Destructor
	*/
	~TaskCreator();

	/**
	* \brief Function that calculates the task torque and nullspace
	*/
	void calcTorqueAndNullspace();

	/**
	* \brief This function is used to set the jacobian of the task.
	*
	@param j The jacobian of the task.
	*/
	void setJacobian(MatrixNd& j);

	/**
	* \brief This function is used to get the jacobian of the task.
	*
	\return The jacobian of the task
	*/
	MatrixNd& getJacobian();

	/**
	* \brief This function is used to get the transpose of the jacobian of the task.
	*
	\return The transpose of the jacobian of the task.
	*/
	MatrixNd& getJacobianTranspose();


	/**
	* \brief This function is used to set the force of the task, alongwith calculating the intermediate desired value of the force and the gravity component of the task force.
	*
	@param f The task force.
	@param actParameter The activation parameter of the task. Default value is a vector of ones.
	@param torque_star_excluding_task The torque of the system without the current task. Default value is a zero vector.
	*/
	void setForce(VectorNd& f, VectorNd& actParameter = vectorOfOnes, VectorNd& torque_star_excluding_task = zeroVector);

	/**
	* \brief This function is used to get the task force.
	*
	\return The task force
	*/
	VectorNd& getForce();

	/**
	* \brief This function is used to set the task Cartesian mass matrix.
	*
	@param l The Cartesian mass matrix of the task.
	*/
	void setLambda(MatrixNd& l);

	/**
	* \brief This function is used to get the Cartesian mass matrix of the task.
	*
	\return The Cartesian mass matrix of the task
	*/
	MatrixNd& getLambda();
	
	int getGravityToggle(){ return gravityToggle;}

};


#endif

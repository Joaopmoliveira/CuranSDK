#ifndef TASKS_H
#define TASKS_H

#include "rbdl/rbdl.h"
#include "robotParameters.h"

using namespace RigidBodyDynamics::Math;



static VectorNd zeroVector = VectorNd::Zero(7, 1); /*!< A vector type initialised with all zeros. */
static VectorNd vectorOfOnes = VectorNd::Ones(7, 1); /*!< A vector type initialised with all ones. */

 
/**
*\brief	This class provides an abstract class that can be used to define different tasks. All tasks have a jacobian, a force, a torque, a nullspace, an inverse jacobian, an activation parameter and a gravity compensation parameter. Alll tasks also have a similar way of calculating inverse jacobians and nullspaces. Since this is an abstract class, no objects can be created. If you want to create a task object, please use the class TaskCreator.
*/
class Tasks
{
public:
	
	/**
	* \brief Constructor.
	*/
	Tasks();
	
	/**
	* \brief Virtual Destructor.
	*/

	virtual ~Tasks();
    
	/**
	* \brief This function computes the inverse jacobian (Jbar) of a task using the formula: Jbar = Minv * Jtranspose * lambda.
	*
	* @param jbar The output matrix which stores the computed result.
	* @param jTranspose The transpose of the Jacobian matrix of the robot task, given as an input.
	* @param lambda The Cartesian mass matrix of the robot task.
	*/
	void getInverseJacobian(MatrixNd& jbar, MatrixNd& jTranspose, MatrixNd& lambda);

	/**
	* \brief This function computes the nullspace of a task using the formula: NspTranspose = Identity - (Jtranspose * JbarTranspose). Note that the transpose of the nullspace is returned.
	*
	* @param nullspace The output matrix which stores the computed result.
	* @param jTranspose The transpose of the Jacobian matrix of the robot task, given as an input.
	* @param jbar The inverse jacobian matrix of the robot task.
	*/

   void getNullspace(MatrixNd& nullspace, MatrixNd& jTranspose, MatrixNd& jbar);

	/**
	* \brief This function sets/ resets the gravity toggle.
	*
	* @param setReset Setting is done by passing 1, resetting by passing 0.
	*/
   void setGravityToggle(int setReset=1);

	/**
	* \brief This function is used to get the task torque.
	*
	\return The task torque
	*/
	VectorNd& getTorque();

	/**
	* \brief This function is used to get the task nullspace.
	*
	\return The transpose of the task nullspace
	*/
	MatrixNd& returnNullspaceTranspose();

protected:
	
	VectorNd force_star;		/*!< A vector used to store the force function chosen. */
	VectorNd force_tilde;		/*!< A vector used to store the force function plus the gravity component of the force function. */
	VectorNd force_tilde_i;		/*!< A vector used to store the intermediate desired value of the force function chosen. */
	VectorNd p;					/*!< A vector used to store the gravity component of the force. */
	VectorNd torque;			/*!< A vector used to store the task torque. */
	VectorNd h;					/*!< A vector used to store the activation parameter of the task. */
	
	MatrixNd jacobian;			/*!< A matrix used to store the jacobian of the task. */
	MatrixNd jacobianTranspose;	/*!< A matrix used to store the transpose of the jacobian. */
	MatrixNd jbar;				/*!< A matrix used to store the inverse task jacobian. A bar where jacobians hang out. */
	MatrixNd nullspace;			/*!< A matrix used to store the nullspace of the task. */
	MatrixNd lambda;			/*!< A matrix used to store the Cartesian mass matrix of the task. */
	MatrixNd lambdaInverse;		/*!< A matrix used to store the inverse of the Cartesian mass matrix. */
	
	MatrixNd constraintNullspace;	/*!< The nullspace of all the higher priority constraints that act on the current task. This is used in the calculation of dynamically consistent Cartesian mass matrix. */
	MatrixNd I;					/*!< The 7x7 identity matrix. Apple might sue you if you use this too often.*/

	RobotParameters *iiwa14;	/*!< A pointer to the robot parameter structure. */

	static int gravityToggle;		/*!< A static toggle used to enable/disable the gravity component of tasks. Public, to enable objects to set this property*/
};


#endif

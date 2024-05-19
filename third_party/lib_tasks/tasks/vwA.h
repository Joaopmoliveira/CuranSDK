#ifndef VIRTUALWALLAVOIDANCE_H
#define VIRTUALWALLAVOIDANCE_H

#include "tasks.h"

/**
* \brief	This class provides functionality for the developer to use virtual wall avoidance in his application. It inherits the class 'Tasks', because virtual wall avoidance is a type of task. This class will be used by Donald Trump to keep the Mexicans out. It can also be used by Jon Snow.
*/
class VirtualWallAvoidance : public Tasks
{
public:
	/**
	* \brief Constructor
	*
	@param robotParam A pointer to the robot type which is used.
	@param virtualWallActivationZone The coordinates of the activation zone for the virtual wall.
	@param virtualWallPosition The position of the virtual wall. (Coordinates of Mexico required)
	*/
	VirtualWallAvoidance(RobotParameters* robotParam, Vector3d& virtualWallActivationZone, Vector3d& virtualWallPosition);
	
	/**
	* \brief Destructor
	*/
	~VirtualWallAvoidance();

   /**
	* \brief This function is used to determine if the robot's sixth joint exceeds a virtual wall in the xz plane.
	*
	returns If the sixth joint of the robot encounters a virtual wall 1 is returned, otherwise 0 is returned.
   */
   int hasRobotExceededAWall();

   /**
   * \brief This function outputs the calculated repulsion torque for the virtual wall avoidance task and the task's nullspace.
   *
   @param torque_star_excluding_vw The torque of the system excluding the virtual wall avoidance task. This is required as per the equation 17 in the Robot Control near Singularities and Joint Limits paper by Hyejin Han and Jaeheung Park (2013).
   @param constraintNullspace The nullspace of the tasks which are at a higher priority than the virtual wall task. This is used to calculate the dynamically consistent lambda matrix.
   */
   void calcTorqueAndNullspace(VectorNd& torque_star_excluding_vw, MatrixNd& constraintNullspace);

private:

	int noOfExceededWallDirections, virtualWallExceeded;

	int exceededWallDirectionIndex[2]; //2 dimensional task

	Vector3d xActivation;
	Vector3d xForbidden;
	Vector3d xBuffer;

	VectorNd torque_star_excluding_vw;
   /**
	* \brief This function is used to determine the Jacobian and Cartesian mass matrix for the wall repulsion task.
   */
   void calcWallRepulsion();
   
   /**
	* \brief This function is used to determine the repulsion force on the robot's sixth joint, when it enters the virtual wall.
   */
   void wallRepulsionForce();

};

#endif
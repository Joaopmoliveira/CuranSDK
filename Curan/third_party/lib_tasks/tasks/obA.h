#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

#include "tasks.h"


/**
* \brief This class provides functionality for the developer to use obstacle avoidance in his application.  It inherits the class 'Tasks', because obstacle avoidance is a type of task. 
*/
class ObstacleAvoidance : public Tasks
{
public:
	/**
	* \brief Constructor
	*
	@param robotParam A pointer to the robot type which is used.
	@param radiusOfObstacle The radius of the obstacle.
	@param actParameterRadius The buffer region where the activation parameter starts. 
	@param radiusOfHullAroundRobotJoints The radius of the hull around each of the robot joint.
	@param centerOfObstacle The Cartesian center of the obstacle.
	*/
	ObstacleAvoidance(RobotParameters* robotParam, double radiusOfObstacle, double actParameterRadius, double radiusOfHullAroundRobotJoints, Vector3d& centerOfObstacle);

	/**
	* \brief Destructor
	*/
	~ObstacleAvoidance();
	
   /**
   * \brief This function is used to determine if any of the axes of the robot are close to an obstacle whose position in space is fixed.
   *
   returns If any joint of the robot encounters an obstacle 1 is returned, otherwise 0 is returned.
   */
   int isObstacleDetected();

   /**
   * \brief This function outputs the calculated repulsion torque for the obstacle avoidance task and the task's nullspace.
   *
   @param torque_star_excluding_ob The torque of the system excluding the obstacle avoidance task. This is required as per the equation 17 in the Robot Control near Singularities and Joint Limits paper by Hyejin Han and Jaeheung Park (2013).
   @param constraintNullspace The nullspace of the tasks which are at a higher priority than the obstacle avoidance task. This is used to calculate the dynamically consistent lambda matrix.
   */
   void calcTorqueAndNullspace(VectorNd& torque_star_excluding_ob, MatrixNd& constraintNullspace);

private:

	int noOfExceededPoints, noOfExceededCartesianDirections, obstacleDetected;

	int exceededCartIndex[3], exceededPointIndex[7];

	double positionVector, rOb, rAct, rHull;

	double positionVectorArray[7];

	Vector3d xCenter;
	Vector3d xRobotPoint;
	Vector3d xDotRobotPoint;

	VectorNd cartRepulsionForce_i;
	VectorNd torque_star_excluding_ob;

	MatrixNd J_ob;
	MatrixNd JRobotPoint;
	
   /**
   * \brief This function monitors the Cartesian position, velocity and Jacobian of each axis of the robot point. Then, for each axis, this function calls the other functions that compute the repulsion forces and constraint task Jacobians. Finally, the function assembles the forces and Jacobians for each axis of the robot, and computes an overall constraint task Cartesian mass matrix and nullspace of this constraint task.
   */
   void calcObstacleAvoidanceTorques();

   /**
   * \brief This function is called by calcObstacleAvoidanceTorques() for each axis. It computes the repulsion force based on the proximity of the axis near the obstacle.
   */

   void calcObstacleRepulsionForces();

   /**
   * \brief This function is called by calcObstacleAvoidanceTorques() for each axis. It computes the Jacobian and final intermediate value of the repulsion force.
   */
   void calcObstacleAvoidanceParameters();

};

#endif

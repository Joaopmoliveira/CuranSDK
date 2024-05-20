#ifndef JLAVOID_H
#define JLAVOID_H

#include "tasks.h"
#include "rbdl/rbdl.h"

using namespace RigidBodyDynamics::Math;

/**
* \brief The class is used to provide the functionality of joint limit avoidance to your application. It inherits the class 'Tasks', because joint limit avoidance is a type of task. 
*/
class JointLimitAvoidance : public Tasks
{

public:
	/**
	*	\brief Constructor
	*
	@param robotParam A pointer to the robot type which is used.
	@param q_activation The activation zones of the different joints of the robot.
	@param qLimit The joint limits of each joint of the robot.
	@param q0 The buffer zone size.
	@param tauMaxJoint The maximum permissible torque in each joint.
	*/
	JointLimitAvoidance(RobotParameters* robotParam, MatrixNd q_activation, MatrixNd qLimit, double q0, VectorNd tauMaxJoint);
	
	/**
	*	\brief Destructor
	*/
	~JointLimitAvoidance();

   /**
   * \brief This function is used to determine if any of the axes of the robot have come close to their respective joint limits.
   *
   @param exceededJointIndex A pointer to an array which indicates the axes which have exceeded the joint limits.
   returns If a joint limit is exceeded 1 is returned, otherwise 0 is returned.
   */

   int hasAJointExceededLimits(int* exceededJointIndex);
   
   /**
   * \brief This function outputs the calculated joint limit avoidance torque and its nullspace.
   *
   @param torque_star_excluding_jla The torque of the system excluding the joint limit avoidance task. This is required as per the equation 17 in the Robot Control near Singularities and Joint Limits paper by Hyejin Han and Jaeheung Park (2013).
   @param constraintNullspace The nullspace of the tasks which are at a higher priority than the joint limit avoidance task. This is used to calculate the dynamically consistent lambda matrix.
   */
   void calcTorqueAndNullspace(VectorNd& torque_star_excluding_jla, MatrixNd& constraintNullspace);



private:

   /**
   * \brief This function is used to determine the repulsion force (FIRAS/Felix) required to be exerted on the joint(s) which is(/are) in its(/their) respective activation zone(s).
   */
   void calcRepulsionForce();

   /**
   * \brief This function is used to determine an activation parameter that is used to introduce the repulsion task smoothly. The value of the activation parameter depends on the position of a joint in its respective activation zone.
   */
   void getActivationParameter();
   
   /**
   * \brief This function is used to determine the repulsion force (FIRAS/Felix) required to be exerted on the joint(s) which is(/are) in its(/their) respective activation zone(s).
   */
   void calcRepulsionParameters();

	int noOfExceededJointLimits, hasAJointExceeded;

	int exceededJointIndex[7];					//
	
	double q0;

	VectorNd tauMaxJoint;
	VectorNd torque_star_excluding_jla;

	MatrixNd q_activation;
	MatrixNd qLimit;

	MatrixNd constraintNullspace;
};

#endif

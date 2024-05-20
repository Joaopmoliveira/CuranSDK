#ifndef SINGAVOID_H
#define SINGAVOID_H

#include <rbdl/rbdl.h>
#include "fstream"
#include "tasks.h"

using namespace RigidBodyDynamics::Math;
using namespace std;

/**
* \brief	This class provides functionality for the developer to use singularity avoidance in his application.  It inherits the class 'Tasks', because singularity avoidance is a type of task. 
*/

class SingAvoid : public Tasks
{
    public:

		/**
		*	\brief Constructor
		*
		@param robotParam A pointer to the robot type which is used.
		*/
        SingAvoid(RobotParameters* robotParam);
		
		/**
		*	\brief Destructor
		*/
		~SingAvoid();

		/**
		* \brief Function to enable robot control in singularity-prone paths.
		*
		@param taskTorque The singularity compensated task torque provided using this variable as output.
		@param lambdaNew The output singularity compensated Cartesian mass matrix. (Note: This has to be of Matrix3d type, as the SVD function used works only on Matrix sizes upto 4x4. Refer Eigen documentation.)
		@param M_inv The robot inertia matrix.
		@param lambda_inv The inverse of the Cartesian mass matrix is used to check for singularities. (Note: This has to be of Matrix3d type, as the SVD function used works only on Matrix sizes upto 4x4. Refer Eigen documentation.)
		@param J The jacobian matrix of the robot task. This matrix is rearranged in non-singular and singular directions. Since the matrix is passed as reference, the same variable is used as an output.
		@param force The force vector of the robot task. This vector is rearranged in non-singular and singular directions. Since the vector is passed as reference, the same variable is used as an output.
		@param constraintTorque The torque that serve as a constraint to the robot's trajectory achieving task. This variable is required to calculate the term of T*[/p,s] in equation 38 of the paper Robot Control near Singularities and Joint Limits by Hyejin Han and Jaeheung Park, 2013.
		@param constraintNullspace The nullspace of the constraint to the robot's trajectory achieving task. This variable is required to calculate the term of T*[/p,s] in equation 38 of the aforementioned paper.
		@param lowerPriorityTorque The torque of the lower priority task that is used in the calculation of the term T*[/p,s] in equation 38 of the aforementioned paper.
		*
		@return 0 - if the trajectory is not singular, 1- if the trajectory is singular.
		*/
        int singularityAvoidance(VectorNd &taskTorque, MatrixNd &lambdaNew, MatrixNd &M_inv, MatrixNd &lambda_inv, MatrixNd &J, Vector3d &force, VectorNd &constraintTorque, MatrixNd &constraintNullspace, MatrixNd &lowerPriorityTorque);

    private:
        //define all functions variables

        int singIndex[3], noOfSingularities, singAvoidance;

        double ul,ll,b;

        VectorNd f_t_ns_i;
        VectorNd f_t_s_i;
        VectorNd f_t_ns;
        VectorNd f_t_s;
        VectorNd h_t_s;
        VectorNd s;


        MatrixNd u;
        MatrixNd J_t_ns;
		MatrixNd J_t_ns_transpose;
        MatrixNd J_t_s;
		MatrixNd jbarNS;
        MatrixNd lambda_t_ns;
        MatrixNd lambda_t_s;
        MatrixNd u_n_s;
        MatrixNd u_s;
        MatrixNd sigma;
        MatrixNd J;
        MatrixNd M_inv;
		MatrixNd nullspaceOfNSDirection;

        Matrix3d lambdaInv3d;

 

};

#endif
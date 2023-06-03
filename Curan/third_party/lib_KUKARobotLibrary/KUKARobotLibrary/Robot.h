#ifndef ROBOT_20150511
#define ROBOT_20150511

#ifdef _WIN32
	#define __SHOW_CLASS__ __declspec(dllexport)
	#define __HIDE_CLASS__ 
#else
	#define __SHOW_CLASS__ 
	#define __HIDE_CLASS__ __attribute__((visibility ("hidden")))
#endif

#include <rbdl/rbdl.h> 

class RobotModel;
class ToolData;

namespace kuka
{

class __SHOW_CLASS__ Robot
{
	public:
		// The naming ceremony of new robots is celebrated here
		enum robotName {LBR4, LBRiiwa, LBRiiwa14, YouBot, Snake};

		Robot(robotName name);
		Robot(robotName name, unsigned int numLinks, RigidBodyDynamics::Math::VectorNd& linkLengths, RigidBodyDynamics::Math::VectorNd& linkMasses);
		~Robot();

		void getMassMatrix(RigidBodyDynamics::Math::MatrixNd& M, RigidBodyDynamics::Math::VectorNd& Q);
		void getCoriolisAndGravityVector(RigidBodyDynamics::Math::VectorNd& c, RigidBodyDynamics::Math::VectorNd& g, RigidBodyDynamics::Math::VectorNd& Q, RigidBodyDynamics::Math::VectorNd& QDot);
		void getJacobian(RigidBodyDynamics::Math::MatrixNd& J, RigidBodyDynamics::Math::VectorNd& Q, RigidBodyDynamics::Math::Vector3d pointPosition, unsigned int bodyIndex);
		void getRotationMatrix(RigidBodyDynamics::Math::Matrix3d& R_i_0, RigidBodyDynamics::Math::VectorNd& Q, unsigned int bodyIndex);
		void getWorldCoordinates(RigidBodyDynamics::Math::Vector3d& x, RigidBodyDynamics::Math::VectorNd& Q, RigidBodyDynamics::Math::Vector3d pointPosition, unsigned int bodyIndex);
		
		// A class method is used to obtain the DOF information of the robot under consideration, even though this information can be obtained from variables present in the robotModel class. This is because we must use the DOF information in the IiwaMatlabWrapper.cpp but the dof member of the robotModel is private to the robotModel class. Hence we use the getDOF function as a work-around.
		int getDOF();

		void updateKinematics(RigidBodyDynamics::Math::VectorNd& Q,RigidBodyDynamics::Math::VectorNd& QDot,RigidBodyDynamics::Math::VectorNd& QDotDot);

		void attachToolToRobotModel(ToolData* toolData);
		

	private:
		RobotModel* robotModel;
		int robot_dof;
		
	
		
};

}


#endif

#ifndef ROBOT_MODEL_20150511
#define ROBOT_MODEL_20150511

#include <rbdl/rbdl.h> 
#include "Robot.h"

class __HIDE_CLASS__ RobotModel
{
	public:
		static RobotModel* createModelSampleRobot();
		static RobotModel* createModelTwoDof();
		static RobotModel* createModelIiwa7kg();
		static RobotModel* createModelIiwa14kg();
		static RobotModel* createModelLBR4();
		static RobotModel* createModelYouBot();
		static RobotModel* createModelSnake(unsigned int numLinks, RigidBodyDynamics::Math::VectorNd& linkLengths, RigidBodyDynamics::Math::VectorNd& linkMasses);

		RigidBodyDynamics::Model* model() { return m_model;}

		~RobotModel() {delete m_model;}
	private:
		RigidBodyDynamics::Model* m_model;
};



#endif

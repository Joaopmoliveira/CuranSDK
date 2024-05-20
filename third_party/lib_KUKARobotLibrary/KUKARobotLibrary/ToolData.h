#ifndef TOOL_DATA_20150427
#define TOOL_DATA_20150427

#include <rbdl/rbdl.h> 
#include "Robot.h"

class ToolData {
public:
	ToolData(double mass,
			RigidBodyDynamics::Math::Vector3d& com,
			RigidBodyDynamics::Math::Matrix3d& inertia);
	~ToolData();

private:
	void initializeVariables();

public:
	double mass;
	RigidBodyDynamics::Math::Vector3d com;
	RigidBodyDynamics::Math::Matrix3d inertia;

	
	RigidBodyDynamics::Math::Vector3d axisOrigin;
	//RigidBodyDynamics::Math::Vector3d axisDirection;

};

#endif

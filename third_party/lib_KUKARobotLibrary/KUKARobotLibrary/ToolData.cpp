#include "ToolData.h"

using namespace RigidBodyDynamics::Math;

ToolData::ToolData(double mass,
		RigidBodyDynamics::Math::Vector3d& com,
		RigidBodyDynamics::Math::Matrix3d& inertia) {
	//initializeVariables();

	this->mass = mass;
	this->com = com;
	this->inertia = inertia;
	
	axisOrigin = Vector3d(1., 0., 0.); // same as axis 12 of Valeri
}

ToolData::~ToolData() {
//	delete[] mass;
//	delete[] com;
//	delete[] inertia;
//
//	delete[] axisOrigin;
	
}

void ToolData::initializeVariables() {
//	//mass = new double[1];
//	com = new Vector3d[1];
//	inertia = new Matrix3d[3];
//
//	axisOrigin = new Vector3d[1];	
}


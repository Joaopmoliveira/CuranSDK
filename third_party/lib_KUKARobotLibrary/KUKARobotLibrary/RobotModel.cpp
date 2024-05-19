
#include <rbdl/rbdl.h> 

#include "RobotModel.h"
#include "RobotData.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;



RobotModel* RobotModel::createModelSnake(unsigned int numLinks, VectorNd& linkLengths, VectorNd& linkMasses)
{
	unsigned int *body_id = new unsigned int[numLinks];
	Body *body = new Body[numLinks];
	Joint *joint = new Joint[numLinks];

	Model* result = new Model();

	RobotData* robotData = new RobotData(1, numLinks, linkLengths, linkMasses);

	result->gravity = Vector3d (0.0, -9.81, 0.0);

	for(int i=0; i<numLinks; i++)
	{
		if(i==0)
		{
			body[i] = Body(robotData->mass[i], robotData->com[i], robotData->inertia[0]);
			joint[i] = Joint(JointTypeRevolute, robotData->axisDirection[i]);
			body_id[i] = result->AddBody(0, Xtrans(robotData->axisOrigin[i]), joint[i], body[i]);

			std::cout << "BodyID " << body_id[i] << " built. \n";
		}
		else
		{
			body[i] = Body(robotData->mass[i], robotData->com[i], robotData->inertia[0]);
			joint[i] = Joint(JointTypeRevolute, robotData->axisDirection[i]);
			body_id[i] = result->AddBody(body_id[i-1], Xtrans(robotData->axisOrigin[i]), joint[i], body[i]);

			std::cout << "BodyID " << body_id[i] << " built. \n";
		}
	}

	std::cout << "Snake model built \n";

	delete robotData;

	RobotModel* ret = new RobotModel();
	ret->m_model = result;
	return ret;


	//#####################################################################//
	//#####################################################################//

	//unsigned int body1_id, body2_id, body3_id;
	//Body body1, body2, body3;   
	//Joint joint1, joint2, joint3;   

	//Model* result = new Model();  

	//RobotData* robotData = new RobotData(1);	// 1 as input parameter describes a snake-robot
	//
	//result->gravity = Vector3d (0.0, -9.81, 0.0);   
	//
	//body1 = Body(robotData->mass[0], robotData->com[0], robotData->inertia[0]);     
	//joint1 = Joint(     JointTypeRevolute,     robotData->axisDirection[0]   );  
	//body1_id = result->AddBody(0, Xtrans(robotData->axisOrigin[0]), joint1, body1);    

	//body2 = Body(robotData->mass[1], robotData->com[1], robotData->inertia[1]);     
	//joint2 = Joint (     JointTypeRevolute,     robotData->axisDirection[1]   );    
	//body2_id = result->AddBody(body1_id, Xtrans(robotData->axisOrigin[1]), joint2, body2);

	//body3 = Body(robotData->mass[2], robotData->com[2], robotData->inertia[2]);     
	//joint3 = Joint (     JointTypeRevolute,     robotData->axisDirection[2]   );    
	//body3_id = result->AddBody(body2_id, Xtrans(robotData->axisOrigin[2]), joint3, body3);    

	//std::cout << "Snake model built \n";
	//std::cout << "BodyIDs: " << body1_id << ", " << body2_id << ", " << body3_id << "\n ";

	//delete robotData;

	//RobotModel* ret = new RobotModel();
	//ret->m_model = result;
	//return ret;
}

RobotModel* RobotModel::createModelTwoDof()
{
	unsigned int body1_id, body2_id;
	Body body1, body2;   
	Joint joint1, joint2;   

	Model* result = new Model();  

	RobotData* robotData = new RobotData(2);
	
	result->gravity = Vector3d (0.0, 0.0, -9.81);   
	
	body1 = Body(robotData->mass[0], robotData->com[0], robotData->inertia[0]);     
	joint1 = Joint(     JointTypeRevolute,     robotData->axisDirection[0]   );  
	body1_id = result->AddBody(0, Xtrans(robotData->axisOrigin[0]), joint1, body1);    

	body2 = Body(robotData->mass[1], robotData->com[1], robotData->inertia[1]);     
	joint2 = Joint (     JointTypeRevolute,     robotData->axisDirection[1]   );    
	body2_id = result->AddBody(body1_id, Xtrans(robotData->axisOrigin[1]), joint2, body2);     

	std::cout << "TwoDof model built \n";
	std::cout << "BodyIDs: " << body1_id << ", " << body2_id << "\n ";

	delete robotData;

	RobotModel* ret = new RobotModel();
	ret->m_model = result;
	return ret;
}

// =======================================================================================================

RobotModel* RobotModel::createModelSampleRobot()
{
	unsigned int body_a_id, body_b_id, body_c_id;
	Body body_a, body_b, body_c;   
	Joint joint_a, joint_b, joint_c;   

	Model* result = new Model();  

	RobotData* robotData = new RobotData(3);
	
	result->gravity = Vector3d (0., 0., -9.81);   
	
	//body_a = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));     
	body_a = Body(robotData->mass[0], robotData->com[0], robotData->inertia[0]);     
	//joint_a = Joint(     JointTypeRevolute,     Vector3d (0., 0., 1.)   );  
	joint_a = Joint(     JointTypeRevolute,     robotData->axisDirection[0]   );  
	//body_a_id = result->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);    
	body_a_id = result->AddBody(0, Xtrans(robotData->axisOrigin[0]), joint_a, body_a);    

	//body_b = Body (1., Vector3d (0., 0.5, 0.), Vector3d (1., 1., 1.));     
	body_b = Body(robotData->mass[1], robotData->com[1], robotData->inertia[1]);     
	//joint_b = Joint (     JointTypeRevolute,     Vector3d (0., 0., 1.)   );    
	joint_b = Joint (     JointTypeRevolute,     robotData->axisDirection[1]   );    
	//body_b_id = result->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);     
	body_b_id = result->AddBody(body_a_id, Xtrans(robotData->axisOrigin[1]), joint_b, body_b);     

	//body_c = Body(0., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));    
	body_c = Body(robotData->mass[2], robotData->com[2], robotData->inertia[2]);     
	//joint_c = Joint(     JointTypeRevolute,     Vector3d (0., 0., 1.)   );    
	joint_c = Joint(     JointTypeRevolute,     robotData->axisDirection[2]   );    
	//body_c_id = result->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);  
	body_c_id = result->AddBody(body_b_id, Xtrans(robotData->axisOrigin[2]), joint_c, body_c);  

	std::cout << "SampleRobot model built \n";
	std::cout << "BodyIDs: " << body_a_id << ", " << body_b_id << ", " << body_c_id << "\n ";

	delete robotData;

	RobotModel* ret = new RobotModel();
	ret->m_model = result;
	return ret;
}


// =======================================================================================================

RobotModel* RobotModel::createModelLBR4()
{
	unsigned int body1_id, body2_id, body3_id, body4_id, body5_id, body6_id, body7_id;
	Body body1, body2, body3, body4, body5, body6, body7;   
	Joint joint1, joint2, joint3, joint4, joint5, joint6, joint7;   

	Model* result = new Model();  

	RobotData* robotData = new RobotData(4);
	
	result->gravity = Vector3d (0.0, 0.0, -9.81);   
	
	body1 = Body(robotData->mass[0], robotData->com[0], robotData->inertia[0]);     
	joint1 = Joint(     JointTypeRevolute,     robotData->axisDirection[0]   );  
	body1_id = result->AddBody(0, Xtrans(robotData->axisOrigin[0]), joint1, body1, "1");    
	
	body2 = Body(robotData->mass[1], robotData->com[1], robotData->inertia[1]);     
	joint2 = Joint (     JointTypeRevolute,     robotData->axisDirection[1]   );    
	body2_id = result->AddBody(body1_id, Xtrans(robotData->axisOrigin[1]), joint2, body2, "2");     

	body3 = Body(robotData->mass[2], robotData->com[2], robotData->inertia[2]);     
	joint3 = Joint (     JointTypeRevolute,     robotData->axisDirection[2]   );    
	body3_id = result->AddBody(body2_id, Xtrans(robotData->axisOrigin[2]), joint3, body3, "3");     

	body4 = Body(robotData->mass[3], robotData->com[3], robotData->inertia[3]);     
	joint4 = Joint (     JointTypeRevolute,     robotData->axisDirection[3]   );    
	body4_id = result->AddBody(body3_id, Xtrans(robotData->axisOrigin[3]), joint4, body4, "4");     

	body5 = Body(robotData->mass[4], robotData->com[4], robotData->inertia[4]);     
	joint5 = Joint (     JointTypeRevolute,     robotData->axisDirection[4]   );    
	body5_id = result->AddBody(body4_id, Xtrans(robotData->axisOrigin[4]), joint5, body5, "5");     

	body6 = Body(robotData->mass[5], robotData->com[5], robotData->inertia[5]);     
	joint6 = Joint (     JointTypeRevolute,     robotData->axisDirection[5]   );    
	body6_id = result->AddBody(body5_id, Xtrans(robotData->axisOrigin[5]), joint6, body6, "6");     

	body7 = Body(robotData->mass[6], robotData->com[6], robotData->inertia[6]);     
	joint7 = Joint (     JointTypeRevolute,     robotData->axisDirection[6]   );    
	body7_id = result->AddBody(body6_id, Xtrans(robotData->axisOrigin[6]), joint7, body7, "7");     

	std::cout << "LBR4 model built \n";
	std::cout << "BodyIDs: " << body1_id << ", " << body2_id << ", " << body3_id << ", " << body4_id << ", " << body5_id << ", " << body6_id << ", " << body7_id << "\n ";

	delete robotData;

	RobotModel* ret = new RobotModel();
	ret->m_model = result;
	return ret;
}

// =======================================================================================================


RobotModel* RobotModel::createModelIiwa7kg()
{
	unsigned int body1_id, body2_id, body3_id, body4_id, body5_id, body6_id, body7_id;
	Body body1, body2, body3, body4, body5, body6, body7;   
	Joint joint1, joint2, joint3, joint4, joint5, joint6, joint7;   

	
	Model* result = new Model();  
	
	RobotData* robotData = new RobotData(7);
	
	result->gravity = Vector3d (0.0, 0.0, -9.81);   
	
	body1 = Body(robotData->mass[0], robotData->com[0], robotData->inertia[0]);     
	joint1 = Joint(     JointTypeRevolute,     robotData->axisDirection[0]   );  
	body1_id = result->AddBody(0, Xtrans(robotData->axisOrigin[0]), joint1, body1, "1");    

	body2 = Body(robotData->mass[1], robotData->com[1], robotData->inertia[1]);     
	joint2 = Joint (     JointTypeRevolute,     robotData->axisDirection[1]   );    
	body2_id = result->AddBody(body1_id, Xtrans(robotData->axisOrigin[1]), joint2, body2, "2");     

	body3 = Body(robotData->mass[2], robotData->com[2], robotData->inertia[2]);     
	joint3 = Joint (     JointTypeRevolute,     robotData->axisDirection[2]   );    
	body3_id = result->AddBody(body2_id, Xtrans(robotData->axisOrigin[2]), joint3, body3, "3");     

	body4 = Body(robotData->mass[3], robotData->com[3], robotData->inertia[3]);     
	joint4 = Joint (     JointTypeRevolute,     robotData->axisDirection[3]   );    
	body4_id = result->AddBody(body3_id, Xtrans(robotData->axisOrigin[3]), joint4, body4, "4");     

	body5 = Body(robotData->mass[4], robotData->com[4], robotData->inertia[4]);     
	joint5 = Joint (     JointTypeRevolute,     robotData->axisDirection[4]   );    
	body5_id = result->AddBody(body4_id, Xtrans(robotData->axisOrigin[4]), joint5, body5, "5");     

	body6 = Body(robotData->mass[5], robotData->com[5], robotData->inertia[5]);     
	joint6 = Joint (     JointTypeRevolute,     robotData->axisDirection[5]   );    
	body6_id = result->AddBody(body5_id, Xtrans(robotData->axisOrigin[5]), joint6, body6, "6");     

	body7 = Body(robotData->mass[6], robotData->com[6], robotData->inertia[6]);     
	joint7 = Joint (     JointTypeRevolute,     robotData->axisDirection[6]   );    
	body7_id = result->AddBody(body6_id, Xtrans(robotData->axisOrigin[6]), joint7, body7, "7");     

	std::cout << "LBR iiwa 7kg model built \n";
	std::cout << "BodyIDs: " << body1_id << ", " << body2_id << ", " << body3_id << ", " << body4_id << ", " << body5_id << ", " << body6_id << ", " << body7_id << "\n ";

	delete robotData;

	RobotModel* ret = new RobotModel();
	ret->m_model = result;
	return ret;
}

RobotModel* RobotModel::createModelIiwa14kg()
{
	unsigned int body1_id, body2_id, body3_id, body4_id, body5_id, body6_id, body7_id;
	Body body1, body2, body3, body4, body5, body6, body7;   
	Joint joint1, joint2, joint3, joint4, joint5, joint6, joint7;   

	
	Model* result = new Model();  
	
	RobotData* robotData = new RobotData(14);
	
	result->gravity = Vector3d (0.0, 0.0, -9.81);   
	
	body1 = Body(robotData->mass[0], robotData->com[0], robotData->inertia[0]);     
	joint1 = Joint(     JointTypeRevolute,     robotData->axisDirection[0]   );  
	body1_id = result->AddBody(0, Xtrans(robotData->axisOrigin[0]), joint1, body1, "1");    

	body2 = Body(robotData->mass[1], robotData->com[1], robotData->inertia[1]);     
	joint2 = Joint (     JointTypeRevolute,     robotData->axisDirection[1]   );    
	body2_id = result->AddBody(body1_id, Xtrans(robotData->axisOrigin[1]), joint2, body2, "2");     

	body3 = Body(robotData->mass[2], robotData->com[2], robotData->inertia[2]);     
	joint3 = Joint (     JointTypeRevolute,     robotData->axisDirection[2]   );    
	body3_id = result->AddBody(body2_id, Xtrans(robotData->axisOrigin[2]), joint3, body3, "3");     

	body4 = Body(robotData->mass[3], robotData->com[3], robotData->inertia[3]);     
	joint4 = Joint (     JointTypeRevolute,     robotData->axisDirection[3]   );    
	body4_id = result->AddBody(body3_id, Xtrans(robotData->axisOrigin[3]), joint4, body4, "4");     

	body5 = Body(robotData->mass[4], robotData->com[4], robotData->inertia[4]);     
	joint5 = Joint (     JointTypeRevolute,     robotData->axisDirection[4]   );    
	body5_id = result->AddBody(body4_id, Xtrans(robotData->axisOrigin[4]), joint5, body5, "5");     

	body6 = Body(robotData->mass[5], robotData->com[5], robotData->inertia[5]);     
	joint6 = Joint (     JointTypeRevolute,     robotData->axisDirection[5]   );    
	body6_id = result->AddBody(body5_id, Xtrans(robotData->axisOrigin[5]), joint6, body6, "6");     

	body7 = Body(robotData->mass[6], robotData->com[6], robotData->inertia[6]);     
	joint7 = Joint (     JointTypeRevolute,     robotData->axisDirection[6]   );    
	body7_id = result->AddBody(body6_id, Xtrans(robotData->axisOrigin[6]), joint7, body7, "7");     

	std::cout << "LBR iiwa 14kg model built \n";
	std::cout << "BodyIDs: " << body1_id << ", " << body2_id << ", " << body3_id << ", " << body4_id << ", " << body5_id << ", " << body6_id << ", " << body7_id << "\n ";

	delete robotData;

	RobotModel* ret = new RobotModel();
	ret->m_model = result;
	return ret;
}

//dennis youBot========================================================================================================
RobotModel* RobotModel::createModelYouBot()
{
	std::cout << "makemodle " << "\n";
	unsigned int
		body_transx_id,
		body_transy_id,
		body_rot1z_id,
		body_rot2z_id,
		body_kip1_id,
		body_kip2_id,
		body_kip3_id,
		body_rot3z_id;

	Body
		body_transx,
		body_transy,
		body_rot1z,
		body_rot2z,
		body_kip1,
		body_kip2,
		body_kip3,
		body_rot3z;

	Joint
		joint_transx,
		joint_transy,
		joint_rot1z,
		joint_rot2z,
		joint_kip1,
		joint_kip2,
		joint_kip3,
		joint_rot3z;


	
	Model* result = new Model();
	std::cout << "makemodle0 " << "\n";
	RobotData* robotData = new RobotData(8);//youbot Dennis Data values_internet.h case 8
	std::cout << "makemodle1 " << "\n";
	result->gravity = Vector3d(0.0, 0.0, -9.81);
	std::cout << "makemodle1.5 " << "\n";

	////////////////////////////Base//////////////////////////////////////

	body_transx = Body(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));
	joint_transx = Joint(
		JointTypePrismatic,
		robotData->axisDirection[0]
		);

	body_transx_id = result->AddBody(0, Xtrans(robotData->axisOrigin[0]), joint_transx, body_transx);



	body_transy = Body(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));
	joint_transy = Joint(
		JointTypePrismatic,
		robotData->axisDirection[1]
		);

	body_transy_id = result->AddBody(body_transx_id, Xtrans(Vector3d(0., 0., 0.)), joint_transy, body_transy);




	body_rot1z = Body(robotData->mass[0], robotData->com[0], robotData->inertia[0]);
	joint_rot1z = Joint(
		JointTypeRevolute,
		robotData->axisDirection[2]
		);

	body_rot1z_id = result->AddBody(body_transy_id, Xtrans(Vector3d(0., 0., 0.)), joint_rot1z, body_rot1z);

	////////////////////////////Arm//////////////////////////////////////




	body_rot2z = Body(robotData->mass[1], robotData->com[1], robotData->inertia[1]);
	joint_rot2z = Joint(
		JointTypeRevolute,
		robotData->axisDirection[3]
		);

	body_rot2z_id = result->AddBody(body_rot1z_id, Xtrans(robotData->axisOrigin[1] + robotData->axisOrigin[2]), joint_rot2z, body_rot2z);




	body_kip1 = Body(robotData->mass[2], robotData->com[2], robotData->inertia[2]);
	joint_kip1 = Joint(
		JointTypeRevolute,
		robotData->axisDirection[4]
		);

	body_kip1_id = result->AddBody(body_rot2z_id, Xtrans(robotData->axisOrigin[3]), joint_kip1, body_kip1);




	body_kip2 = Body(robotData->mass[3], robotData->com[3], robotData->inertia[3]);
	joint_kip2 = Joint(
		JointTypeRevolute,
		robotData->axisDirection[5]
		);

	body_kip2_id = result->AddBody(body_kip1_id, Xtrans(robotData->axisOrigin[4]), joint_kip2, body_kip2);




	body_kip3 = Body(robotData->mass[4], robotData->com[4], robotData->inertia[4]);
	joint_kip3 = Joint(
		JointTypeRevolute,
		robotData->axisDirection[6]
		);

	body_kip3_id = result->AddBody(body_kip2_id, Xtrans(robotData->axisOrigin[5]), joint_kip3, body_kip3);

	//

	Matrix3d temp = Matrix3d::Zero(3,3);

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			temp(i,j) = robotData->inertia[5](i,j) + robotData->inertia[6](i,j);
		}
	}
	

	//mit endeff
	body_rot3z = Body((robotData->mass[5] + robotData->mass[6]), ((robotData->com[5] * robotData->mass[5] + robotData->com[6] * robotData->mass[6]) / (robotData->mass[5] + robotData->mass[6]) ) , temp);
	joint_rot3z = Joint(
		JointTypeRevolute,
		robotData->axisDirection[7]
		);

	body_rot3z_id = result->AddBody(body_kip3_id, Xtrans(robotData->axisOrigin[6]), joint_rot3z, body_rot3z);
	std::cout << "makemodel2 " << "\n";


	delete robotData;

	RobotModel* ret = new RobotModel();
	ret->m_model = result;
	return ret;
}
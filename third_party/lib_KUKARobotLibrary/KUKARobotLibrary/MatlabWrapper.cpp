

#include <rbdl/rbdl.h>
#include "MatlabWrapper.h"
#include "Robot.h"
#include <iostream>



using namespace RigidBodyDynamics::Math;
using namespace kuka;


__declspec(dllexport) void* createSnake(unsigned int* numLinksInput,double* linkLengthsInput, double* linkMassesInput)
{
	unsigned int numLinks = (unsigned int)*numLinksInput;
	VectorNd linkLengths = VectorNd::Zero(numLinks,1);
	VectorNd linkMasses = VectorNd::Zero(numLinks,1);

	for (int i = 0; i < numLinks; i++)
	{
		linkLengths[i] = linkLengthsInput[i];
		linkMasses[i] = linkMassesInput[i];
	}

	Robot::robotName myName(Robot::Snake);
	Robot* myRobot= new Robot(myName, numLinks, linkLengths, linkMasses);
	return reinterpret_cast<void*>(myRobot);

}

__declspec(dllexport) void* createIIwa7kg()
{
	Robot::robotName myName(Robot::LBRiiwa);
	Robot* myRobot= new Robot(myName);  
	return reinterpret_cast<void*>(myRobot);

}

__declspec(dllexport) void* createYouBot()
{
	Robot::robotName myName(Robot::YouBot);
	Robot* myRobot = new Robot(myName);
	return reinterpret_cast<void*>(myRobot);

}

__declspec(dllexport) void* createIIwa14kg()
{
	Robot::robotName myName(Robot::LBRiiwa14);		//LBRiiwa14
	Robot* myRobot= new Robot(myName);  
	return reinterpret_cast<void*>(myRobot);

}

__declspec(dllexport) void* createLBR4()
{
	Robot::robotName myName(Robot::LBR4);		//LBR4
	Robot* myRobot= new Robot(myName);  
	return reinterpret_cast<void*>(myRobot);

}

__declspec(dllexport) void getMassMatrix(void* robotInstance, double* MOutput, double* QInput)
{
	Robot* myRobot = reinterpret_cast<Robot*>(robotInstance);
	
	VectorNd Q = VectorNd::Zero(myRobot->getDOF(),1);   
	for (int i=0; i<(myRobot->getDOF()); i++)
	{
		Q[i] = QInput[i];
	} 
	

	// Get the mass matrix
	MatrixNd M = MatrixNd::Zero(myRobot->getDOF(),myRobot->getDOF());
	myRobot->getMassMatrix(M, Q);

	int k=0;

	for(int i=0; i<myRobot->getDOF(); i++)
	{
		for(int j=0; j<myRobot->getDOF(); j++)
		{
			MOutput[k] = M(i,j);
			k++;
		}
	}


	//for (int i=0; i<(myRobot->getDOF()* myRobot->getDOF()); i++)
	//{
	//	MOutput[i] = M[i];
	//}

	//delete myRobot;  // nicht notwendig, da reinterpret_cast kein neues Objekt erzeugt??

}



__declspec(dllexport) void getCoriolisAndGravityVector(void* robotInstance, double* cOutput, double* gOutput, double* QInput, double* QDotInput)
{
	Robot* myRobot = reinterpret_cast<Robot*>(robotInstance);

	VectorNd Q = VectorNd::Zero(myRobot->getDOF(),1);   
	VectorNd QDot = VectorNd::Zero(myRobot->getDOF(),1);  

	for (int i=0; i<(myRobot->getDOF()); i++)
	{
		Q[i] = QInput[i];
		QDot[i] = QDotInput[i];
	}
		
	// Get the vector of velocity-dependent and generalized gravity forces
	VectorNd g = VectorNd::Zero(myRobot->getDOF(),1);
	VectorNd c = VectorNd::Zero(myRobot->getDOF(),1);
	myRobot->getCoriolisAndGravityVector(c, g, Q, QDot);
	
	for (int i=0; i<myRobot->getDOF(); i++)
	{
		cOutput[i] = c[i];
		gOutput[i] = g[i];
	}

	//delete myRobot;

}


__declspec(dllexport) void getJacobian(void* robotInstance, double* JOutput, double* QInput, double* pointPositionInput, unsigned int* bodyIndexInput)
{
	Robot* myRobot = reinterpret_cast<Robot*>(robotInstance);

	VectorNd Q = VectorNd::Zero(myRobot->getDOF(), 1);   

	for (int i=0; i<myRobot->getDOF(); i++)
	{
		Q[i] = QInput[i];
	}

	// Get the Jacobian of any point on any body
	MatrixNd J = MatrixNd::Zero(6, myRobot->getDOF());
	unsigned int bodyIndex = (unsigned int)*bodyIndexInput;
	Vector3d pointPosition;
	pointPosition[0] = pointPositionInput[0];
	pointPosition[1] = pointPositionInput[1];
	pointPosition[2] = pointPositionInput[2];

	myRobot->getJacobian(J, Q, pointPosition, bodyIndex);
	
	int k=0;

	for (int i=0; i<6; i++)
	{
		for(int j=0; j< myRobot->getDOF(); j++)
		{
			JOutput[k] = J(i,j);
			k++;
		}
	}


	//for (int i=0; i<(6*myRobot->getDOF()); i++)
	//{
	//	JOutput[i] = J[i];
	//}

	//delete myRobot;



}

__declspec(dllexport) void getWorldCoordinates(void* robotInstance, double* xOutput, double* QInput, double* pointPositionInput, unsigned int* bodyIndexInput)
{
	Robot* myRobot = reinterpret_cast<Robot*>(robotInstance);
	unsigned int bodyIndex = (unsigned int)*bodyIndexInput;
	Vector3d pointPosition; 

	pointPosition[0] = pointPositionInput[0];
	pointPosition[1] = pointPositionInput[1];
	pointPosition[2] = pointPositionInput[2];

	VectorNd Q = VectorNd::Zero(myRobot->getDOF(), 1);
	for (int i = 0; i < myRobot->getDOF(); i++)
	{
		Q[i] = QInput[i];
	}


	// Get the x-end
	Vector3d x = Vector3d::Zero(3,1 );
	myRobot->getWorldCoordinates(x, Q, pointPosition, bodyIndex);

	for (int i = 0; i < 3; i++)
	{
		xOutput[i] = x[i];
	}
}
	//delete myRobot;  // nicht notwendig, da reinterpret_cast kein neues Objekt erzeugt??


__declspec(dllexport) void getRotationMatrix(void* robotInstance, double* ROutput, double* QInput, unsigned int* bodyIndexInput, double* finalOrientation)
{
	Robot* myRobot = reinterpret_cast<Robot*>(robotInstance);
	unsigned int bodyIndex = (unsigned int)*bodyIndexInput;


	VectorNd Q = VectorNd::Zero(myRobot->getDOF(), 1);
	for (int i = 0; i < myRobot->getDOF(); i++)
	{
		Q[i] = QInput[i];
	}

	Matrix3d R_final = Matrix3d::Zero(3, 3);
	int k=0;
	for (int i = 0; i < 3; i++)
	{
		for(int j=0; j<3; j++)
		{
			R_final(j,i) = finalOrientation[k];
			k++;
		}
	}


	// Get the Rotation
	Matrix3d R = Matrix3d::Zero(3, 3);
	myRobot->getRotationMatrix(R, Q, bodyIndex);
	// perform the final rotation
	R = R*R_final;

	k=0;
	for (int i = 0; i < 3; i++)
	{
		for(int j=0; j<3; j++)
		{
			ROutput[k] = R(i,j);
			k++;
		}
	}
}

__declspec(dllexport) void getDOF(void* robotInstance, int* DOF)
{
	Robot* myRobot = reinterpret_cast<Robot*>(robotInstance);
	*DOF = myRobot->getDOF();
}

__declspec(dllexport) void updateKinematics(void* robotInstance, double* QIn, double* QDotIn, double* QDotDotIn)
{
	Robot* myRobot = reinterpret_cast<Robot*>(robotInstance);

	VectorNd Q = VectorNd::Zero(myRobot->getDOF(), 1);
	for (int i = 0; i < myRobot->getDOF(); i++)
	{
		Q[i] = QIn[i];
	}

	VectorNd QDot = VectorNd::Zero(myRobot->getDOF(), 1);
	for (int i = 0; i < myRobot->getDOF(); i++)
	{
		QDot[i] = QDotIn[i];
	}

	VectorNd QDotDot = VectorNd::Zero(myRobot->getDOF(), 1);
	for (int i = 0; i < myRobot->getDOF(); i++)
	{
		QDotDot[i] = QDotDotIn[i];
	}

	myRobot->updateKinematics(Q,QDot,QDotDot);
}



	
	

	
	

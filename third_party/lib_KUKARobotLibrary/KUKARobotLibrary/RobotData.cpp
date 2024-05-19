#include "RobotData.h"

using namespace RigidBodyDynamics::Math;

RobotData::RobotData(int whatRobot, unsigned int numLinks, VectorNd& linkLengths, VectorNd& linkMasses)
{		
	switch (whatRobot)
	{
	case 1: // it's a Snake-robot
		{
			initializeVariables(numLinks);

			Vector3d positionOfFirstJointInWorldCoords = Vector3d(0,0,0);

			for (int i=0; i<numLinks; i++)
			{
				mass[i] = linkMasses[i];

				com[i] = Vector3d(0.5*linkLengths[i], 0.0, 0.0);

				double I_com = linkMasses[i] * pow(linkLengths[i],2)/12;

				std::cout << "I_com " << i << ": " << I_com << "\r\n";
				inertia[i] = Matrix3d(	0.0,	0.0,	0.0,
										0.0,	I_com,	0.0,
										0.0,	0.0,	I_com );

				if(i==0)
				{
					axisOrigin[i] = positionOfFirstJointInWorldCoords;
				}
				else
				{
					double x = linkLengths[i-1];
					axisOrigin[i] = Vector3d(x, 0.0, 0.0);
				}

				axisDirection[i] = Vector3d(0.0, 0.0, 1.0);
			}

			std::cout << "Data for " << numLinks << "-Dof robot generated \n";

			break;
		}

	//###################################################
	//###################################################
	//case 1: // it's a Snake-robot
		//{
		//	initializeVariables(3);

		//	mass[0] = 1;
		//	mass[1] = 1;
		//	mass[2] = 1;
		//				
		//	com[0] = Vector3d(0.5, 0.0, 0.0);
		//	com[1] = Vector3d(0.5, 0.0, 0.0);
		//	com[2] = Vector3d(0.5, 0.0, 0.0);	
		//	
		//	inertia[0] = Matrix3d(0.0, 0.0, 0.0, 
		//		                  0.0, 0.083, 0.0,
		//						  0.0, 0.0, 0.083);
		//	
		//	inertia[1] = Matrix3d(0.0, 0.0, 0.0, 
		//		                  0.0, 0.083, 0.0,
		//						  0.0, 0.0, 0.083);

		//	inertia[2] = Matrix3d(0.0, 0.0, 0.0, 
		//		                  0.0, 0.083, 0.0,
		//						  0.0, 0.0, 0.083);

		//	Vector3d positionOfFirstJointInWorldCoords = Vector3d(0,0,0);

		//	axisOrigin[0] = positionOfFirstJointInWorldCoords; 
		//	axisOrigin[1] = Vector3d(1.0, 0.0, 0.0);
		//	axisOrigin[2] = Vector3d(1.0, 0.0, 0.0);

		//	axisDirection[0] = Vector3d(0.0, 0.0, 1.0);
		//	axisDirection[1] = Vector3d(0.0, 0.0, 1.0);
		//	axisDirection[2] = Vector3d(0.0, 0.0, 1.0);

		//	std::cout << "Inertia data for TwoDof robot generated \n";

		//	break;
		//}

	}

}


RobotData::RobotData(int whatRobot)
{	
	
	switch (whatRobot)
	{
	case 2: // it's a 2Dof pendulum
		{

			initializeVariables(2);

			mass[0] = 2;
			mass[1] = 3;
						
			com[0] = Vector3d(0.75, 0.0, 0.0);
			com[1] = Vector3d(0.9, 0.0, 0.0);	
			
			inertia[0] = Matrix3d(0.0, 0.0, 0.0, 
				                  0.0, 5.0, 0.0,
								  0.0, 0.0, 0.0);
			
			inertia[1] = Matrix3d(0.0, 0.0, 0.0, 
				                  0.0, 4.0, 0.0,
								  0.0, 0.0, 0.0);

			Vector3d positionOfFirstJointInWorldCoords = Vector3d(1,0,0);
			double l1 = 1.5;

			axisOrigin[0] = positionOfFirstJointInWorldCoords; 
			axisOrigin[1] = Vector3d(l1, 0.0, 0.0);

			axisDirection[0] = Vector3d(0.0, 1.0, 0.0);
			axisDirection[1] = Vector3d(0.0, 1.0, 0.0);

			std::cout << "Inertia data for TwoDof robot generated \n";

			break;
		}
	case 3: // it's a sample Robot
		{
			
			initializeVariables(3);

			mass[0] = 1;
			mass[1] = 1;
			mass[2] = 0;
						
			com[0] = Vector3d(0.5, 0.0, 0.0);
			com[1] = Vector3d(0.0, 0.5, 0.0);
			com[2] = Vector3d(0.5, 0.0, 0.0);
			
			
			inertia[0] = Matrix3d(1.0, 0.0, 0.0, 
				                  0.0, 1.0, 0.0,
								  0.0, 0.0, 1.0);
			
			inertia[1] = Matrix3d(1.0, 0.0, 0.0, 
				                  0.0, 1.0, 0.0,
								  0.0, 0.0, 1.0);

			inertia[2] = Matrix3d(1.0, 0.0, 0.0, 
				                  0.0, 1.0, 0.0,
								  0.0, 0.0, 1.0);

			axisOrigin[0] = Vector3d(0.0, 0.0, 0.0);
			axisOrigin[1] = Vector3d(1.0, 0.0, 0.0);
			axisOrigin[2] = Vector3d(0.0, 1.0, 0.0);


			axisDirection[0] = Vector3d(0.0, 0.0, 1.0);
			axisDirection[1] = Vector3d(0.0, 0.0, 1.0);
			axisDirection[2] = Vector3d(0.0, 0.0, 1.0);


			std::cout << "Inertia data for SampleRobot generated \n";

			break;
		}

	case 4: // it's an LBR4
		{
			initializeVariables(7);

			// This is the data that I adapted from Init_LBR_simpl.m
			axisOrigin[0] = Vector3d(0,   0,    310.0e-3);    // axis 1
			axisOrigin[1] = Vector3d(0,   0,      0);
			axisOrigin[2] = Vector3d(0,   0,	400.0e-3);
			axisOrigin[3] = Vector3d(0,   0,      0);
			axisOrigin[4] = Vector3d(0,	  0,	390.0e-3);
			axisOrigin[5] = Vector3d(0,   0,      0);
			axisOrigin[6] = Vector3d(0,   0,      0);
			
			axisDirection[0] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[1] = Vector3d(0.0,  1.0,  0.0);
			axisDirection[2] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[3] = Vector3d(0.0, -1.0,  0.0);
			axisDirection[4] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[5] = Vector3d(0.0,  1.0,  0.0);
			axisDirection[6] = Vector3d(0.0,  0.0,  1.0);

			
			mass[0] = 2.708;
			mass[1] = 2.710;
			mass[2] = 2.5374;
			mass[3] = 2.5053;
			mass[4] = 1.3028;
			mass[5] = 1.5686;
			mass[6] = 0.1943;
			

			com[0] = Vector3d(  0.0, -16.98e-3,  -59.1e-3);  
			com[1] = Vector3d(  0.0, +14.10e-3,  110.9e-3);  
			com[2] = Vector3d(  0.0,  16.28e-3,  -66.21e-3);  
			com[3] = Vector3d(  0.0, -15.25e-3,	+105.38e-3);
			com[4] = Vector3d(  0.0, -15.66e-3, -125.11e-3);  
			com[5] = Vector3d(  0.0,  -2.28e-3,    2.83e-3);  
			com[6] = Vector3d(  0.0,   0.0e-3,    60.31e-3);  
			

			inertia[0] = Matrix3d(0.0216417, 0.0,   0.0, 
				                  0.0 , 0.0214810, -0.0022034,
								  0.0, -0.0022034,   0.0049639);

			inertia[1] = Matrix3d(0.0244442, 0.0,    0.0, 
				                  0.0,   0.0239951,  0.0036944,
								  0.0,   0.0036944,  0.0052508);

			inertia[2] = Matrix3d(0.0213026,  0.0,    0.0, 
				                  0.0,  0.0210353,   0.0022204,
								  0.0,  0.0022204,   0.0046970);

			inertia[3] = Matrix3d(0.0231668,  0.0,    0.0, 
				                  0.0,	0.0227509,    0.0,
								  0.0,		  0.0,   0.0048331);

			inertia[4] = Matrix3d(0.0081391,  0.0,    0.0, 
				                  0.0,    0.0075015,  -0.0021299,
								  0.0,    -0.0021299,    0.0030151);

			inertia[5] = Matrix3d(0.0033636,  0.0,     0.0, 
				                  0.0,    0.0029876,   0.0,
								  0.0,    0.0,     0.0029705);

			inertia[6] = Matrix3d(0.0000793, 0.0,     0.0, 
				                  0.0,    0.0000783,  0.0,
								  0.0,    0.0,     0.0001203);


			std::cout << "Inertia data for LBR4 generated \n";

			break;
		}

	case 7: // it's a 7kg iiwa
		{

			initializeVariables(7);

		    // **************** This is the data that I adapted from Init_LBR_simpl.m
			axisOrigin[0] = Vector3d(0,   0,    152.5e-3);    // axis 1
			axisOrigin[1] = Vector3d(0, -11e-3, 187.5e-3);
			axisOrigin[2] = Vector3d(0, +11e-3, 212.5e-3);
			axisOrigin[3] = Vector3d(0, +11e-3, 187.5e-3);
			axisOrigin[4] = Vector3d(0, -11e-3, 212.5e-3);
			axisOrigin[5] = Vector3d(0, -62e-3, 187.5e-3);
			axisOrigin[6] = Vector3d(0, +62e-3,  79.6e-3);
			
			axisDirection[0] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[1] = Vector3d(0.0,  1.0,  0.0);
			axisDirection[2] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[3] = Vector3d(0.0, -1.0,  0.0);
			axisDirection[4] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[5] = Vector3d(0.0,  1.0,  0.0);
			axisDirection[6] = Vector3d(0.0,  0.0,  1.0);

			
			mass[0] = 2.7426;
			mass[1] = 4.9464;
			mass[2] = 2.5451;
			mass[3] = 4.6376;
			mass[4] = 1.7140;
			mass[5] = 2.4272;
			mass[6] = 0.4219;
			

			com[0] = Vector3d(  0.0   , -18.7e-3,  101.6e-3);  
			com[1] = Vector3d( -0.21e-3,  25.0e-3,   82.5e-3);  
			com[2] = Vector3d( -0.20e-3,  19.5e-3,   98.4e-3);  
			com[3] = Vector3d( -0.21e-3, -20.1e-3,   86.0e-3);
			com[4] = Vector3d( -0.04e-3, -13.5e-3,   66.0e-3);  
			com[5] = Vector3d( -0.35e-3,  51.4e-3,   17.1e-3);  
			com[6] = Vector3d( -0.01e-3,   0.1e-3,   11.0e-3);  
			

			inertia[0] = Matrix3d(0.24, 0.0,   0.0, 
				                  0.0 , 0.024, 0.0,
								  0.0,  0.0,   0.0128);

			inertia[1] = Matrix3d(0.0468, 0.0,    0.0, 
				                  0.0,   0.0282, 0.0,
								  0.0,   0.0,    0.0101);

			inertia[2] = Matrix3d(0.02,  0.0,    0.0, 
				                  0.0,   0.02,   0.0,
								  0.0,   0.0,    0.06);

			inertia[3] = Matrix3d(0.04,  0.0,    0.0, 
				                  0.0,   0.027,   0.0,
								  0.0,   0.0,    0.01);

			inertia[4] = Matrix3d(0.019,  0.0,    0.0, 
				                  0.0,    0.016,  0.0,
								  0.0,    0.0,    0.012);

			inertia[5] = Matrix3d(0.007,  0.0,     0.0, 
				                  0.0,    0.006,   0.0,
								  0.0,    0.0,     0.005);

			inertia[6] = Matrix3d(0.0003, 0.0,     0.0, 
				                  0.0,    0.0003,  0.0,
								  0.0,    0.0,     0.0005);



			/*
			// **************** This is the data from ChainData.xml and AxisData.xml
			axisOrigin[0] = Vector3d(0,   0,    152.5e-3);    // axis 1
			axisOrigin[1] = Vector3d(0, -11e-3, 187.5e-3);
			axisOrigin[2] = Vector3d(0, +11e-3, 212.5e-3);
			axisOrigin[3] = Vector3d(0, +11e-3, 187.5e-3);
			axisOrigin[4] = Vector3d(0, -11e-3, 212.5e-3);
			axisOrigin[5] = Vector3d(0, -62e-3, 187.5e-3);
			axisOrigin[6] = Vector3d(0, +62e-3,  79.6e-3);
			
			axisDirection[0] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[1] = Vector3d(0.0,  1.0,  0.0);
			axisDirection[2] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[3] = Vector3d(0.0, -1.0,  0.0);
			axisDirection[4] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[5] = Vector3d(0.0,  1.0,  0.0);
			axisDirection[6] = Vector3d(0.0,  0.0,  1.0);

			
			mass[0] = 2.74;
			mass[1] = 4.87;
			mass[2] = 2.65;
			mass[3] = 4.70;
			mass[4] = 1.66;
			mass[5] = 2.51;
			mass[6] = 0.427;
			

			com[0] = Vector3d(  0.0   , -18.7e-3,  101.6e-3);  
			com[1] = Vector3d( -0.2e-3,  30.6e-3,   82.9e-3);  
			com[2] = Vector3d( -0.2e-3,  22.6e-3,  126.0e-3);  
			com[3] = Vector3d( -0.2e-3, -19.4e-3,  101.0e-3);
			com[4] = Vector3d( -0.0e-3, -13.0e-3,   71.0e-3);  
			com[5] = Vector3d( -0.4e-3,  56.9e-3,   16.0e-3);  
			com[6] = Vector3d( -0.0e-3,   0.1e-3,   10.1e-3);  
			

			inertia[0] = Matrix3d(0.24, 0.0,   0.0, 
				                  0.0 , 0.024, 0.0,
								  0.0,  0.0,   0.006);

			inertia[1] = Matrix3d(0.046, 0.0,    0.0, 
				                  0.0,   0.0454, 0.0,
								  0.0,   0.0,    0.01);

			inertia[2] = Matrix3d(0.02,  0.0,    0.0, 
				                  0.0,   0.02,   0.0,
								  0.0,   0.0,    0.05);

			inertia[3] = Matrix3d(0.04,  0.0,    0.0, 
				                  0.0,   0.04,   0.0,
								  0.0,   0.0,    0.01);

			inertia[4] = Matrix3d(0.017,  0.0,    0.0, 
				                  0.0,    0.015,  0.0,
								  0.0,    0.0,    0.005);

			inertia[5] = Matrix3d(0.007,  0.0,     0.0, 
				                  0.0,    0.006,   0.0,
								  0.0,    0.0,     0.004);

			inertia[6] = Matrix3d(0.0003, 0.0,     0.0, 
				                  0.0,    0.0003,  0.0,
								  0.0,    0.0,     0.0005);

			*/

			std::cout << "Inertia data for LBR iiwa 7kg generated \n";

			break;

		}
	case 8: // it's youBot Dennis values_internet.h
	{
		
		initializeVariables(8);
		std::cout << "setdata " << "\n";
		axisOrigin[0] = Vector3d(0.,0.,0.084);         //start
		std::cout << "setdata " << "\n";
		axisOrigin[1] = Vector3d(0.143, 0., 0.046 );     //lb
		axisOrigin[2] = Vector3d(0.024, 0., 0.115);     //l1
		axisOrigin[3] = Vector3d(0.033, 0., 0.);         //l2
		axisOrigin[4] = Vector3d(0., 0., 0.155);         //l3
		axisOrigin[5] = Vector3d(0., 0., 0.135);         //l4
		axisOrigin[6] = Vector3d(0., 0., 0.1136);         //l5
		std::cout << "setdata " << "\n";


		axisOrigin[7] = Vector3d(0., 0., 0.05716);         //lf
		
		std::cout << "setdata l " << "\n";

		mass[0] = 25.403; //base
		mass[1] = 2.351;  //armdreh2
		mass[2] = 1.318; //armkipp1
		mass[3] = 0.821; //armkipp2
		mass[4] = 0.769; //armkipp3
		mass[5] = 0.687; //armdreh3
		mass[6] = 0.219; //greifer

		std::cout << "setdata m " << "\n";

		com[0] = Vector3d(0., 0., 0.);   //cb
		com[1] = Vector3d(0.01516, 0.00359, -0.03105);   //c1
		com[2] = Vector3d(-0.01903, 0.0150, 0.11397);   //c2
		com[3] = Vector3d(0.00013, 0.02022, 0.10441);   //c3
		com[4] = Vector3d(0.00015, -0.02464, 0.05353);   //c4
		com[5] = Vector3d(0., 0.0012, 0.01648);   //c5
		com[6] = Vector3d(0., 0., 0.0289);   //cg

		std::cout << "setdata c " << "\n";

		double Ibx = 1.;
		double Iby = 1.;
		double Ibz = 1.;
		double I1x = 0.0029525;
		double I1y = 0.0060091;
		double I1z = 0.0058821;
		double I2x = 0.0031145;
		double I2y = 0.0005843;
		double I2z = 0.0031631;
		double I3x = 0.00172767;
		double I3y = 0.00041967;
		double I3z = 0.0018468;
		double I4x = 0.0006764;
		double I4y = 0.0010573;
		double I4z = 0.0006610;
		double I5x = 0.0001934;
		double I5y = 0.0001602;
		double I5z = 0.0000689;
		double Igx = 0.0002324;
		double Igy = 0.0003629;
		double Igz = 0.0002067;

		std::cout << "setdata I1 " << "\n";


		inertia[0] = Matrix3d(Ibx, 0.0, 0.0,
							0.0, Iby, 0.0,
							0.0, 0.0, Ibz);

		inertia[1] = Matrix3d(I1x, 0.0, 0.0,
							0.0, I1y, 0.0,
							0.0, 0.0, I1z);

		inertia[2] = Matrix3d(I2x, 0.0, 0.0,
							0.0, I2y, 0.0,
							0.0, 0.0, I2z);

		inertia[3] = Matrix3d(I3x, 0.0, 0.0,
							0.0, I3y, 0.0,
							0.0, 0.0, I3z);

		inertia[4] = Matrix3d(I4x, 0.0, 0.0,
							0.0, I4y, 0.0,
							0.0, 0.0, I4z);

		inertia[5] = Matrix3d(I5x, 0.0, 0.0,
							0.0, I5y, 0.0,
							0.0, 0.0, I5z);

		inertia[6] = Matrix3d(Igx, 0.0, 0.0,
								0.0, Igy, 0.0,
								0.0, 0.0, Igz);



		std::cout << "setdata I2 " << "\n";


		
	//	eth
	/*	axisDirection[0] = Vector3d(1., 0., 0.);
		axisDirection[1] = Vector3d(0., 1., 0.);
		axisDirection[2] = Vector3d(0., 0., 1.);
		axisDirection[3] = Vector3d(0., 0., 1.);
		axisDirection[4] = Vector3d(0., -1., 0.);
		axisDirection[5] = Vector3d(0., 1., 0.);
		axisDirection[6] = Vector3d(0., -1., 0.);
		axisDirection[7] = Vector3d(0., 0., 1.); */

		axisDirection[0] = Vector3d(1., 0., 0.);
		axisDirection[1] = Vector3d(0., 1., 0.);
		axisDirection[2] = Vector3d(0., 0., 1.);
		axisDirection[3] = Vector3d(0., 0., 1.);
		axisDirection[4] = Vector3d(0., 1., 0.);
		axisDirection[5] = Vector3d(0., 1., 0.);
		axisDirection[6] = Vector3d(0., 1., 0.);
		axisDirection[7] = Vector3d(0., 0., 1.);
		std::cout << "setdata ori " << "\n";
		std::cout << "Inertia data for youBot Dennis values_intenet \n";

			break;
	}
		case 14: // it's a 14kg iiwa
		{

			initializeVariables(7);

		    // **************** data taken from AxisData.xml/ChainData.xml, adapted to the upright robot
			axisOrigin[0] = Vector3d(0.0,   0.0,    152.5e-3);    // axis 1
			axisOrigin[1] = Vector3d(0.0, -13.0e-3, 207.5e-3);
			axisOrigin[2] = Vector3d(0.0, +13.0e-3, 232.5e-3);
			axisOrigin[3] = Vector3d(0.0, +11.0e-3, 187.5e-3);
			axisOrigin[4] = Vector3d(0.0, -11.0e-3, 212.5e-3);
			axisOrigin[5] = Vector3d(0.0, -62.0e-3, 187.5e-3);
			axisOrigin[6] = Vector3d(0.0, +62.0e-3,  79.6e-3);
			
			axisDirection[0] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[1] = Vector3d(0.0,  1.0,  0.0);
			axisDirection[2] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[3] = Vector3d(0.0, -1.0,  0.0);
			axisDirection[4] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[5] = Vector3d(0.0,  1.0,  0.0);
			axisDirection[6] = Vector3d(0.0,  0.0,  1.0);

			
			mass[0] = 6.404;
			mass[1] = 7.89;
			mass[2] = 2.54;
			mass[3] = 4.82;
			mass[4] = 1.76;
			mass[5] = 2.5;
			mass[6] = 0.42;
			

			com[0] = Vector3d( 0.0,	-14.0e-3,  102.0e-3);  
			com[1] = Vector3d( 0.0,  16.0e-3,   64.0e-3);  
			com[2] = Vector3d( 0.0,  19.0e-3,   98.0e-3);  
			com[3] = Vector3d( 0.0, -20.0e-3,   86.0e-3);
			com[4] = Vector3d( 0.0, -13.0e-3,   66.0e-3);  
			com[5] = Vector3d( 0.0,  60.0e-3,   16.0e-3);  
			com[6] = Vector3d( 0.0,   0.0e-3,   11.0e-3);  
			

			inertia[0] = Matrix3d(0.069, 0.0,   0.0, 
				                  0.0 , 0.071, 0.0,
								  0.0,  0.0,   0.02);

			inertia[1] = Matrix3d(0.08, 0.0,    0.0, 
				                  0.0,   0.08, 0.0,
								  0.0,   0.0,    0.01);

			inertia[2] = Matrix3d(0.02,  0.0,    0.0, 
				                  0.0,   0.02,   0.0,
								  0.0,   0.0,    0.06);

			inertia[3] = Matrix3d(0.04,  0.0,    0.0, 
				                  0.0,   0.03,   0.0,
								  0.0,   0.0,    0.01);

			inertia[4] = Matrix3d(0.01,  0.0,    0.0, 
				                  0.0,    0.01,  0.0,
								  0.0,    0.0,    0.01);

			inertia[5] = Matrix3d(0.007,  0.0,     0.0, 
				                  0.0,    0.006,   0.0,
								  0.0,    0.0,     0.005);

			inertia[6] = Matrix3d(0.0003, 0.0,     0.0, 
				                  0.0,    0.0003,  0.0,
								  0.0,    0.0,     0.0005);


			std::cout << "Inertia data for LBR iiwa 14kg generated \n";

			break;

		}
		case 77: // it's a 14kg iiwa
		{

			initializeVariables(7);

		    // **************** data taken from AxisData.xml/ChainData.xml, adapted to the upright robot
			axisOrigin[0] = Vector3d(0,   0,    152.5e-3);    // axis 1
			axisOrigin[1] = Vector3d(0, -11e-3, 187.5e-3);
			axisOrigin[2] = Vector3d(0, +11e-3, 212.5e-3);
			axisOrigin[3] = Vector3d(0, +11e-3, 187.5e-3);
			axisOrigin[4] = Vector3d(0, -11e-3, 212.5e-3);
			axisOrigin[5] = Vector3d(0, -62e-3, 187.5e-3);
			axisOrigin[6] = Vector3d(0, +62e-3,  79.6e-3);
			
			axisDirection[0] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[1] = Vector3d(0.0,  1.0,  0.0);
			axisDirection[2] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[3] = Vector3d(0.0, -1.0,  0.0);
			axisDirection[4] = Vector3d(0.0,  0.0,  1.0);
			axisDirection[5] = Vector3d(0.0,  1.0,  0.0);
			axisDirection[6] = Vector3d(0.0,  0.0,  1.0);

			
			mass[0] = 3.94781; //3.94781 4.50275 2.45520 2.61155 3.41000 3.38795 0.35432
			mass[1] = 4.50275;
			mass[2] = 2.45520;
			mass[3] = 2.61155;
			mass[4] = 3.41000;
			mass[5] = 3.38795;
			mass[6] =  0.35432;
			

			com[0] = Vector3d(  0.0   , -18.7e-3,  101.6e-3);  
			com[1] = Vector3d( -0.21e-3,  25.0e-3,   82.5e-3);  
			com[2] = Vector3d( -0.20e-3,  19.5e-3,   98.4e-3);  
			com[3] = Vector3d( -0.21e-3, -20.1e-3,   86.0e-3);
			com[4] = Vector3d( -0.04e-3, -13.5e-3,   66.0e-3);  
			com[5] = Vector3d( -0.35e-3,  51.4e-3,   17.1e-3);  
			com[6] = Vector3d( -0.01e-3,   0.1e-3,   11.0e-3);  
			

			inertia[0] = Matrix3d(0.00455, 0.0,   0.00001, 
				                  0.0 , 0.00454, 0.0,
								  0.00001,  0.0,   0.00029);

			inertia[1] = Matrix3d(0.0468, 0.0,    0.0, 
				                  0.0,   0.0282, 0.0,
								  0.0,   0.0,    0.0101);

			inertia[2] = Matrix3d(0.02,  0.0,    0.0, 
				                  0.0,   0.02,   0.0,
								  0.0,   0.0,    0.06);

			inertia[3] = Matrix3d(0.04,  0.0,    0.0, 
				                  0.0,   0.027,   0.0,
								  0.0,   0.0,    0.01);

			inertia[4] = Matrix3d(0.019,  0.0,    0.0, 
				                  0.0,    0.016,  0.0,
								  0.0,    0.0,    0.012);

			inertia[5] = Matrix3d(0.007,  0.0,     0.0, 
				                  0.0,    0.006,   0.0,
								  0.0,    0.0,     0.005);

			inertia[6] = Matrix3d(0.0003, 0.0,     0.0, 
				                  0.0,    0.0003,  0.0,
								  0.0,    0.0,     0.0005);


			std::cout << "Inertia data for LBR iiwa 7kg generated \n";

			break;

		}
	}

}

RobotData::~RobotData()
{
	delete[] mass; 
	delete[] com;
	delete[] inertia;

	delete[] axisOrigin;
	delete[] axisDirection;
}

void RobotData::initializeVariables(unsigned int numAxes)
{
	mass = new double[numAxes];
	com = new Vector3d[numAxes];
	inertia = new Matrix3d[numAxes];

	axisOrigin = new Vector3d[numAxes];
	axisDirection = new Vector3d[numAxes];
}

/*
:
mass[numAxes],
com[numAxes],
inertia[numAxes],
*/






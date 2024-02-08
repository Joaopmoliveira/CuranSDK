#include "obA.h"

ObstacleAvoidance::ObstacleAvoidance(RobotParameters* robotParam, double radiusOfObstacle, double actParameterRadius, double radiusOfHullAroundRobotJoints, Vector3d& centerOfObstacle)
{
	noOfExceededCartesianDirections = 0;
	noOfExceededPoints = 0;
	positionVector = 0.0;
	for (int iii= 0; iii< LBR_N_JOINTS; iii++)
	{
		if (iii <3)
		{
			exceededCartIndex[iii] = -1;
		}
		exceededPointIndex[iii] = -1;
		positionVectorArray[iii] = 0.0;
	}

	rOb = radiusOfObstacle;
	rAct = actParameterRadius;
	rHull = radiusOfHullAroundRobotJoints;
	xCenter = centerOfObstacle;

	xRobotPoint = Vector3d::Zero(3,1);
	xDotRobotPoint = Vector3d::Zero(3,1);
	
	cartRepulsionForce_i = VectorNd::Zero(2,1);

	torque_star_excluding_ob = VectorNd::Zero(LBR_N_JOINTS, 1);

	J_ob = MatrixNd::Zero(6,LBR_N_JOINTS);
	JRobotPoint = MatrixNd::Zero(6,LBR_N_JOINTS);
	constraintNullspace = MatrixNd::Identity(LBR_N_JOINTS, LBR_N_JOINTS);

	iiwa14 = robotParam;
}

ObstacleAvoidance::~ObstacleAvoidance()
{
}

int ObstacleAvoidance::isObstacleDetected()
{
	int counter=0;

	noOfExceededCartesianDirections=2; //Robot exceeds obstacle direction in both x- and y, but not in z
	obstacleDetected=0;

	//Robot exceeds obstacle direction in both x- and y, but not in z
	exceededCartIndex[0] = 0;
	exceededCartIndex[1] = 1;
	exceededCartIndex[2] = -1;

	for (int i=0; i< LBR_N_JOINTS; i++)
	{
		exceededPointIndex[i] = -1;
	}

	// Run only for x- and y- directions
				
	for (int i=0; i< LBR_N_JOINTS; i++)
	{
		positionVectorArray[i] = sqrt(pow(iiwa14->xRobotPoints(i,0)-xCenter(0),2) + pow(iiwa14->xRobotPoints(i,1)-xCenter(1) , 2));
		if ( positionVectorArray[i] <= (rOb + rAct + rHull) ) 
		{
			obstacleDetected=1; 
			exceededPointIndex[counter] = i;
			counter++;

		}
		//else
		//{
		//	if (!obstacleDetected) //To prevent cases where obstacle were detected in previous timestep, but not in the current timestep
		//		obstacleDetected=0;
		//}
	}

	noOfExceededPoints = counter;
	jacobian.resize(noOfExceededPoints*2,LBR_N_JOINTS);
	lambda.resize(noOfExceededPoints*2, noOfExceededPoints*2);
	force_tilde_i.resize(noOfExceededPoints*2,1);
	//positionVectorElbow = sqrt(pow(xRobotPoint(0)-xCenter(0),2) + pow(xRobotPoint(1)-xCenter(1) , 2));
	//if ( positionVectorElbow <= (rOb + rAct + rHull) ) 
	//{
	//	obstacleDetected=1;
	//	
	//}
	//else
	//{
	//		//obstacleDetected &= 1;
	//		obstacleDetected = 0;
	//}

	
	if (obstacleDetected==0)
	{
		torque = VectorNd::Zero(LBR_N_JOINTS,1);
		nullspace = MatrixNd::Identity(LBR_N_JOINTS, LBR_N_JOINTS);
	}

	return obstacleDetected;
}

void ObstacleAvoidance::calcObstacleAvoidanceTorques()
{
	int ctr=0, index=0, index2=0;

	for (int i=0; i< LBR_N_JOINTS; i++)
	{
		if ( i == exceededPointIndex[ctr] )
		{
			
			ctr++;
			
			positionVector = positionVectorArray[i];
			

			xRobotPoint(0) = iiwa14->xRobotPoints(i,0);
			xRobotPoint(1) = iiwa14->xRobotPoints(i,1);
			xRobotPoint(2) = iiwa14->xRobotPoints(i,2);

			
			xDotRobotPoint(0) = iiwa14->xDotRobotPoints(i,0);
			xDotRobotPoint(1) = iiwa14->xDotRobotPoints(i,1);
			xDotRobotPoint(2) = iiwa14->xDotRobotPoints(i,2);

			
			JRobotPoint.row(0) = iiwa14->jacobiansOfRobotPoints.row(index);
			JRobotPoint.row(1) = iiwa14->jacobiansOfRobotPoints.row(index+1);
			JRobotPoint.row(2) = iiwa14->jacobiansOfRobotPoints.row(index+2);
			JRobotPoint.row(3) = zeroVector.row(0);
			JRobotPoint.row(4) = zeroVector.row(0);
			JRobotPoint.row(5) = zeroVector.row(0);
			

			calcObstacleAvoidanceParameters();

			jacobian.row(index2) = J_ob.row(0);
			jacobian.row(index2+1) = J_ob.row(1);

			force_tilde_i(index2) = cartRepulsionForce_i(0);
			force_tilde_i(index2+1) = cartRepulsionForce_i(1);

			
			index2 += 2;
		}

		index+=3;
	}

	jacobianTranspose = jacobian.transpose();

	lambda = (jacobian * constraintNullspace.transpose() * iiwa14->Minv * constraintNullspace * jacobianTranspose).inverse();

	getInverseJacobian(jbar, jacobianTranspose, lambda);

}

void ObstacleAvoidance::calcObstacleRepulsionForces()
{
	int repulsionType=0; //0 - firas, 1 - felix
	//double kvr_firas = 0.005;
	//double eta[3] = {0.0070, 0.007, 0.007};
	double kvr_firas = 0.005;
	double eta[3] = {0.0070, 0.007, 0.007};
	double felix_constant[3] ={160.0, 160.0, 160.0};
	double kvr_felix = 50.0;
	int t;
//	double rho_i;
	int ctr = 0;
	double lambda;
	double fDirection[2] = {0.0,0.0};
	double distMod, dist, dist2;
	lambda = iiwa14->qDot.transpose() * iiwa14->M * iiwa14->qDot;

	cartRepulsionForce_i.resize(noOfExceededCartesianDirections,1);
	force_star.resize(noOfExceededCartesianDirections,1);
	force_tilde.resize(noOfExceededCartesianDirections,1);
	force_tilde_i.resize(noOfExceededCartesianDirections,1);
	h.resize(noOfExceededCartesianDirections,1);

	for(int i=0; i<noOfExceededCartesianDirections; i++)
	{
		cartRepulsionForce_i(i) = 0.0;
		force_star(i) = 0.0;
		force_tilde(i) = 0.0;
		force_tilde_i(i) = 0.0;
		h(i) = 0.0;
	}

	distMod = positionVector;
	dist =  distMod - rOb -rHull;
	dist2 = rAct - dist;
	fDirection[0] = (xRobotPoint(0) - xCenter(0))/ distMod;
	fDirection[1] = (xRobotPoint(1) - xCenter(1))/ distMod;
	for (int i=0; i<noOfExceededCartesianDirections; i++)
	{
		t= exceededCartIndex[ctr];
		ctr++;
		if ( positionVector <= (rOb+rAct+rHull) )
		{
			
			//Define Force based on Rep Type
			if (repulsionType==0)
			{
				//rho_i = xRobotPoint(t) - (xForbidden(t));
				force_star(i) = fDirection[t] * (eta[t] * ( (1/dist - 1/rAct) ) * (1/pow(dist,2)) - kvr_firas*xDotRobotPoint(t));//This side, to check
			}
			else
			{
				//rho_i = xRobotPoint(t) - xForbidden(t);
				force_star(i) = fDirection[t] * (felix_constant[t] * lambda * (dist) - kvr_felix*xDotRobotPoint(t)); //This side, to check
			}
			
			if (  positionVector <= (rOb+rHull) )
			{
				h(i) = 1.0;
			}
			else if (  positionVector <= (rOb+rAct+rHull) )
			{
				h(i) = 0.5 + 0.5* sin ( (M_PI/rAct * (distMod - rOb -rHull ) +M_PI/2.0) );
			}
			else
			{
				h(i) = 0.0;
			}
		}
		else
		{
			force_star(i) = 0.0;
			h(i) = 0.0;
		}
	}

}

void ObstacleAvoidance::calcObstacleAvoidanceParameters()
{
	calcObstacleRepulsionForces();
	
	J_ob.resize(noOfExceededCartesianDirections,LBR_N_JOINTS);

	J_ob.row(0) = JRobotPoint.row(0);
	J_ob.row(1) = JRobotPoint.row(1);

	if (gravityToggle==1)
	{
		p = J_ob * iiwa14->Minv * iiwa14->g;
		force_star += p;
	}

	for (int i=0;i<noOfExceededCartesianDirections;i++)
	{
		cartRepulsionForce_i[i]= h[i] * force_star[i] + (1 - h[i]) * J_ob.row(i) * iiwa14->Minv * torque_star_excluding_ob;
	}	

}

void ObstacleAvoidance::calcTorqueAndNullspace(VectorNd& torque_star_excluding_ob, MatrixNd& constraintNullspace)
{
	this->torque_star_excluding_ob = torque_star_excluding_ob;
	this->constraintNullspace = constraintNullspace;


	if (obstacleDetected == 1)
	{
		calcObstacleAvoidanceTorques();
		torque = jacobianTranspose * lambda * force_tilde_i ;
	
		getNullspace(nullspace,jacobianTranspose, jbar);
	}
	else
	{
		torque = this->torque;
		nullspace = this->nullspace;
	}
}
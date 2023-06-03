#include "vwA.h"

VirtualWallAvoidance::VirtualWallAvoidance(RobotParameters* robotParam, Vector3d& virtualWallActivationZone, Vector3d& virtualWallPosition)
{
	noOfExceededWallDirections = 0;
	virtualWallExceeded = 0;

	for (int iii=0; iii<2; iii++)
	{
		exceededWallDirectionIndex[iii] = -1;
	}

	xActivation = virtualWallActivationZone;
	xForbidden = virtualWallPosition;

	xBuffer(0) = abs(xForbidden(0)) - abs(xActivation(0));
	xBuffer(1) = abs(xForbidden(1)) - abs(xActivation(1));
	xBuffer(2) = abs(xForbidden(2)) - abs(xActivation(2));

	torque_star_excluding_vw = VectorNd::Zero(NUMBER_OF_JOINTS,1);

	iiwa14 = robotParam;
}

VirtualWallAvoidance::~VirtualWallAvoidance()
{
}

int VirtualWallAvoidance::hasRobotExceededAWall()
{
	int counter=0;

	noOfExceededWallDirections=0;
	virtualWallExceeded = 0;

	for(int i=0;i<2;i++)
	{
		exceededWallDirectionIndex[i] = -1;
	}
	for (int i=1; i<2;i++) // in x and y directions only
	{
		//if ( (abs( xRobotPoints(5,i) ) >= abs( xActivation(i) ) ) )
		if ( ( (abs( iiwa14->xRobotPoints(5,i) ) ) >= abs( xActivation(i) ) ) )
		{
			virtualWallExceeded=1;
			noOfExceededWallDirections++;
			exceededWallDirectionIndex[counter] = i;
			counter++;
		}
		else
		{
			virtualWallExceeded &= 1;
		}
	}

	if (virtualWallExceeded==0)
	{
		torque = VectorNd::Zero(NUMBER_OF_JOINTS,1);
		nullspace = MatrixNd::Identity(NUMBER_OF_JOINTS, NUMBER_OF_JOINTS);
	}
	
	return virtualWallExceeded;
}

void VirtualWallAvoidance::wallRepulsionForce()
{
	int repulsionType=0; //0 - firas, 1 - felix
	double kvr_firas = 0.006;
	double eta[3] = {0.0050, 0.008, 0.80};
	double felix_constant[3] ={160.0, 160.0, 160.0};
	double kvr_felix = 50.0;
	int t;
	double rho_i;
	int ctr = 0;
	double lambda;
	double temp = 0.0;

	for (int i=0; i<NUMBER_OF_JOINTS; i++)
	{
		for (int j=0; j<NUMBER_OF_JOINTS; j++)
		{
			temp += iiwa14->qDot[j] * iiwa14->M(j,i);
		}
		lambda+= temp * iiwa14->qDot[i];
	}

	force_star.resize(noOfExceededWallDirections,1);
	force_tilde.resize(noOfExceededWallDirections,1);
	force_tilde_i.resize(noOfExceededWallDirections,1);
	h.resize(noOfExceededWallDirections,1);

	for(int i=0; i<noOfExceededWallDirections; i++)
	{
		force_star(i) = 0.0;
		force_tilde(i) = 0.0;
		force_tilde_i(i) = 0.0;
		h(i) = 0.0;
	}

	for (int i=0; i<noOfExceededWallDirections; i++)
	{
		t= exceededWallDirectionIndex[ctr];
		
		if ( ( abs(iiwa14->xRobotPoints(5,t)) >= abs(xActivation(t))) )
		{
			//Define Force based on Rep Type
			if (repulsionType==0)
			{
				if ( xForbidden(t) <0)
				{
					rho_i = ( iiwa14->xRobotPoints(5,t) ) - (xForbidden(t)) ;
					force_star(i) = (eta[t] * ( (1/rho_i - 1/xBuffer(t)) ) * (1/pow(rho_i,2))  - kvr_firas*( iiwa14->xDotRobotPoints(5,t) )); 
				}	
				else
				{
					rho_i = ( xForbidden(t) ) - ( iiwa14->xRobotPoints(5,t) );
					force_star(i) = -(eta[t] * ( (1/rho_i - 1/xBuffer(t)) ) * (1/pow(rho_i,2)) - kvr_firas*( iiwa14->xDotRobotPoints(5,t) )); 
				}
			}
			else
			{
				if (xForbidden(t) <0)
				{
					//rho_i = iiwa14->xRobotPoint(t) - xForbidden(t);
					//force_star(i) = felix_constant[t] * lambda * (rho_i) - kvr_felix*iiwa14->xDotRobotPoint(t); 
				}
				else
				{
					//rho_i =xForbidden(t) - (iiwa14->xRobotPoint(t));
					//force_star(i) = -felix_constant[t] * lambda * (rho_i) - kvr_felix*iiwa14->xDotRobotPoint(t);
				}
			}
			
			if ( (abs( iiwa14->xRobotPoints(5,t) ) >= abs(xForbidden(t))) )
			{
				h(i) = 1.0;
			}
			else if ( (abs( iiwa14->xRobotPoints(5,t) ) >= abs(xActivation(t))) )
			{
				h(i) = 0.5 + 0.5* sin ( (M_PI/abs(xBuffer(t)) * ((abs( iiwa14->xRobotPoints(5,t) ) - abs(xActivation(t))) )  -M_PI/2.0) );
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
		ctr++;
	}
	
}

void VirtualWallAvoidance::calcWallRepulsion()
{
	wallRepulsionForce();
	jacobian.resize(noOfExceededWallDirections, NUMBER_OF_JOINTS);
	jacobianTranspose.resize(NUMBER_OF_JOINTS, noOfExceededWallDirections);
	p.resize(noOfExceededWallDirections);
	
	//Jacobian of only the 6th joint. 3 rows per axis, so (5*3 + 0/1/2) gives the Jacobian of the 6th axis
	for(int i=0; i<noOfExceededWallDirections; i++)
	{
		jacobian.row(i) = iiwa14->jacobiansOfRobotPoints.row(5*3+exceededWallDirectionIndex[i]);
	}
	
	jacobianTranspose = jacobian.transpose();
		
	lambda = (jacobian * constraintNullspace.transpose() * iiwa14->Minv * constraintNullspace * jacobian).inverse();
	
	p = jacobian * iiwa14->Minv * iiwa14->g;
	
	force_tilde = force_star;
	
	if (gravityToggle == 1)
	{
		force_tilde += p;
	}

	for (int i=0;i<noOfExceededWallDirections;i++)
    {
		force_tilde_i[i]= h[i] * force_tilde[i] + (1 - h[i]) * jacobian.row(i) * iiwa14->Minv * torque_star_excluding_vw;
    }
	
	getInverseJacobian(jbar,jacobianTranspose,lambda);
		 
}

void VirtualWallAvoidance::calcTorqueAndNullspace(VectorNd& torque_star_excluding_vw, MatrixNd& constraintNullspace)
{
	this->torque_star_excluding_vw = torque_star_excluding_vw;
	this->constraintNullspace = constraintNullspace;


	if (virtualWallExceeded == 1)
	{
		calcWallRepulsion();
		torque = jacobianTranspose * lambda * force_tilde_i ;
	
		getNullspace(nullspace, jacobianTranspose, jbar);
	}
	else
	{
		torque = this->torque;
		nullspace = this->nullspace;
	}

}
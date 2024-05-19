#include "jlA.h"

#ifndef M_PI
#define M_PI 3.14159
#endif

/** Constructor

	robotParam:
	q_a: Activation joint angle values (7x2), q_a -> q_activation = q_l + buffer = qLimit + q0;
	q_l: Limit of each axis (= hardware limits of LBR)
	buffer: Buffer for zone where activation function h is activated
	tau: 

	*/

JointLimitAvoidance::JointLimitAvoidance(RobotParameters* robotParam, MatrixNd q_a, MatrixNd q_l, double buffer, VectorNd tau) 
{

	noOfExceededJointLimits = 0;
	hasAJointExceeded = 0;
	
	for (int i=0; i<7; i++)
	{
		exceededJointIndex[i] = 0;
	}

	torque = VectorNd::Zero(7,1);
	nullspace = MatrixNd::Identity(7,7);			//Nullspace projection matrix
	constraintNullspace = MatrixNd::Identity(7,7);

	force_star = VectorNd::Zero(7,1);
	force_tilde = VectorNd::Zero(7,1);
	force_tilde_i = VectorNd::Zero(7,1);			//Control force for joint limit avoidance
	h = VectorNd::Zero(7,1);

	tauMaxJoint = VectorNd::Zero(7, 1);
	tauMaxJoint = tau;

	q_activation = MatrixNd::Zero(7, 2);
	q_activation = q_a;
	
	qLimit = MatrixNd::Zero(7, 2);
	qLimit = q_l;

	q0 = buffer;

	iiwa14 = robotParam;
}

JointLimitAvoidance::~JointLimitAvoidance() 
{
}

int JointLimitAvoidance::hasAJointExceededLimits(int* exceededJointIndex)
{
	int counter = 0;
	noOfExceededJointLimits = 0;
	hasAJointExceeded = 0;

	for(int i=0; i<7; i++)
	{
		exceededJointIndex[i] = -1;	//The variable exceededJointIndex is used to store which of the 7 axes exceed their joint limit.
	}
	
	for (int i=0; i<7; i++)
	{
		if ( iiwa14->q[i] <= q_activation(i,0) || iiwa14->q[i] >= q_activation(i,1))
		{
			hasAJointExceeded =1;
			noOfExceededJointLimits++;
			exceededJointIndex[counter] = i;
			counter++;
		}
		else
		{
			torque = VectorNd::Zero(7,1);
			nullspace = MatrixNd::Identity(7, 7);
		}
	}

	exceededJointIndex = this->exceededJointIndex;

	return hasAJointExceeded;
}

void JointLimitAvoidance::calcRepulsionForce()
{
	int repulsionType=1; //Firas ==1, Felix == 0
	double kv_firas[7];
	double eta[7];
	double rho_i;
	int ctr=0;
	double felix_constant[7] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	double lambda = 0.0;
	double kvr_felix = 0;
	double temp= 0.0;
	double sign;

	for (int i=0; i<7; i++)
	{
		for (int j=0; j<7; j++)
		{
			temp += iiwa14->qDot[j] * iiwa14->M(j,i);
		}
		lambda+= temp * iiwa14->qDot[i];
	}

	for (int i=0; i< 7 ;i++)
	{
		temp = tauMaxJoint[i] / (4 * pow(q0,3));
		eta[i] = 0.001*temp;
		kv_firas[i] = 0.5;
		//eta[i] = 0.01*( M(i, i) * pow(qDot[i],2) )/ ( pow( (q0 / (q_activation(i,1) * qLimit(i,1)) ), 2 ) );
		felix_constant[i] *= 1000;
	}
	
	// High force parameters for the 7th joint, in order to see a good pre-damping effect.
	kv_firas[6] = 3500;
	eta[6] *= 100;

	force_star.resize(noOfExceededJointLimits);
	force_tilde.resize(noOfExceededJointLimits);
	force_tilde_i.resize(noOfExceededJointLimits);
	
	for(int i=0; i<noOfExceededJointLimits;i++)
	{
		force_star[i] = 0.0;
		force_tilde[i] = 0.0;
		force_tilde_i[i] = 0.0;
	}
	
	for (int i=0; i<noOfExceededJointLimits; i++)
	{
		if (exceededJointIndex[ctr] == 6 && abs(iiwa14->q[exceededJointIndex[ctr]]) >= (174.8 / 180.0 *M_PI ))
		{
			exit(0);
		}
		//if ( (q[i] <= q_activation(i,1)) || (q[i] >= q_activation(i,2)) )
			if (repulsionType==1)
			{
				if ( iiwa14->q[exceededJointIndex[ctr]] <= q_activation(exceededJointIndex[ctr],0) )
				{
					sign = (iiwa14->qDot[exceededJointIndex[ctr]] < 0 )? 1 :0;
					
					force_star[i] = ( - sign*(kv_firas[exceededJointIndex[ctr]] * iiwa14->qDot[exceededJointIndex[ctr]] )); 
					
					if ( iiwa14->q[exceededJointIndex[ctr]] <= ( q_activation(exceededJointIndex[ctr],0) - (20.0 / 180.0 * M_PI) ) )
					{
						rho_i = (qLimit(exceededJointIndex[ctr],0)) - (iiwa14->q[exceededJointIndex[ctr]]);
						force_star[i] += -(eta[exceededJointIndex[ctr]] * ( ( ( 1 /  ( rho_i ) - 1 / (q0) )) * ((1/ ( pow(rho_i,2) )) )) ); 
					}
					//force_repulsion[i] = -(eta[exceededJointIndex[ctr]] * ( ( ( 1 /  ( rho_i ) - 1 / (q0) )) * ((1/ ( pow(rho_i,2) )) )) - (kv_firas * qDot[exceededJointIndex[ctr]] )); 
					//std::cout<<"QLimit:"<<std::cout<<qLimit(exceededJointIndex[i],1)*180/3.14159<<std::endl;
					//std::cout<<"Q:"<<std::cout<<q[exceededJointIndex[i]]*180/3.14159<<std::endl;
					//std::cout<<"Eta:"<<std::cout<<eta[exceededJointIndex[i]] <<std::endl;
					//std::cout<<"qDot:"<<std::cout<<qDot[exceededJointIndex[i]]<<std::endl;
					//counter++;
				}
				else if ( iiwa14->q[exceededJointIndex[ctr]] >= q_activation(exceededJointIndex[ctr],1))
				{
					sign = (iiwa14->qDot[exceededJointIndex[ctr]] > 0 )? 1 :0;

					force_star[i] = (- sign*(kv_firas[exceededJointIndex[ctr]] * iiwa14->qDot[exceededJointIndex[ctr]] )); 

					if ( iiwa14->q[exceededJointIndex[ctr]] >= (q_activation(exceededJointIndex[ctr],1) + (20.0 / 180.0 * M_PI) ) )
					{
						rho_i = iiwa14->q[exceededJointIndex[ctr]] - qLimit(exceededJointIndex[ctr],1);
						force_star[i] += (eta[exceededJointIndex[ctr]] * ( ( ( 1 /  (  rho_i ) - 1 / (q0) )) * ((1/ ( pow(rho_i,2) ))) ) ); 
					}
					//force_repulsion[i] = (eta[exceededJointIndex[ctr]] * ( ( ( 1 /  (  rho_i ) - 1 / (q0) )) * ((1/ ( pow(rho_i,2) ))) ) - (kv_firas * qDot[exceededJointIndex[ctr]] )); 
					//counter++;
				}
				else
				{
					//Do nothing
				}
				
			}

			if (repulsionType==0)
			{
				
                if ( (iiwa14->q(i) <= q_activation(i,1)) )
					force_star[i] = -0.5 * iiwa14->M(exceededJointIndex[ctr], exceededJointIndex[ctr]) * pow(iiwa14->qDot[exceededJointIndex[ctr]],2) * (1 / (( (qLimit(exceededJointIndex[ctr],1)) - (iiwa14->q[exceededJointIndex[ctr]]) )) )  - (kvr_felix * iiwa14->qDot[exceededJointIndex[ctr]] ); 
                else
                    force_star[i] = 0.5* iiwa14->M(exceededJointIndex[ctr], exceededJointIndex[ctr]) * pow(iiwa14->qDot[exceededJointIndex[ctr]],2)  * (1 / (((iiwa14->q[exceededJointIndex[ctr]])-(qLimit(exceededJointIndex[ctr],2)))) )  - (kvr_felix * iiwa14->qDot[exceededJointIndex[ctr]] );  
			}
			
			ctr++;	
	}

	
}

void JointLimitAvoidance::getActivationParameter()
{
	int ctr=0;
	h.resize(noOfExceededJointLimits);
	for (int i=0; i<noOfExceededJointLimits; i++)
	{
		h[i] = 0.0;
	}
	
	ctr = 0;

	if (noOfExceededJointLimits != 0){

		for(int i=0; i<noOfExceededJointLimits;i++)
		{
			//if ( i == exceededJointIndex[ctr])
			//{

				if (iiwa14->q[exceededJointIndex[ctr]] <= qLimit(exceededJointIndex[ctr],0) || iiwa14->q[exceededJointIndex[ctr]] >= qLimit(exceededJointIndex[ctr],1) )
					h[i] = 1.0;
                
                else if (iiwa14->q[exceededJointIndex[ctr]] <= q_activation(exceededJointIndex[ctr],0) && iiwa14->q[exceededJointIndex[ctr]] > qLimit(exceededJointIndex[ctr],0) ){
					h[i] = 0.5 + 0.5*sin ((M_PI/q0 * ((iiwa14->q[exceededJointIndex[ctr]]) - (q_activation(exceededJointIndex[ctr],0)))) - M_PI/2);
				}
                
                else //if (q[i] < qLimit(i,1) && q[i] >= q_activation(i,1) )
					{
						h[i] = 0.5 + 0.5*sin ((M_PI/q0 * (iiwa14->q[exceededJointIndex[ctr]] - q_activation(exceededJointIndex[ctr],1))) - M_PI/2);
						//_myFile<<"Start:"<<endl<<M_PI/q0<<endl<<q[i]*180/M_PI<<endl<<q_activation(i,0)*180/M_PI<<endl<<sin ((M_PI/q0 * (abs(q[i]) - abs(q_activation(i,0)))) - M_PI/2)<<endl<<"End:";
				}

				ctr++;
     //           
     //           else
					//h[i] = 0.0;
			//}
		}
	}
}

void JointLimitAvoidance::calcRepulsionParameters()
{
	calcRepulsionForce();
	getActivationParameter(); 
	jacobian.resize(noOfExceededJointLimits, 7);
	jacobianTranspose.resize(7, noOfExceededJointLimits);
	p.resize(noOfExceededJointLimits);
		
	int ctr=0;

	//Write Jacobian (7x7) with 1.0 and 0.0 -> e.g. axis 4&6 in JL -> J4 = [0.0 0.0 0.0 1.0 0.0 0.0 0.0], J6 = [0.0 0.0 0.0 0.0 0.0 1.0 0.0] 

	for(int i=0;i<noOfExceededJointLimits;i++){

		for (int j=0;j<7;j++){
			if (j==exceededJointIndex[i])
				jacobian(i,j) = 1.0;
			else
				jacobian(i,j) = 0.0;
		}
	}	
	
	jacobianTranspose = jacobian.transpose();
	lambda.resize(noOfExceededJointLimits, noOfExceededJointLimits);

	//Compute Lambda (Paper (15))

	lambda = (jacobian * constraintNullspace.transpose() * iiwa14->Minv * constraintNullspace * jacobian.transpose()).inverse();

	//Compute Vector of gravity forces p = J * M(inv) * g

	p = jacobian * iiwa14->Minv * iiwa14->g;

	//Compute comanded torque (Paper (22))

	if (gravityToggle==1)
	{
		force_tilde = force_star + p;			//force_star is control input for unit mass system
	}
	else
	{
		force_tilde = force_star;
	}

	for (int i=0;i<noOfExceededJointLimits;i++)
	{
		force_tilde_i[i]= h[i] * force_tilde[i] + (1 - h[i]) * jacobian.row(i) * iiwa14->Minv * torque_star_excluding_jla;		
	}	

	getInverseJacobian(jbar, jacobianTranspose, lambda); 
}

void JointLimitAvoidance::calcTorqueAndNullspace(VectorNd& torque_star_excluding_jla, MatrixNd& constraintNullspace)
{
	this->torque_star_excluding_jla = torque_star_excluding_jla;
	this->constraintNullspace = constraintNullspace;

	calcRepulsionParameters();

	if ( hasAJointExceeded == 1)											
	{
		torque = jacobianTranspose * lambda * force_tilde_i;					//Commanded torque - paper (22)
		getNullspace(nullspace,jacobianTranspose,jbar);							//Nullspace projection matrix with torque of main task
	}
	else
	{
		torque = this->torque;
		nullspace = this->nullspace;
	}

}
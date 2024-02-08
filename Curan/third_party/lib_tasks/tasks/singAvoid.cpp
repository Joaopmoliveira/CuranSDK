#include "singAvoid.h"
#include <iostream>
#include <fstream>


using namespace std;

using namespace RigidBodyDynamics::Math;

SingAvoid::SingAvoid(RobotParameters* robotParam){
   
    //init. all function variab.
    //NUMBER_OF_JOINTS = 7;

    s = VectorNd::Zero(3,1);
    f_t_ns_i = VectorNd::Zero(3,1);
    f_t_s_i = VectorNd::Zero(3,1);
    f_t_ns = VectorNd::Zero(3,1);
    f_t_s = VectorNd::Zero(3,1);
    h_t_s = VectorNd::Zero(3,1);


    u = MatrixNd::Zero(3,3);
    J_t_ns = MatrixNd::Zero(3, LBR_N_JOINTS);
	J_t_ns_transpose = MatrixNd::Zero(LBR_N_JOINTS, 3);
	jbarNS = MatrixNd::Zero(LBR_N_JOINTS, 3);
    J_t_s = MatrixNd::Zero(3, LBR_N_JOINTS);
    lambda_t_ns = MatrixNd::Zero(3, 3);
    lambda_t_s = MatrixNd::Zero(3, 3);
    u_n_s = MatrixNd::Zero(3, 3);
    u_s = MatrixNd::Zero(3, 3);
    sigma = MatrixNd::Zero(3, 3);
    lambdaInv3d = Matrix3d::Zero(3, 3);

    M_inv = MatrixNd::Zero(LBR_N_JOINTS,LBR_N_JOINTS);
	nullspaceOfNSDirection = MatrixNd::Identity(LBR_N_JOINTS,LBR_N_JOINTS);

	iiwa14 = robotParam;
}

int SingAvoid::singularityAvoidance(VectorNd &taskTorque, MatrixNd &lambdaNew, MatrixNd &M_inv, MatrixNd &lambda_inv, MatrixNd &J, Vector3d &force, VectorNd &constraintTorque, MatrixNd &constraintNullspace,  MatrixNd &lowerPriorityTorque){

        ul = 0.03;
        ll = 0.02;
        b = ul-ll;
        noOfSingularities = 0;

		//Eigen accepts only matrices upto 4x4. But do robots know that?
		lambdaInv3d.row(0) = lambda_inv.row(0);
		lambdaInv3d.row(1) = lambda_inv.row(1);
		lambdaInv3d.row(2) = lambda_inv.row(2);

        Eigen::JacobiSVD<MatrixNd> svd(lambdaInv3d, Eigen::ComputeFullU);
        s = svd.singularValues();
        u = svd.matrixU();

        //cout<<"u: "<<u<<endl;


        int k=0;
        int l=0;

        for (int i=0; i<3;i++)
            singIndex[i] = -1;

        //Assuming Cartesian tasks with only 3 AXES
        for(int i=0; i<s.rows(); i++)
        {
            if (abs(s[i]) <= ul)
            {
                noOfSingularities++;
                singIndex[k] = i;
                k=k+1;
            }
        }

        sigma.resize((s.rows()-noOfSingularities), (s.rows()-noOfSingularities));

        for (int i=0;i<(s.rows()-noOfSingularities);i++)
        {
            for (int j=0;j<(s.rows()-noOfSingularities);j++)
            {
                if (i==j)
                    sigma(i,i) = 1/s(i);
                else
                    sigma(i,j) = 0.0;
            }
        }

        if (noOfSingularities == 3)
        {
            u_s.resize(u.rows(),noOfSingularities);
            f_t_s_i.resize(noOfSingularities,1);
            f_t_s.resize(noOfSingularities,1);
            J_t_s.resize(noOfSingularities, 7);
            lambda_t_s.resize(noOfSingularities, noOfSingularities);
        }
        else if ( (noOfSingularities == 2) || (noOfSingularities == 1) )
        {
            u_s.resize(u.rows(),noOfSingularities);
            u_n_s.resize(u.rows(),(u.rows()-noOfSingularities));
            f_t_s.resize(noOfSingularities,1);
            f_t_ns.resize(u.rows()-noOfSingularities,1);
            f_t_s_i.resize(noOfSingularities,1);
            f_t_ns_i.resize(u.rows()-noOfSingularities,1);
            J_t_s.resize(noOfSingularities, 7);
            J_t_ns.resize(u.rows()-noOfSingularities, 7);
            lambda_t_ns.resize(u.rows()-noOfSingularities, u.rows()-noOfSingularities);
            lambda_t_s.resize(noOfSingularities, noOfSingularities);
        }
        else
        {
            lambdaNew = lambda_inv.inverse();
            taskTorque = J.transpose() * lambdaNew * force;
            return 0;
        }

        h_t_s.conservativeResize(noOfSingularities);
        for(int i=0;i <noOfSingularities; i++)
        {
            h_t_s[i] = 1.0;
        }

        k=0;

        for(int i=0; i<u.cols(); i++)
        {
            if (i == singIndex[k])
            {
                u_s.col(k) = u.col(i);
                if ( abs(s[i])>=ll)
                    h_t_s[k] = (0.5 + 0.5 * sin (M_PI/b * (abs(s[i]) - ll) - M_PI/2 ));
                else
                    h_t_s[k] = 0.0;

                k=k+1;
            }
            else
            {
                u_n_s.col(l) = u.col(i);
                l=l+1;
            }
        }


        if (noOfSingularities!=3)
        {
            J_t_s = u_s.transpose() * J;
            f_t_s = u_s.transpose() * force;
            J_t_ns = u_n_s.transpose() * J;
            f_t_ns = u_n_s.transpose() * force;
			J_t_ns_transpose = J_t_ns.transpose();

            k=0;

            for(int i=0;i<J_t_ns.rows();i++)
            {
                J.row(k) = J_t_ns.row(i);
                k=k+1;
            }
            //Important to NOT reinitialise k
            for(int i=0;i<J_t_s.rows();i++)
            {
                J.row(k) = J_t_s.row(i);
                k=k+1;
            }


            lambdaNew = u_n_s * sigma * u_n_s.transpose();

            lambda_t_ns = (J_t_ns * iiwa14->Minv * J_t_ns_transpose).inverse();

            //lambda_t_ns = u_n_s.transpose()*lambdaNew;

			getInverseJacobian(jbarNS,J_t_ns_transpose,lambda_t_ns);
			getNullspace(nullspaceOfNSDirection, J_t_ns_transpose, jbarNS);

            f_t_ns_i = f_t_ns;
            VectorNd tempNew  = VectorNd::Zero(7, 1);
            VectorNd temp2;
            temp2.resize(noOfSingularities, 1);

            tempNew = iiwa14->Minv * (constraintTorque + constraintNullspace * J_t_ns.transpose() * lambda_t_ns * f_t_ns + constraintNullspace * nullspaceOfNSDirection *lowerPriorityTorque);

            for (int i=0; i<J_t_s.rows();i++)
            {
                //for (int j=0; j<J_t_s.cols(); j++)
                    temp2.row(i) = J_t_s.row(i) * tempNew;
            }

            for (int i=0;i<noOfSingularities;i++)
            {
                if (h_t_s[i] == 1.0)
                {
                    f_t_s_i[i] = f_t_s[i];
                    break;
                }

                f_t_s_i[i]  = h_t_s[i] * f_t_s[i] +  (1 - h_t_s[i] ) * temp2(i,0);
            }

            k=0;

            for(int i=0;i<f_t_ns_i.rows();i++)
            {
                force.row(k) = f_t_ns_i.row(i);
                k=k+1;
            }

            for(int i=0;i<f_t_s_i.rows();i++)
            {
                force.row(k) = f_t_s_i.row(i);
                k=k+1;
            }

            taskTorque = J_t_ns.transpose() * lambda_t_ns * f_t_ns_i + J_t_s.transpose() * f_t_s_i;
        }
        else
        {
            J_t_s = u_s.transpose() * J;
            f_t_s = u_s.transpose() * force;

            for (int i=0;i<noOfSingularities; i++)
                f_t_s_i[i] = h_t_s[i] * f_t_s[i];

            J = J_t_s;
            force = f_t_s_i;
            taskTorque = J.transpose() * force;
        }
        return 1;

}

SingAvoid::~SingAvoid(){
}

#include "robotutils/WallAvoidanceData.h"
#include <fstream>

namespace curan{
namespace robotic
{
    static inline Eigen::MatrixXd getLambdaLeastSquares(const Eigen::MatrixXd &M, const Eigen::MatrixXd& J, const double& k)
    {
        Eigen::MatrixXd Iden = Eigen::MatrixXd::Identity(J.rows(), J.rows());
        Eigen::MatrixXd Lambda_Inv = J * M.inverse() * J.transpose() + (k*k)*Iden;
        Eigen::MatrixXd Lambda = Lambda_Inv.inverse();
        return Lambda;
    }

    WallAvoidanceData::WallAvoidanceData(Eigen::Vector3d plane_point, Eigen::Vector3d direction_along_valid_region, double in_max_accel, double in_max_vel):
        max_accel{in_max_accel},max_vel{in_max_vel},f_plane_point{plane_point},f_direction_along_valid_region{direction_along_valid_region}
    {

    };

    EigenState &&WallAvoidanceData::update(const RobotModel<number_of_joints>& iiwa,EigenState &&state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        static double currentTime = 0.0;
        
        double Dmax = -f_direction_along_valid_region.transpose()*f_plane_point;
        double D = f_direction_along_valid_region.transpose()*iiwa.translation()+Dmax;
        Dmax= 0;

        Eigen::VectorXd vel = iiwa.jacobian()*iiwa.velocities();
        Eigen::Vector3d vel_pos = vel.block(0,0,3,1);
        double dotD = (f_direction_along_valid_region.transpose()*vel_pos)(0,0);

        constexpr double scalling_factor = 15.0;
        double dtvar = scalling_factor*iiwa.sample_time();
        if(dtvar<0.001)
            dtvar = scalling_factor*0.001;
        double dt2 = dtvar;
        double wallTopD = Dmax-D;

        double lowestdtfactor = 10;

        if(wallTopD<0.1){
            if(wallTopD<0.0)
                wallTopD = 0.0;
            dt2 = (lowestdtfactor+sqrt(lowestdtfactor*wallTopD))*dtvar;
            if(dt2< lowestdtfactor*dtvar)
                dt2 = lowestdtfactor*dtvar;
        }

        double dotDMaxFromD = (Dmax-D)/dt2;
        double dotDMaxFormdotdotD = ((Dmax-D)<0.0) ? 1000000.0 : sqrt(2*max_accel*(Dmax-D)); 

        Eigen::Vector3d vMaxVector{{max_vel,dotDMaxFormdotdotD,dotDMaxFromD}};
        double dotDMaxFinal = vMaxVector.minCoeff();

        double aMaxDotD = (dotDMaxFinal-dotD) / dtvar;
        double aMaxD = 2 * (Dmax-D - dotD * dt2) / (std::pow(dt2,2));

        Eigen::Vector3d aMaxVector{{1000000,aMaxDotD,aMaxD}};
        double dotdotDMaxFinal = aMaxVector.minCoeff();
    
        state.user_defined[0] = D;
        state.user_defined[1] = dotD;
        state.user_defined[2] = dotdotDMaxFinal;
        
        Eigen::Matrix<double,1,7> jacobianPos = f_direction_along_valid_region.transpose()*iiwa.jacobian().block(0,0,3,7);

        double LambdaInvPos = (jacobianPos*iiwa.invmass()*jacobianPos.transpose())(0,0)+(std::pow(0.3,2));
        double lambdaPos = 1/LambdaInvPos;
        Eigen::Matrix<double,7,1> JsatBar = iiwa.invmass() * jacobianPos.transpose() * lambdaPos;

        bool CreateTaskSat = false;

        Eigen::Matrix<double,7,7> Psat = Eigen::Matrix<double,7,7>::Identity();

        Eigen::Matrix<double,6,1> linear_acceleration_cartesian = iiwa.jacobian()*iiwa.invmass()*state.cmd_tau;
        Eigen::Matrix<double,3,1> translation_acceleration = linear_acceleration_cartesian.block(0,0,3,1);
        double linear_acceleration = translation_acceleration.transpose()*f_direction_along_valid_region;
        if(dotdotDMaxFinal + 0.001 < linear_acceleration)
            CreateTaskSat = true;

        Eigen::Matrix<double,7,1> tauS = Eigen::Matrix<double,7,1>::Zero();

        if(CreateTaskSat){
            Psat = Eigen::Matrix<double,7,7>::Identity()-jacobianPos.transpose()*JsatBar.transpose();
            tauS = jacobianPos.transpose()*lambdaPos*dotdotDMaxFinal;
        }
        
        auto projected_control = tauS+Psat*state.cmd_tau;
        state.cmd_tau = projected_control;

        state.cmd_q = iiwa.joints() + Eigen::Matrix<double,number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));

        currentTime += iiwa.sample_time();
        return std::move(state);
};
}
}

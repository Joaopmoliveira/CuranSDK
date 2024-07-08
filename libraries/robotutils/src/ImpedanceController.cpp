#include "robotutils/ImpedanceController.h"

namespace curan {
namespace robotic {
    EigenState&& ImpedanceController::update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
        static Eigen::Matrix<double,6,6> Iden = Eigen::Matrix<double,6,6>::Identity();
        Eigen::Matrix<double,6,6> Lambda_Inv = iiwa.jacobian() * iiwa.invmass() * iiwa.jacobian().transpose() + (0.071*0.071)*Iden;
        Eigen::Matrix<double,6,6> Lambda = Lambda_Inv.inverse();
        static Eigen::Matrix<double,6,1> error = Eigen::Matrix<double,6,1>::Zero();
        Eigen::AngleAxisd E_AxisAngle(iiwa.rotation().transpose() * f_equilibrium.desired_rotation());
        error.block<3,1>(3,0) = f_equilibrium.desired_translation()-iiwa.translation();
        error.block<3,1>(3,0) = E_AxisAngle.angle()*iiwa.rotation() *E_AxisAngle.axis();
        /*
        We use the decomposition introduced in 
        "Cartesian Impedance Control of Redundant and Flexible-Joint Robots" page 37
        but we use the nomenclature of the book in "Matrix Algebra From a Statistician's Perspective" 
        */
        Eigen::LLT<Eigen::Matrix<double,6,6>> lltOfLambda(Lambda);
        Eigen::Matrix<double,6,6> L = lltOfLambda.matrixL();
        Eigen::Matrix<double,6,6> R = L.inverse();
        Eigen::Matrix<double,6,6> C = R*stiffness*R.transpose();
        Eigen::JacobiSVD<Eigen::Matrix<double,6,6>> svdofsitff{C,Eigen::ComputeFullU};
        Eigen::Matrix<double,6,6> P = svdofsitff.matrixU();
        Eigen::Matrix<double,6,6> Q = R.inverse()*P;
        Eigen::Matrix<double,6,6> Qinv = Q.inverse();
        Eigen::Matrix<double,6,6> B0 = Qinv*stiffness*Qinv.transpose();
        Eigen::Matrix<double,6,6> damping = 2*Q.transpose()*diagonal_damping*B0.diagonal().array().sqrt().matrix().asDiagonal()*Q;

        state.cmd_tau = iiwa.jacobian().transpose()*Lambda*(stiffness*error-damping*iiwa.jacobian()*iiwa.velocities());//(Eigen::Matrix<double,number_of_joints,number_of_joints>::Identity() - iiwa.jacobian().transpose() * (iiwa.invmass() * iiwa.jacobian().transpose() * Lambda).inverse())*(-iiwa.mass() * 10 * iiwa.velocities());
        
        /*
        The Java controller has two values which it reads, namely: 
        1) commanded_joint_position 
        2) commanded_torque 
        The torque is computed in the previous line, but the position can remain empty. One problem with this approach is that if the deviation
        between the reference position and the commanded position is larger than 5 degrees the robot triggers a safety stop 1. To avoid this
        the first solution is to set the commanded position to be equal to the current position. This approach has a drawback where the error between
        both commanded and current position is always zero, which results in the friction compensator being "shut off". We avoid this problem
        by adding a small perturbation to the reference position with a relative high frequency. 
        */
        state.cmd_q = iiwa.joints() + Eigen::Matrix<double,number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));

        currentTime += iiwa.sample_time();
        return std::move(state);
    }

}
}
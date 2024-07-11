#include "robotutils/ImpedanceController.h"

namespace curan
{
    namespace robotic
    {
        EigenState &&ImpedanceController::update(const RobotModel<number_of_joints> &iiwa, EigenState &&state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians)
        {
            static double currentTime = 0.0;
            /*
            We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
            */
            Eigen::Matrix<double, 6, 6> Lambda;
            {
                Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svdofsitff{iiwa.jacobian() * iiwa.invmass() * iiwa.jacobian().transpose(), Eigen::ComputeFullU};
                Eigen::Matrix<double, 6, 6> P = svdofsitff.matrixU();
                auto singular_values = svdofsitff.singularValues();
                for(auto& diag : singular_values)
                    diag = (diag<0.02) ? 0.02 : diag;
                Lambda = (P*singular_values.asDiagonal()*P.transpose()).inverse();
            }            
            
            Eigen::Matrix<double, number_of_joints, number_of_joints> nullspace_projector = Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity() - iiwa.jacobian().transpose() * (iiwa.invmass() * iiwa.jacobian().transpose() * Lambda).transpose();
            Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
            Eigen::AngleAxisd E_AxisAngle(iiwa.rotation().transpose() * f_equilibrium.desired_rotation());
            error.block<3, 1>(0, 0) = (f_equilibrium.desired_translation() - iiwa.translation());
            error.block<3, 1>(3, 0) = E_AxisAngle.angle() * iiwa.rotation() * E_AxisAngle.axis();
            Eigen::Matrix<double, number_of_joints, 1> error_in_nullspace = -iiwa.joints();
            /*
            We use the decomposition introduced in
            "Cartesian Impedance Control of Redundant and Flexible-Joint Robots" page 37
            but we use the nomenclature of the book in "Matrix Algebra From a Statistician's Perspective"
            */

           /*
           We have two controllers running simultaneously, the first controller, the so called cartersian impedance controller 
           has a stiffness specified by the user, while the second impedance controller focuses on bringing all joints to their 
           home configuration.
           */
            Eigen::Matrix<double, 6, 6> B0_cartesian;
            Eigen::Matrix<double, 6, 6> Q_cartesian;
            {
                Eigen::LLT<Eigen::Matrix<double, 6, 6>> lltOfLambda(Lambda);
                Eigen::Matrix<double, 6, 6> L = lltOfLambda.matrixL();
                Eigen::Matrix<double, 6, 6> R = L.inverse();
                Eigen::Matrix<double, 6, 6> C = R * stiffness * R.transpose();
                Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svdofsitff{C, Eigen::ComputeFullU};
                Eigen::Matrix<double, 6, 6> P = svdofsitff.matrixU();
                Q_cartesian = R.inverse() * P;
                Eigen::Matrix<double, 6, 6> Qinv = Q_cartesian.inverse();
                B0_cartesian = Qinv * stiffness * Qinv.transpose();
            }
            Eigen::Matrix<double, number_of_joints, number_of_joints> nullspace_stiffness = 10.0*Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity();
            Eigen::Matrix<double, number_of_joints, number_of_joints> B0_nullspace;
            Eigen::Matrix<double, number_of_joints, number_of_joints> Q_nullspace;
            {
                Eigen::LLT<Eigen::Matrix<double, number_of_joints, number_of_joints>> lltOfLambda(iiwa.mass());
                Eigen::Matrix<double, number_of_joints, number_of_joints> L = lltOfLambda.matrixL();
                Eigen::Matrix<double, number_of_joints, number_of_joints> R = L.inverse();
                Eigen::Matrix<double, number_of_joints, number_of_joints> C = R * nullspace_stiffness * R.transpose();
                Eigen::JacobiSVD<Eigen::Matrix<double, number_of_joints, number_of_joints>> svdofsitff{C, Eigen::ComputeFullU};
                Eigen::Matrix<double, number_of_joints, number_of_joints> P = svdofsitff.matrixU();
                Q_nullspace = R.inverse() * P;
                Eigen::Matrix<double, number_of_joints, number_of_joints> Qinv = Q_nullspace.inverse();
                B0_nullspace = Qinv * nullspace_stiffness * Qinv.transpose();
            }

            Eigen::Matrix<double, 6, 6> damping_cartesian = 2 * Q_cartesian * diagonal_damping * B0_cartesian.diagonal().array().sqrt().matrix().asDiagonal() * Q_cartesian.transpose();
            //it is assumed that the nullspace has a diagonal damping ratio of 1.0. 
            Eigen::Matrix<double, number_of_joints, number_of_joints> damping_nullspace = 2 * Q_nullspace * B0_nullspace.diagonal().array().sqrt().matrix().asDiagonal() * Q_nullspace.transpose();

            /*
            We need to filer out the velocity because in steady stady state they can become problematic
            */
            static Eigen::Matrix<double, number_of_joints, 1> filtered_velocity =  iiwa.velocities();
            auto val = 0.8 * filtered_velocity + 0.2 * iiwa.velocities();
            filtered_velocity = val;


            // normalize the error to an upper bound
            state.cmd_tau = iiwa.jacobian().transpose() * (stiffness * error - damping_cartesian *  iiwa.jacobian() * filtered_velocity) + nullspace_projector * ( nullspace_stiffness* error_in_nullspace-damping_nullspace*filtered_velocity-10.0*iiwa.mass()*filtered_velocity);
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
            state.cmd_q = iiwa.joints() + Eigen::Matrix<double, number_of_joints, 1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));

            currentTime += iiwa.sample_time();
            return std::move(state);
        }

    }
}
#include "robotutils/CartersianVelocityController.h"
#include "utils/Overloading.h"

namespace curan
{
    namespace robotic
    {

        CartersianVelocityController::CartersianVelocityController(const Transformation &equilibrium,
                                                                   std::initializer_list<double> stiffness_diagonal_gains,
                                                                   std::initializer_list<double> in_diagonal_damping) : ref_trajectory{equilibrium}
        {
            if (stiffness_diagonal_gains.size() != 6)
                throw std::runtime_error("the dimensions of the diagonal matrix must be 6x6");
            if (in_diagonal_damping.size() != 6)
                throw std::runtime_error("the dimensions of the diagonal matrix must be 6x6");
            stiffness = Eigen::Matrix<double, 6, 6>::Identity();
            auto stiffness_entry_value = stiffness_diagonal_gains.begin();
            auto damping_entry_value = in_diagonal_damping.begin();
            for (size_t entry = 0; entry < 6; ++entry, ++stiffness_entry_value, ++damping_entry_value)
            {
                if (*damping_entry_value < 0.0 || *damping_entry_value > 1.0)
                    throw std::runtime_error("the damping matrix must be positive definite (in [0 1]) : (" + std::to_string(entry) + ") -> (" + std::to_string(*damping_entry_value) + ")");
                if (*stiffness_entry_value < 0.0)
                    throw std::runtime_error("the stiffness matrix must be positive definite : (" + std::to_string(entry) + ") -> (" + std::to_string(*stiffness_entry_value) + ")");
                diagonal_damping(entry, entry) = *damping_entry_value;
                stiffness(entry, entry) = *stiffness_entry_value;
            }
        }

        CartersianVelocityController::CartersianVelocityController(Generator &&generator,
                                                                   std::initializer_list<double> stiffness_diagonal_gains,
                                                                   std::initializer_list<double> in_diagonal_damping) : ref_trajectory{std::move(generator)}
        {
            if (stiffness_diagonal_gains.size() != 6)
                throw std::runtime_error("the dimensions of the diagonal matrix must be 6x6");
            if (in_diagonal_damping.size() != 6)
                throw std::runtime_error("the dimensions of the diagonal matrix must be 6x6");
            stiffness = Eigen::Matrix<double, 6, 6>::Identity();
            auto stiffness_entry_value = stiffness_diagonal_gains.begin();
            auto damping_entry_value = in_diagonal_damping.begin();
            for (size_t entry = 0; entry < 6; ++entry, ++stiffness_entry_value, ++damping_entry_value)
            {
                if (*damping_entry_value < 0.0 || *damping_entry_value > 1.0)
                    throw std::runtime_error("the damping matrix must be positive definite (in [0 1]) : (" + std::to_string(entry) + ") -> (" + std::to_string(*damping_entry_value) + ")");
                if (*stiffness_entry_value < 0.0)
                    throw std::runtime_error("the stiffness matrix must be positive definite : (" + std::to_string(entry) + ") -> (" + std::to_string(*stiffness_entry_value) + ")");
                diagonal_damping(entry, entry) = *damping_entry_value;
                stiffness(entry, entry) = *stiffness_entry_value;
            }
        }

        EigenState &&CartersianVelocityController::update(const RobotModel<number_of_joints> &iiwa, EigenState &&state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians)
        {
            static double currentTime = 0.0;
            Eigen::Matrix<double, 6, 6> Lambda;
            {
                Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svdofsitff{iiwa.jacobian() * iiwa.invmass() * iiwa.jacobian().transpose(), Eigen::ComputeFullU};
                Eigen::Matrix<double, 6, 6> P = svdofsitff.matrixU();
                auto singular_values = svdofsitff.singularValues();
                for (auto &diag : singular_values)
                    diag = (diag < 0.02) ? 0.02 : diag;
                Lambda = (P * singular_values.asDiagonal() * P.transpose()).inverse();
            }
            Eigen::Matrix<double, 6, 1> desired_velocity = Eigen::Matrix<double, 6, 1>::Zero();

            /*
            The velocity tracker has two modes, the first mode uses a lambda to generate the desired velocity
            whilst the second mode takes an equilibrium position and converges towards it uses a standard dot_x = (x_equilibrium-x);
            */
            std::visit(utilities::overloaded{[&](Generator &arg)
                                             {
                                                 desired_velocity = arg(iiwa);
                                             },
                                             [&](Transformation &f_equilibrium)
                                             {
                                                 Eigen::AngleAxisd E_AxisAngle(iiwa.rotation().transpose() * f_equilibrium.desired_rotation());
                                                 desired_velocity.block<3, 1>(0, 0) = (f_equilibrium.desired_translation() - iiwa.translation());
                                                 desired_velocity.block<3, 1>(3, 0) = E_AxisAngle.angle() * iiwa.rotation() * E_AxisAngle.axis();
                                             }},
                       ref_trajectory);

            Eigen::Matrix<double, number_of_joints, number_of_joints> nullspace_projector = Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity() - iiwa.jacobian().transpose() * (iiwa.invmass() * iiwa.jacobian().transpose() * Lambda).transpose();
            Eigen::Matrix<double, number_of_joints, 1> error_in_nullspace = -iiwa.joints();

            static Eigen::Matrix<double, 7, 1> filtered_velocity = iiwa.velocities();
            auto val = 0.8187 * filtered_velocity + 0.1813 * iiwa.velocities();
            filtered_velocity = val;

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

            Eigen::Matrix<double, number_of_joints, number_of_joints> nullspace_stiffness = 3.0 * Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity();
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
            // it is assumed that the nullspace has a diagonal damping ratio of 1.0.
            Eigen::Matrix<double, number_of_joints, number_of_joints> damping_nullspace = 2 * Q_nullspace * B0_nullspace.diagonal().array().sqrt().matrix().asDiagonal() * Q_nullspace.transpose();

            auto error_in_velocity = desired_velocity - iiwa.jacobian() * filtered_velocity;
            auto actuation = stiffness * desired_velocity + damping_cartesian * error_in_velocity;

            /*
            Computation of forces caused by coordinate change from joints to cartersian pose
            */
            static auto previous_jacobian = iiwa.jacobian();
            static Eigen::Matrix<double, 6, curan::robotic::number_of_joints> jacobian_derivative = (iiwa.jacobian() - previous_jacobian) / iiwa.sample_time();
            jacobian_derivative = (0.8 * jacobian_derivative + 0.2 * (iiwa.jacobian() - previous_jacobian) / iiwa.sample_time()).eval();
            previous_jacobian = iiwa.jacobian();
            auto conditioned_matrix = iiwa.jacobian().transpose() * iiwa.jacobian();
            Eigen::JacobiSVD<Eigen::Matrix<double, curan::robotic::number_of_joints, curan::robotic::number_of_joints>> svdjacobian{conditioned_matrix, Eigen::ComputeFullU};
            auto U = svdjacobian.matrixU();
            auto singular_values = svdjacobian.singularValues();
            for (auto &diag : singular_values)
            {
                diag = (diag < 0.02) ? 0.02 : diag;
            }
            auto properly_conditioned_matrix = (U * singular_values.asDiagonal() * U.transpose());
            auto inverse_jacobian = properly_conditioned_matrix.inverse() * iiwa.jacobian().transpose();
            // these values can becomes quite high with large accelerations
            Eigen::Matrix<double,6,1> forces_caused_by_coordinate_change = inverse_jacobian.transpose()*iiwa.mass()* inverse_jacobian * jacobian_derivative * inverse_jacobian * iiwa.jacobian() * filtered_velocity;
            
            
            state.cmd_tau = iiwa.jacobian().transpose() * (stiffness * desired_velocity + damping_cartesian * error_in_velocity) +
                            nullspace_projector * (nullspace_stiffness * error_in_nullspace - damping_nullspace * filtered_velocity - 10.0 * iiwa.mass() * filtered_velocity);
            // absolute damper - this thing should avoid larger accelerations


            for (size_t i = 0; i < actuation.rows(); ++i)
            {
                state.user_defined[i] = actuation[i];
                state.user_defined2[i] = error_in_velocity[i];
                if(i<6)
                    state.user_defined3[i] = forces_caused_by_coordinate_change[i];
                state.user_defined4[i] = svdjacobian.singularValues()[i];
            }
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
#include "robotutils/PassiveControllerAude.h"
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

    PassiveControllerData::PassiveControllerData(const std::string &model_file){
        {
            std::ifstream modelfile{model_file};
            modelfile >> model;
        }
    };

    EigenState &&PassiveControllerData::update(kuka::Robot *robot, RobotParameters *iiwa, EigenState &&state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        static double currentTime = 0.0;
        auto desLinVelocity = model.likeliest(state.translation);
        // Operational Space Control (OSC)

        // Compute goal position based on desired velocity.
        Eigen::Vector3d posDes = state.translation + desLinVelocity * state.sampleTime;

        // Set matrices for current robot configuration.
        const Eigen::MatrixXd jacobianPos = state.jacobian.block(0, 0, 3, LBR_N_JOINTS);
        const Eigen::MatrixXd jacobianRot = state.jacobian.block(3, 0, 3, LBR_N_JOINTS);
        Eigen::MatrixXd lambda = getLambdaLeastSquares(iiwa->M, state.jacobian, 0.071);
        Eigen::MatrixXd lambdaPos = getLambdaLeastSquares(iiwa->M, jacobianPos, 0.071);
        Eigen::MatrixXd lambdaRot = getLambdaLeastSquares(iiwa->M, jacobianRot, 0.071);

        // ############################################################################################
        // Compute positional part.
        // ############################################################################################

        auto maxCartSpeed = 0.3;
        const double stiffness = 800;
        const double damping = 100;
        Eigen::Vector3d posErr = posDes - state.translation;
        Eigen::Vector3d velCurr = jacobianPos * state.dq;
        Eigen::Vector3d velDes = desLinVelocity + (stiffness / damping) * posErr;
        // Limit velocity to maxCartSpeed.
        velDes = velDes.norm() == 0.0
                 ? velDes * 0.0
                 : velDes * std::min(1.0, maxCartSpeed / velDes.norm());
        // Compute positional force command.
        Eigen::Vector3d forcePos = damping * (velDes - velCurr);

        // ############################################################################################
        // Compute rotation part.
        // ############################################################################################

        const double maxRotSpeed = 3.0;
        const double angularStiffness = 400;
        const double angularDamping = 40;
        Eigen::Matrix3d R_E_Ed = state.rotation.transpose() * desRotation;
        // Convert rotation error in eef frame to axis angle representation.
        Eigen::AngleAxisd E_AxisAngle(R_E_Ed);
        Eigen::Vector3d E_u = E_AxisAngle.axis();
        double angleErr = E_AxisAngle.angle();
        // Convert axis to base frame.
        Eigen::Vector3d O_u = state.rotation * E_u;
        // Compute rotational error. (Scaled axis angle)
        Eigen::Vector3d rotErr = O_u * angleErr;
        Eigen::Vector3d rotVelCurr = jacobianRot * state.dq;
        Eigen::Vector3d rotVelDes = (angularStiffness / angularDamping) * rotErr; // + 0 desired
        // Limit rotational velocity to maxRotSpeed.
        rotVelDes = rotVelDes.norm() == 0.0
                    ? rotVelDes * 0.0
                    : rotVelDes * std::min(1.0, maxRotSpeed / rotVelDes.norm());
        // Compute rotational force command.
        Eigen::Vector3d forceRot = angularDamping * (rotVelDes - rotVelCurr);

        // ############################################################################################
        // Compute positional and rotational nullspace.
        // ############################################################################################

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(LBR_N_JOINTS, LBR_N_JOINTS);
        Eigen::MatrixXd jbarPos = iiwa->Minv * jacobianPos.transpose() * lambdaPos;
        Eigen::MatrixXd jbarRot = iiwa->Minv * jacobianRot.transpose() * lambdaRot;
        Eigen::MatrixXd nullSpaceTranslation = I - jacobianPos.transpose() * jbarPos.transpose();
        Eigen::MatrixXd nullSpaceRotation = I - jacobianRot.transpose() * jbarRot.transpose();

        // Compute positional and rotation torque from force commands.
        Eigen::VectorXd torquePos = jacobianPos.transpose() * lambdaPos * forcePos;
        Eigen::VectorXd torqueRot = jacobianRot.transpose() * lambdaRot * forceRot;

        // Set torque command.
        state.cmd_tau = torquePos + torqueRot; // Prioritize positional torques.

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
        state.cmd_q = state.q + Eigen::Matrix<double,number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));

        currentTime += state.sampleTime;
        // Set nullspace.
        //nullSpace = nullSpaceTranslation * nullSpaceRotation;
        return std::move(state);
};
}
}

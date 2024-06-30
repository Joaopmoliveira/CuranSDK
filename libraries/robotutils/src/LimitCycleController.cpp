#include "robotutils/LimitCycleController.h"
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

    LimitCycleController::LimitCycleController(const std::string& model_file,const std::string& transform_file){
        {
            std::ifstream modelfile{model_file};
            modelfile >> model;
        }

        {   
	        std::ifstream transformfile{transform_file};
            if(transformfile.is_open())
                std::cout << "partial success\n";
	        nlohmann::json calibration_data = nlohmann::json::parse(transformfile);

            std::string rotationstring = calibration_data["rotation"];
            std::stringstream s;
            s <<  rotationstring;
            std::string translationstring = calibration_data["translation"];
        
            transformation_to_model_coordinates.rotation = curan::utilities::convert_matrix(s);
            s = std::stringstream{};
            s <<  translationstring;
            transformation_to_model_coordinates.translation = curan::utilities::convert_matrix(s);
        }

    };

    EigenState &&LimitCycleController::update(const RobotModel<number_of_joints>& iiwa, EigenState &&state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
    static double currentTime = 0.0;
    constexpr double sample_time = 0.005;

    Eigen::Vector3d transformed_position = transformation_to_model_coordinates.rotation.transpose()*(iiwa.translation() - transformation_to_model_coordinates.translation);
    Eigen::Vector2d limit_cycle_portion = transformed_position.block(0,0,2,1);

    auto in_plane_velocity = model.likeliest(limit_cycle_portion);

    constexpr double scalling_limit_cycle = 1.0;
    Eigen::Vector3d desired_velocity_in_plane;
    desired_velocity_in_plane << scalling_limit_cycle*in_plane_velocity(0) , scalling_limit_cycle*in_plane_velocity(1) , -2.0*transformed_position(2);

    Eigen::Vector3d desLinVelocity = transformation_to_model_coordinates.rotation*desired_velocity_in_plane;

    Eigen::Matrix3d desRotation = Eigen::Matrix3d::Identity();

    desRotation <<  -1.0 , -0.0 , 0.0 ,
                    -0.0 ,  1.0 , 0.0 ,
                    -0.0 ,  0.0 , -1.0;


        
    // Operational Space Control (OSC)

	// Compute goal position based on desired velocity.
	Eigen::Vector3d posDes  = iiwa.translation() + desLinVelocity * sample_time;

	// Set matrices for current robot configuration.
	const Eigen::MatrixXd jacobianPos = iiwa.jacobian().block(0,0,3, number_of_joints);
	const Eigen::MatrixXd jacobianRot = iiwa.jacobian().block(3,0,3, number_of_joints);
	Eigen::MatrixXd lambda    = getLambdaLeastSquares(iiwa.mass(),iiwa.jacobian()   ,0.3);
	Eigen::MatrixXd lambdaPos = getLambdaLeastSquares(iiwa.mass(),jacobianPos,0.3);
	Eigen::MatrixXd lambdaRot = getLambdaLeastSquares(iiwa.mass(),jacobianRot,0.3);

	// ############################################################################################
	// Compute positional part.
	// ############################################################################################

	auto maxCartSpeed = 1.2;
    const double stiffness = 1000;
	const double damping   = 35;
	Eigen::Vector3d posErr  = posDes - iiwa.translation();
	Eigen::Vector3d velCurr = jacobianPos * iiwa.velocities();
	Eigen::Vector3d velDes  = desLinVelocity + (stiffness / damping) * posErr;
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
    const double angularStiffness = 1000;
    const double angularDamping   = 40;
	Eigen::Matrix3d R_E_Ed = iiwa.rotation().transpose() * desRotation;
	// Convert rotation error in eef frame to axis angle representation.
    Eigen::AngleAxisd E_AxisAngle(R_E_Ed);
    Eigen::Vector3d E_u = E_AxisAngle.axis();
    double angleErr     = E_AxisAngle.angle();
	// Convert axis to base frame.
    Eigen::Vector3d O_u = iiwa.rotation() * E_u;
	// Compute rotational error. (Scaled axis angle)
	Eigen::Vector3d rotErr     = O_u * angleErr;
	Eigen::Vector3d rotVelCurr = jacobianRot * iiwa.velocities();
	Eigen::Vector3d rotVelDes  = (angularStiffness / angularDamping) * rotErr; // + 0 desired 
	// Limit rotational velocity to maxRotSpeed.
	rotVelDes = rotVelDes.norm() == 0.0
		? rotVelDes * 0.0
		: rotVelDes * std::min(1.0, maxRotSpeed / rotVelDes.norm());
	// Compute rotational force command.
	Eigen::Vector3d forceRot = angularDamping * (rotVelDes - rotVelCurr);

	// ############################################################################################
	// Compute positional and rotational nullspace.
	// ############################################################################################
	
	Eigen::MatrixXd I       = Eigen::MatrixXd::Identity(number_of_joints, number_of_joints);
	Eigen::MatrixXd jbarPos = iiwa.invmass() * jacobianPos.transpose() * lambdaPos;
	Eigen::MatrixXd jbarRot = iiwa.invmass() * jacobianRot.transpose() * lambdaRot;
	Eigen::MatrixXd nullSpaceTranslation = I - jacobianPos.transpose() * jbarPos.transpose();
	Eigen::MatrixXd nullSpaceRotation    = I - jacobianRot.transpose() * jbarRot.transpose();

	// Compute positional and rotation torque from force commands.
	Eigen::VectorXd torquePos = jacobianPos.transpose() * lambdaPos * forcePos;
	Eigen::VectorXd torqueRot = jacobianRot.transpose() * lambdaRot * forceRot;

	// Set nullspace.
	auto nullSpace = nullSpaceTranslation * nullSpaceRotation;

    // Apply damping torques in nullspace.
	const double dampingQ = 1.0;
    //Eigen::MatrixXd clamperQ = Eigen::MatrixXd::Zero(LBR_N_JOINTS,LBR_N_JOINTS);
    //clamperQ(0,0) = 10;
    Eigen::VectorXd dampingTorque = -iiwa.mass() * dampingQ * iiwa.velocities();

    //Eigen::VectorXd firstJointClamperTorque = -clamperQ*state.q;

    // Set torque command.
    state.cmd_tau = torquePos + torqueRot + nullSpace*(dampingTorque); // Prioritize positional torques.
    state.user_defined[0] = velDes[0]; // Prioritize positional torques.
    state.user_defined[1] = velDes[1]; // Prioritize positional torques.
    state.user_defined[2] = velDes[2]; // Prioritize positional torques.
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
    // Set nullspace.
    //nullSpace = nullSpaceTranslation * nullSpaceRotation;
    return std::move(state);
};
}
}

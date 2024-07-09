#include "robotutils/LimitCycleController.h"
#include <fstream>

namespace curan
{
    namespace robotic
    {
        LimitCycleController::LimitCycleController(const std::string &model_file, const std::string &transform_file) : transformation_to_model_coordinates{}{
            {std::ifstream modelfile{model_file};
        modelfile >> model;
    }

    {
        std::ifstream transformfile{transform_file};
        if (transformfile.is_open())
            std::cout << "partial success\n";
        nlohmann::json calibration_data = nlohmann::json::parse(transformfile);

        std::string rotationstring = calibration_data["rotation"];
        std::stringstream s;
        s << rotationstring;
        std::string translationstring = calibration_data["translation"];

        transformation_to_model_coordinates.f_rotation = curan::utilities::convert_matrix(s);
        s = std::stringstream{};
        s << translationstring;
        transformation_to_model_coordinates.f_translation = curan::utilities::convert_matrix(s);
    }

};

EigenState &&LimitCycleController::update(const RobotModel<number_of_joints> &iiwa, EigenState &&state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians)
{
    static double currentTime = 0.0;

    /*
    We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
    */
    Eigen::Matrix<double, 6, 6> Lambda = (iiwa.jacobian() * iiwa.invmass() * iiwa.jacobian().transpose() + 0.005 * Eigen::Matrix<double, 6, 6>::Identity()).inverse();
    Eigen::Matrix<double, number_of_joints, number_of_joints> nullspace_projector = Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity() - iiwa.jacobian().transpose() * (iiwa.invmass() * iiwa.jacobian().transpose() * Lambda).transpose();
    Eigen::Matrix<double, 6, 1> desired_velocity = Eigen::Matrix<double, 6, 1>::Zero();

    /*
    To compute the desired velocity we need to first transform the point into the proper coordinates and only afterwards can we 
    */
    Eigen::Matrix<double,3,1> rotated_translation = transformation_to_model_coordinates.desired_rotation()*iiwa.translation()+transformation_to_model_coordinates.desired_translation();
    auto velocity_in_2d = model.likeliest(rotated_translation.block<2,1>(0,0));
    Eigen::Matrix<double,3,1> rotated_velocity ;
    rotated_velocity << velocity_in_2d[0] , velocity_in_2d[1] , -rotated_translation[2];
    Eigen::Matrix<double,3,1> velocity = transformation_to_model_coordinates.desired_rotation()*rotated_velocity;

    Eigen::AngleAxisd E_AxisAngle(iiwa.rotation().transpose() * transformation_to_model_coordinates.desired_rotation());
    desired_velocity.block<3, 1>(0, 0) = rotated_velocity;
    desired_velocity.block<3, 1>(3, 0) = E_AxisAngle.angle() * iiwa.rotation() * E_AxisAngle.axis();
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
    Eigen::Matrix<double, number_of_joints, number_of_joints> nullspace_stiffness = 10.0 * Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity();
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
    // it is assumed that the nullspace has a diagonal damping ratio of 1.0.
    Eigen::Matrix<double, number_of_joints, number_of_joints> damping_nullspace = 2 * Q_nullspace * B0_nullspace.diagonal().array().sqrt().matrix().asDiagonal() * Q_nullspace.transpose();

    /*
    We need to filer out the velocity because in steady stady state they can become problematic
    */
    static Eigen::Matrix<double, number_of_joints, 1> filtered_velocity = iiwa.velocities();
    auto val = 0.8187 * filtered_velocity + 0.1813 * iiwa.velocities();
    filtered_velocity = val;

    // normalize the error to an upper bound
    state.cmd_tau = iiwa.jacobian().transpose() * (Lambda * (desired_velocity - iiwa.jacobian() * filtered_velocity)) + nullspace_projector * (nullspace_stiffness * error_in_nullspace - damping_nullspace * filtered_velocity);
    //                                                                            ---- cartesian velocity ----
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
    // Set nullspace.
    // nullSpace = nullSpaceTranslation * nullSpaceRotation;
    return std::move(state);
};
}
}

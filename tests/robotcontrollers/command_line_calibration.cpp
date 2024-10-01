#include "robotutils/LBRController.h"
#include "robotutils/SimulateModel.h"


inline double rad2deg(double in)
{
    constexpr double constant_convert_deg_two_radians = 0.0174532925;
    return constant_convert_deg_two_radians * in;
}

int main()
{
    std::list<Eigen::Matrix<double, 4, 4>> list_of_recorded_points_for_calibration;

    curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    curan::robotic::State state;
    double joint_config = 0.0;

    while (true)
    {
        for (size_t i = 0; i < 7; ++i)
        {
            
            std::cin >> joint_config;
            std::cout << "\n";
            state.q[i] = rad2deg(joint_config);
        }
        robot_model.update(state);
        Eigen::Matrix<double, 4, 4> homo = Eigen::Matrix<double, 4, 4>::Identity();
        homo.block<3, 3>(0, 0) = robot_model.rotation();
        homo.block<3, 1>(0, 3) = robot_model.translation();
        list_of_recorded_points_for_calibration.push_back(homo);
        std::cout << "insert another? ";
        char c;
        std::cin >> c;
        if (c != 's')
            break;
    }

    if (list_of_recorded_points_for_calibration.size() == 0)
        return 1;
    Eigen::Matrix<double, Eigen::Dynamic, 4> calib_p = Eigen::Matrix<double, Eigen::Dynamic, 4>::Ones(list_of_recorded_points_for_calibration.size(), 4);
    auto iterator = list_of_recorded_points_for_calibration.begin();
    for (size_t i = 0; i < list_of_recorded_points_for_calibration.size(); ++i, ++iterator)
        calib_p.row(i) = (*iterator).col(3).transpose();
    std::cout << "calib matrix: \n"
              << calib_p << std::endl;
    double conditioned = (calib_p.transpose() * calib_p).determinant();
    std::cout << "determinant \n"
              << conditioned << std::endl;
    if (std::abs(conditioned) < 1e-4)
        return 1;

    std::cout << "array without" << calib_p.array().square().rowwise().sum() << std::endl;
    std::cout << (calib_p.array().square().rowwise().sum() - 1.0).matrix() << std::endl;

    Eigen::Matrix<double, 4, 1> solution = (calib_p.transpose() * calib_p).inverse() * (calib_p.transpose()) * (calib_p.array().square().rowwise().sum() - 1.0).matrix();
    std::cout << "solution: \n"
              << solution.transpose() << std::endl;
    Eigen::Matrix<double, 3, 1> pivot_point = solution.block<3, 1>(0, 0) * 0.5;

    std::cout << "pivot point: \n"
              << pivot_point.transpose() << std::endl;

    double radius = std::sqrt(solution[3] + pivot_point.transpose() * pivot_point);
    if (radius < 1e-4)
        return 1;

    std::cout << "radius: \n"
              << radius << std::endl;

    Eigen::Matrix<double, 3, 1> average_normal_vectors = Eigen::Matrix<double, 3, 1>::Zero();
    iterator = list_of_recorded_points_for_calibration.begin();
    for (size_t i = 0; i < list_of_recorded_points_for_calibration.size(); ++i, ++iterator)
    {
        Eigen::Matrix<double, 4, 1> pivot_point_homogenenous = Eigen::Matrix<double, 4, 1>::Ones();
        pivot_point_homogenenous.block<3, 1>(0, 0) = pivot_point;
        Eigen::Matrix<double, 3, 1> to_normalize = ((((*iterator)).inverse() * pivot_point_homogenenous)).block<3, 1>(0, 0);
        to_normalize.normalize();
        average_normal_vectors += (1.0 / list_of_recorded_points_for_calibration.size()) * to_normalize;
    }

    average_normal_vectors.normalize();

    std::cout << "normalized: \n"
              << average_normal_vectors << std::endl;

    Eigen::Matrix<double, 4, 4> calibrated_needle = Eigen::Matrix<double, 4, 4>::Identity();
    calibrated_needle.block<3, 1>(0, 3) = radius * average_normal_vectors;

    double calibration_error = 0.0;
    iterator = list_of_recorded_points_for_calibration.begin();
    for (size_t i = 0; i < list_of_recorded_points_for_calibration.size(); ++i, ++iterator)
    {
        Eigen::Matrix<double, 4, 1> pivot_point_homogenenous = Eigen::Matrix<double, 4, 1>::Ones();
        pivot_point_homogenenous.block<3, 1>(0, 0) = pivot_point;
        auto supposed_pivot_point = (*iterator).inverse() * calibrated_needle;
        calibration_error += (supposed_pivot_point.col(3) - pivot_point_homogenenous).norm();
    }

    auto return_current_time_and_date = []()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
        return ss.str();
    };

    nlohmann::json calibration_data;
    calibration_data["timestamp"] = return_current_time_and_date();
    std::stringstream ss;
    ss << calibrated_needle;
    calibration_data["needle_homogeneous_transformation"] = ss.str();
    calibration_data["optimization_error"] = calibration_error;
    // write prettified JSON to another file
    std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/needle_calibration.json");
    o << calibration_data;
    std::cout << "calibration data from needle coordinates" << calibration_data << std::endl;

    return 0;
}
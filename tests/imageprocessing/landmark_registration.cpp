#include <Eigen/Dense>
#include <cmath>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include "utils/Reader.h"

int main()
{
    Eigen::Matrix<double, 3, Eigen::Dynamic> moving_points;
    Eigen::Matrix<double, 3, Eigen::Dynamic> fixed_points;

    {
        nlohmann::json needle_poses_recorded_from_world_coordinates;
        std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/points_in_world_space.json");
        if(!in){
            std::cout << "failed to read fixed point file\n";
            return 1;
        }
        in >> needle_poses_recorded_from_world_coordinates;

        std::string string_world_points = needle_poses_recorded_from_world_coordinates["world_points"];
        std::stringstream matrix_strm;
        matrix_strm << string_world_points;
        auto world_points = curan::utilities::convert_matrix(matrix_strm, ',');
        fixed_points = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3,world_points.cols());
        for (Eigen::Index row = 0; row < world_points.rows(); ++row)
            for (Eigen::Index col = 0; col < world_points.cols(); ++col)
                fixed_points(row, col) = world_points(row, col);
    }

    {
        nlohmann::json needle_poses_recorded_from_ct_coordinates;
        std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/landmarks_to_register.json");
        in >> needle_poses_recorded_from_ct_coordinates;
        if(!in){
            std::cout << "failed to read moving point file\n";
            return 1;
        }
        std::stringstream matrix_strm;
        std::string landmarks_to_register =  needle_poses_recorded_from_ct_coordinates["landmarks_to_register"];
        matrix_strm << landmarks_to_register;
        auto ct_points = curan::utilities::convert_matrix(matrix_strm, ',');
        moving_points = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3,ct_points.cols());
        for (Eigen::Index row = 0; row < ct_points.rows(); ++row)
            for (Eigen::Index col = 0; col < ct_points.cols(); ++col)
                moving_points(row, col) = ct_points(row, col);
    }

    Eigen::Matrix<double, 3, 1> centroid_fixed_points = fixed_points.rowwise().mean();
    Eigen::Matrix<double, 3, 1> centroid_moving_points = moving_points.rowwise().mean();

    Eigen::Matrix<double, 3, Eigen::Dynamic> centered_fixed_points = fixed_points;
    Eigen::Matrix<double, 3, Eigen::Dynamic> centered_moving_points = moving_points;

    centered_fixed_points.colwise() -= centroid_fixed_points;
    centered_moving_points.colwise() -= centroid_moving_points;

    Eigen::Matrix<double, 3, 3> H = Eigen::Matrix<double, 3, 3>::Zero();

    if (centered_fixed_points.cols() != centered_moving_points.cols())
    {
        std::cout << "number of collumns differs\n";
        return 1;
    }

    for (size_t i = 0; i < centered_fixed_points.cols(); ++i)
        H += centered_fixed_points.col(i) * centered_moving_points.col(i).transpose();

    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<double, 3, 3> rotation_fixed_to_moving = svd.matrixV() * svd.matrixU().transpose();
    auto determinant = rotation_fixed_to_moving.determinant();
    if (determinant < 1 - 1e-7 || determinant > 1 + 1e-7)
    {
        std::cout << "determinant is not positive\n";
        return 1;
    }

    Eigen::Matrix<double, 3, 1> translation_fixed_to_moving = centroid_moving_points - rotation_fixed_to_moving * centroid_fixed_points;

    Eigen::Matrix<double, 4, 4> registration_solution = Eigen::Matrix<double, 4, 4>::Identity();
    registration_solution.block<3, 3>(0, 0) = rotation_fixed_to_moving.transpose();
    registration_solution.block<3, 1>(0, 3) = -rotation_fixed_to_moving.transpose() * translation_fixed_to_moving;

    Eigen::Matrix<double, 3, Eigen::Dynamic> aligned_moving_points =  moving_points;

    for(size_t col = 0; col <aligned_moving_points.cols() ; ++col)
        aligned_moving_points.col(col) = (rotation_fixed_to_moving.transpose()*aligned_moving_points.col(col)- rotation_fixed_to_moving.transpose() * translation_fixed_to_moving).eval();

    std::cout << "error:\n" << aligned_moving_points - fixed_points << std::endl;
    std::cout << "error squared:\n" << (aligned_moving_points - fixed_points).array().square() << std::endl;
    std::cout << "error squared summed:\n" << (aligned_moving_points - fixed_points).array().square().rowwise().sum() << std::endl;
    std::cout << "error squared summed rooted:\n" << (aligned_moving_points - fixed_points).array().square().rowwise().sum().sqrt() << std::endl;
    std::cout << "error:\n" << (aligned_moving_points - fixed_points).array().square().rowwise().sum().sqrt().colwise().sum() << std::endl;   
    
    auto error = (aligned_moving_points - fixed_points).array().square().rowwise().sum().sqrt().colwise().sum();
    std::cout << "error: " << error << std::endl;

    std::cout << "registration solution:\n" << registration_solution << std::endl;

    return 0;
}
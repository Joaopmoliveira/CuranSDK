#include "utils/FileStructures.h"
#include <gtest/gtest.h>

const std::string date{"1970-01-01 00:00:00"};

TEST(UnitTestFilePropagator, NeedleCalibrationData)
{
    for (size_t run_n_times = 0; run_n_times < 20; ++run_n_times)
    {
        Eigen::Matrix<double, 4, 4> transform = Eigen::Matrix<double, 4, 4>::Identity();
        curan::utilities::NeedleCalibrationData source_needle_calib(date, transform, 1.0);
        std::stringstream mock_file;
        mock_file << source_needle_calib; //here we mock an application that previously wrote to disk and now we are reading from disk
        curan::utilities::NeedleCalibrationData target_needle_calib{mock_file};
        ASSERT_EQ(source_needle_calib.needle_calibration().isApprox(target_needle_calib.needle_calibration(),0.0001), true) << "the serialization deserealization process loses data (" << source_needle_calib.needle_calibration() << ") (" << target_needle_calib.needle_calibration() << ")";
        ASSERT_EQ(source_needle_calib.optimization_error(), target_needle_calib.optimization_error()) << "the serialization deserealization process loses data";
        ASSERT_EQ(source_needle_calib.timestamp(), target_needle_calib.timestamp()) << "the serialization deserealization process loses data";
    }
}

TEST(UnitTestFilePropagator, RegistrationData)
{
    for (size_t run_n_times = 0; run_n_times < 20; ++run_n_times)
    {
        Eigen::Matrix<double, 4, 4> transform = Eigen::Matrix<double, 4, 4>::Random();
        curan::utilities::RegistrationData source_registration_data(date, transform, 1.0, curan::utilities::SURFACE);
        std::stringstream mock_file;
        mock_file << source_registration_data;  //here we mock an application that previously wrote to disk and now we are reading from disk
        curan::utilities::RegistrationData target_registration_data{mock_file};
        ASSERT_EQ(source_registration_data.moving_to_fixed_transform().isApprox(target_registration_data.moving_to_fixed_transform(),0.0001), true) << "the serialization deserealization process loses data (" << source_registration_data.moving_to_fixed_transform() << ") (" << target_registration_data.moving_to_fixed_transform() << ")";
        ASSERT_EQ(source_registration_data.registration_error(), target_registration_data.registration_error()) << "the serialization deserealization process loses data";
        ASSERT_EQ(source_registration_data.timestamp(), target_registration_data.timestamp()) << "the serialization deserealization process loses data";
        ASSERT_EQ(source_registration_data.registration_type(), target_registration_data.registration_type()) << "the serialization deserealization process loses data";
    }
}

TEST(UnitTestFilePropagator, TrajectorySpecificationData)
{
    for (size_t run_n_times = 0; run_n_times < 20; ++run_n_times)
    {
        Eigen::Matrix<double, 3, 1> target = Eigen::Matrix<double, 3, 1>::Random();
        Eigen::Matrix<double, 3, 1> entrypoint = Eigen::Matrix<double, 3, 1>::Random();
        Eigen::Matrix<double, 3, 3> orientation = Eigen::Matrix<double, 3, 3>::Random();
        const std::string path_to_fixed_image{"blub/mock/not/real/path"};
        curan::utilities::TrajectorySpecificationData source_trajectory_specification_data(date, target, entrypoint, orientation, path_to_fixed_image);
        std::stringstream mock_file;
        mock_file << source_trajectory_specification_data;  //here we mock an application that previously wrote to disk and now we are reading from disk
        curan::utilities::TrajectorySpecificationData target_trajectory_specification_data{mock_file};
        ASSERT_EQ(source_trajectory_specification_data.desired_direction().isApprox(target_trajectory_specification_data.desired_direction(),0.0001), true) << "the serialization deserealization process loses data";
        ASSERT_EQ(source_trajectory_specification_data.entry().isApprox(target_trajectory_specification_data.entry(),0.0001), true) << "the serialization deserealization process loses data";
        ASSERT_EQ(source_trajectory_specification_data.target().isApprox(target_trajectory_specification_data.target(),0.0001), true) << "the serialization deserealization process loses data";
        ASSERT_EQ(source_trajectory_specification_data.path_to_image(), target_trajectory_specification_data.path_to_image()) << "the serialization deserealization process loses data";
        ASSERT_EQ(source_trajectory_specification_data.timestamp(), target_trajectory_specification_data.timestamp()) << "the serialization deserealization process loses data";
    }
}

TEST(UnitTestFilePropagator, UltrasoundCalibrationData)
{
    for (size_t run_n_times = 0; run_n_times < 20; ++run_n_times)
    {
        Eigen::Matrix<double, 4, 4> homogeneous_transformation = Eigen::Matrix<double, 4, 4>::Random();
        std::vector<Eigen::Matrix<double, 1, 4>> line_parameterization;
        for(size_t i = 0; i < 3; ++i)
            line_parameterization.push_back(Eigen::Matrix<double,1,4>::Random());
        curan::utilities::UltrasoundCalibrationData source_ultrasound_data(date, homogeneous_transformation,line_parameterization, 1.0);
        std::stringstream mock_file;
        mock_file << source_ultrasound_data;  //here we mock an application that previously wrote to disk and now we are reading from disk
        curan::utilities::UltrasoundCalibrationData target_ultrasound_data{mock_file};
        ASSERT_EQ(source_ultrasound_data.homogeneous_transformation().isApprox(target_ultrasound_data.homogeneous_transformation(),0.0001), true) << "the serialization deserealization process loses data";
        ASSERT_EQ(source_ultrasound_data.optimization_error(), target_ultrasound_data.optimization_error()) << "the serialization deserealization process loses data";
        ASSERT_EQ(source_ultrasound_data.timestamp(), target_ultrasound_data.timestamp()) << "the serialization deserealization process loses data";
    }
}
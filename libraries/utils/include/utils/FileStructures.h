#ifndef CURAN_FILE_STRUCTURES_HEADER_FILE_
#define CURAN_FILE_STRUCTURES_HEADER_FILE_

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "utils/Reader.h"
#include <fstream>
#include <vector>

namespace curan
{
    namespace utilities
    {

        class UltrasoundCalibrationData
        {
            std::string f_timestamp;
            Eigen::Matrix<double, 4, 4> f_homogeneous_transformation;
            std::vector<Eigen::Matrix<double,1,4>> f_line_parameterization; 
            double f_optimization_error;

            void decode(std::istream &instream);

        public:
            UltrasoundCalibrationData(const std::string &in_timestamp,
                                      const Eigen::Matrix<double, 4, 4> &in_homogeneous_transformation,
                                      const std::vector<Eigen::Matrix<double,1,4>> &in_parameterization,
                                      const double &in_optimization_error) : f_timestamp{in_timestamp},
                                                                             f_homogeneous_transformation{in_homogeneous_transformation},
                                                                             f_optimization_error{in_optimization_error},
                                                                             f_line_parameterization{in_parameterization}
            {
            }

            UltrasoundCalibrationData(const std::string &path);

            UltrasoundCalibrationData(std::istream &instream);

            std::string timestamp() const;

            Eigen::Matrix<double, 4, 4> homogeneous_transformation() const;

            std::vector<Eigen::Matrix<double,1,4>> line_parameterization() const;

            double optimization_error() const;
        };

        std::ostream &operator<<(std::ostream &os, const UltrasoundCalibrationData &calib);

        enum Type
        {
            LANDMARK,
            SURFACE,
            VOLUME
        };

        class RegistrationData
        {

            std::string f_timestamp;
            Eigen::Matrix<double, 4, 4> f_moving_to_fixed_transform;
            double f_registration_error;
            Type f_registration_type = Type::LANDMARK;

            void decode(std::istream &instream);

        public:
            RegistrationData(const std::string &in_timestamp,
                             const Eigen::Matrix<double, 4, 4> &in_moving_to_fixed_transform,
                             const double &in_registration_error,
                             const Type &in_registration_type) : f_timestamp{in_timestamp},
                                                                 f_moving_to_fixed_transform{in_moving_to_fixed_transform},
                                                                 f_registration_error{in_registration_error},
                                                                 f_registration_type{in_registration_type}
            {
            }

            RegistrationData(const std::string &path);

            RegistrationData(std::istream &instream);

            std::string timestamp() const;

            Eigen::Matrix<double, 4, 4> moving_to_fixed_transform() const;

            double registration_error() const;

            Type registration_type() const;
        };

        std::ostream &operator<<(std::ostream &os, const RegistrationData &calib);

        class NeedleCalibrationData
        {
            std::string f_timestamp;
            Eigen::Matrix<double, 4, 4> f_needle_homogeneous_transformation;
            double f_optimization_error;

            void decode(std::istream &instream);

        public:
            NeedleCalibrationData(const std::string &in_timestamp,
                                  const Eigen::Matrix<double, 4, 4> &in_needle_homogeneous_transformation,
                                  const double &in_optimization_error) : f_timestamp{in_timestamp},
                                                                         f_needle_homogeneous_transformation{in_needle_homogeneous_transformation},
                                                                         f_optimization_error{in_optimization_error}
            {
            }

            NeedleCalibrationData(const std::string &path);

            NeedleCalibrationData(std::istream &instream);

            std::string timestamp() const;

            Eigen::Matrix<double, 4, 4> needle_calibration() const;

            double optimization_error() const;
        };

        std::ostream &operator<<(std::ostream &os, const NeedleCalibrationData &calib);

        class TrajectorySpecificationData
        {
            std::string f_timestamp;
            Eigen::Matrix<double, 3, 1> f_target;
            Eigen::Matrix<double, 3, 1> f_entrypoint;
            Eigen::Matrix<double, 3, 3> f_desired_orientation;
            std::string f_path_to_image;

            void decode(std::istream &instream);

        public:
            TrajectorySpecificationData(const std::string &in_timestamp,
                                        const Eigen::Matrix<double, 3, 1> &in_target,
                                        const Eigen::Matrix<double, 3, 1> &in_entrypoint,
                                        const Eigen::Matrix<double,3,3>& in_desired_orientation,
                                        const std::string &in_path_to_image) : f_timestamp{in_timestamp},
                                                                               f_target{in_target},
                                                                               f_entrypoint{in_entrypoint},
                                                                               f_desired_orientation{in_desired_orientation},
                                                                               f_path_to_image{in_path_to_image}
            {
            }

            TrajectorySpecificationData(const std::string &path);

            TrajectorySpecificationData(std::istream &instream);

            std::string timestamp() const;

            Eigen::Matrix<double, 3, 1> target() const;

            Eigen::Matrix<double, 3, 1> entry() const;

            Eigen::Matrix<double, 3, 3> desired_direction() const;

            std::string path_to_image() const;
        };

        std::ostream &operator<<(std::ostream &os, const TrajectorySpecificationData &calib);

    }
}

#endif
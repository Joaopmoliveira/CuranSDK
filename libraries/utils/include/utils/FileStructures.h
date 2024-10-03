#ifndef CURAN_FILE_STRUCTURES_HEADER_FILE_
#define CURAN_FILE_STRUCTURES_HEADER_FILE_

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "utils/Reader.h"

namespace curan
{
    namespace utilities
    {

        const Eigen::IOFormat &desired_matrix_format()
        {
            static Eigen::IOFormat CleanFmt{Eigen::StreamPrecision, 0, ", ", "\n", " ", " "};
            return CleanFmt;
        };

        class UltrasoundCalibrationData
        {
            std::string f_timestamp;
            Eigen::Matrix<double, 4, 4> f_homogeneous_transformation;
            double f_optimization_error;

        public:
            UltrasoundCalibrationData(const std::string &in_timestamp,
                                      const Eigen::Matrix<double, 4, 4> &in_homogeneous_transformation,
                                      const double &in_optimization_error) : f_timestamp{in_timestamp},
                                                                             f_homogeneous_transformation{in_homogeneous_transformation},
                                                                             f_optimization_error{in_optimization_error}
            {
            }

            UltrasoundCalibrationData(const std::string &path)
            {
                nlohmann::json calibration_data;
                std::ifstream in(path);

                if (!in.is_open())
                    throw std::runtime_error("failure to open configuration file");

                in >> calibration_data;
                f_timestamp = calibration_data["timestamp"];
                std::string homogenenous_transformation = calibration_data["homogeneous_transformation"];
                f_optimization_error = calibration_data["optimization_error"];

                std::stringstream matrix_strm;
                matrix_strm << homogenenous_transformation;
                auto calibration_matrix = curan::utilities::convert_matrix(matrix_strm, ',');

                if (calibration_matrix.cols() != f_homogeneous_transformation.cols() || calibration_matrix.rows() != f_homogeneous_transformation.rows())
                    throw std::runtime_error("unexptected dimensions");

                for (Eigen::Index row = 0; row < calibration_matrix.rows(); ++row)
                    for (Eigen::Index col = 0; col < calibration_matrix.cols(); ++col)
                        f_homogeneous_transformation(col, row) = calibration_matrix(row, col);
            }

            std::string timestamp() const
            {
                return f_timestamp;
            }

            Eigen::Matrix<double, 4, 4> homogeneous_transformation() const
            {
                return f_homogeneous_transformation;
            }

            double optimization_error() const
            {
                return f_optimization_error;
            }
        };

        std::ostream &operator<<(std::ostream &os, const UltrasoundCalibrationData &calib)
        {

            std::stringstream optimized_values;
            optimized_values << calib.homogeneous_transformation().format(desired_matrix_format()) << std::endl;

            nlohmann::json calibration_data;
            calibration_data["timestamp"] = calib.timestamp();
            calibration_data["homogeneous_transformation"] = optimized_values.str();
            calibration_data["optimization_error"] = calib.optimization_error();

            os << calibration_data;

            return os;
        }

        enum Type
        {
            LANDMARK,
            SURFACE,
            VOLUME
        };

        std::map<std::string, Type> conversion_to_type{{"landmark", LANDMARK}, {"surface", SURFACE}, {"volume", VOLUME}};
        const char *conversion_from_type[] = {"landmark", "surface", "volume"};

        class RegistrationData
        {

            std::string f_timestamp;
            Eigen::Matrix<double, 4, 4> f_moving_to_fixed_transform;
            double f_registration_error;
            Type f_registration_type = Type::LANDMARK;

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

            RegistrationData(const std::string &path)
            {
                nlohmann::json registration_data;
                std::ifstream in(path);

                if (!in.is_open())
                    throw std::runtime_error("failure to open configuration file");

                in >> registration_data;

                if (auto search = conversion_to_type.find(registration_data["type"]); search != conversion_to_type.end())
                    f_registration_type = search->second;
                else
                    throw std::runtime_error("incorrect type");

                f_timestamp = registration_data["timestamp"];

                std::string homogenenous_transformation = registration_data["moving_to_fixed_transform"];
                f_registration_error = registration_data["registration_error"];

                std::stringstream matrix_strm;
                matrix_strm << homogenenous_transformation;
                auto registration_mat = curan::utilities::convert_matrix(matrix_strm, ',');
                if (registration_mat.cols() != f_moving_to_fixed_transform.cols() || registration_mat.rows() != f_moving_to_fixed_transform.rows())
                    throw std::runtime_error("unexptected dimensions");

                for (Eigen::Index row = 0; row < registration_mat.rows(); ++row)
                    for (Eigen::Index col = 0; col < registration_mat.cols(); ++col)
                        f_moving_to_fixed_transform(col, row) = registration_mat(row, col);
            }

            std::string timestamp() const
            {
                return f_timestamp;
            }

            Eigen::Matrix<double, 4, 4> moving_to_fixed_transform() const
            {
                return f_moving_to_fixed_transform;
            }

            double registration_error() const
            {
                return f_registration_error;
            }

            Type registration_type() const
            {
                return f_registration_type;
            }
        };

        std::ostream &operator<<(std::ostream &os, const RegistrationData &calib)
        {
            std::stringstream optimized_values;
            optimized_values << calib.moving_to_fixed_transform().format(desired_matrix_format()) << std::endl;

            nlohmann::json registration_data;
            registration_data["timestamp"] = calib.timestamp();
            registration_data["moving_to_fixed_transform"] = optimized_values.str();
            registration_data["registration_error"] = calib.registration_error();
            registration_data["type"] = conversion_from_type[calib.registration_type()];

            os << registration_data;
            return os;
        }

        class NeedleCalibrationData
        {
            std::string f_timestamp;
            Eigen::Matrix<double, 4, 4> f_needle_homogeneous_transformation;
            double f_optimization_error;

        public:
            NeedleCalibrationData(const std::string &in_timestamp,
                                  const Eigen::Matrix<double, 4, 4> &in_needle_homogeneous_transformation,
                                  const double &in_optimization_error) : f_timestamp{in_timestamp},
                                                                         f_needle_homogeneous_transformation{in_needle_homogeneous_transformation},
                                                                         f_optimization_error{in_optimization_error}
            {
            }

            NeedleCalibrationData(const std::string &path)
            {
                nlohmann::json needle_calibration_data;
                std::ifstream in(path);

                if (!in.is_open())
                    throw std::runtime_error("failure to open configuration file");

                in >> needle_calibration_data;
                f_timestamp = needle_calibration_data["timestamp"];
                std::string homogenenous_transformation = needle_calibration_data["needle_homogeneous_transformation"];
                f_optimization_error = needle_calibration_data["optimization_error"];

                std::stringstream matrix_strm;
                matrix_strm << homogenenous_transformation;
                auto calibration_matrix = curan::utilities::convert_matrix(matrix_strm, ',');

                if (calibration_matrix.cols() != f_needle_homogeneous_transformation.cols() || calibration_matrix.rows() != f_needle_homogeneous_transformation.rows())
                    throw std::runtime_error("unexptected dimensions");

                for (Eigen::Index row = 0; row < calibration_matrix.rows(); ++row)
                    for (Eigen::Index col = 0; col < calibration_matrix.cols(); ++col)
                        f_needle_homogeneous_transformation(col, row) = calibration_matrix(row, col);

            }

            std::string timestamp() const
            {
                return f_timestamp;
            }

            Eigen::Matrix<double, 4, 4> needle_calibration() const
            {
                return f_needle_homogeneous_transformation;
            }

            double optimization_error() const
            {
                return f_optimization_error;
            }
        };

        std::ostream &operator<<(std::ostream &os, const NeedleCalibrationData &calib)
        {
            std::stringstream optimized_values;
            optimized_values << calib.needle_calibration().format(desired_matrix_format()) << std::endl;

            nlohmann::json calibration_data;
            calibration_data["timestamp"] = calib.timestamp();
            calibration_data["needle_homogeneous_transformation"] = optimized_values.str();
            calibration_data["optimization_error"] = calib.optimization_error();

            os << calibration_data;
            return os;
        }

        class TrajectorySpecificationData
        {
            std::string f_timestamp;
            Eigen::Matrix<double, 3, 1> f_target;
            Eigen::Matrix<double, 3, 1> f_entrypoint;
            std::string f_path_to_image;

        public:
            TrajectorySpecificationData(const std::string &in_timestamp,
                                        const Eigen::Matrix<double, 3, 1> &in_target,
                                        const Eigen::Matrix<double, 3, 1> &in_entrypoint,
                                        const std::string &in_path_to_image) : f_timestamp{in_timestamp},
                                                                               f_target{in_target},
                                                                               f_entrypoint{in_entrypoint},
                                                                               f_path_to_image{in_path_to_image}
            {
            }

            TrajectorySpecificationData(const std::string &path)
            {
                nlohmann::json trajectory_data;
                std::ifstream in(path);
                if (!in.is_open())
                    throw std::runtime_error("failure to open configuration file");

                in >> trajectory_data;

                std::stringstream ss;
                std::string target = trajectory_data["target"];
                ss << target;
                auto eigen_target = curan::utilities::convert_matrix(ss, ',');
                ss = std::stringstream{};
                std::string entry = trajectory_data["entry"];
                ss << entry;
                auto eigen_entry = curan::utilities::convert_matrix(ss, ',');
                if (eigen_target.cols() != 1 || eigen_target.rows() != 3)
                    throw std::runtime_error("incorrect target dimensions");
                if (eigen_entry.cols() != 1 && eigen_entry.rows() != 3)
                    throw std::runtime_error("incorrect entry dimensions");

                for (size_t i = 0; i < 3; ++i)
                {
                    f_target[i] = eigen_target(i, 0);
                    f_entrypoint[i] = eigen_entry(i, 0);
                }
                f_path_to_image = trajectory_data["moving_image_directory"];
            }

            std::string timestamp() const
            {
                return f_timestamp;
            }

            Eigen::Matrix<double, 3, 1> target() const
            {
                return f_target;
            }

            Eigen::Matrix<double, 3, 1> entry() const
            {
                return f_entrypoint;
            }

            std::string path_to_image() const
            {
                return f_path_to_image;
            }
        };

        std::ostream &operator<<(std::ostream &os, const TrajectorySpecificationData &calib)
        {

            nlohmann::json calibration_data;
            calibration_data["timestamp"] = calib.timestamp();
            {
                std::stringstream optimized_values;
                optimized_values << calib.target().format(desired_matrix_format()) << std::endl;
                calibration_data["target"] = optimized_values.str();
            }

            {
                std::stringstream optimized_values;
                optimized_values << calib.entry().format(desired_matrix_format()) << std::endl;
                calibration_data["entry"] = optimized_values.str();
            }

            calibration_data["moving_image_directory"] = calib.path_to_image();

            os << calibration_data;
            return os;
        }

    }
}

#endif
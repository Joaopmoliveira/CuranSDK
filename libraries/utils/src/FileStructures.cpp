#include "utils/FileStructures.h"

namespace curan
{
    namespace utilities
    {

        static const Eigen::IOFormat &desired_matrix_format()
        {
            static Eigen::IOFormat CleanFmt{Eigen::FullPrecision, 0, ", ", "\n", " ", " "};
            return CleanFmt;
        };

        UltrasoundCalibrationData::UltrasoundCalibrationData(const std::string &path)
        {
            nlohmann::json calibration_data;
            std::ifstream in(path);
            if (!in.is_open())
                throw std::runtime_error("failure to open configuration file");
            decode(in);
        }

        UltrasoundCalibrationData::UltrasoundCalibrationData(std::istream &instream){
            decode(instream);
        }

        void UltrasoundCalibrationData::decode(std::istream &instream){
            nlohmann::json calibration_data;
            instream >> calibration_data;
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
                    f_homogeneous_transformation(row, col) = calibration_matrix(row, col);
        }

        

        std::string UltrasoundCalibrationData::timestamp() const
        {
            return f_timestamp;
        }

        Eigen::Matrix<double, 4, 4> UltrasoundCalibrationData::homogeneous_transformation() const
        {
            return f_homogeneous_transformation;
        }

        double UltrasoundCalibrationData::optimization_error() const
        {
            return f_optimization_error;
        }

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

        static std::map<std::string, Type> conversion_to_type{{"landmark", LANDMARK}, {"surface", SURFACE}, {"volume", VOLUME}};
        static const char *conversion_from_type[] = {"landmark", "surface", "volume"};

        RegistrationData::RegistrationData(const std::string &path)
        {
            nlohmann::json registration_data;
            std::ifstream in(path);

            if (!in.is_open())
                throw std::runtime_error("failure to open configuration file");

            decode(in);
        }

        RegistrationData::RegistrationData(std::istream &instream){
            decode(instream);
        }

        void RegistrationData::decode(std::istream &instream){
            nlohmann::json registration_data;
            instream >> registration_data;

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
                    f_moving_to_fixed_transform(row, col) = registration_mat(row, col);
        }

        std::string RegistrationData::timestamp() const
        {
            return f_timestamp;
        }

        Eigen::Matrix<double, 4, 4> RegistrationData::moving_to_fixed_transform() const
        {
            return f_moving_to_fixed_transform;
        }

        double RegistrationData::registration_error() const
        {
            return f_registration_error;
        }

        Type RegistrationData::registration_type() const
        {
            return f_registration_type;
        }

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

        NeedleCalibrationData::NeedleCalibrationData(const std::string &path)
        {
            nlohmann::json needle_calibration_data;
            std::ifstream in(path);

            if (!in.is_open())
                throw std::runtime_error("failure to open configuration file");

            decode(in);
        }

        NeedleCalibrationData::NeedleCalibrationData(std::istream &instream)
        {
            decode(instream);
        }

        void NeedleCalibrationData::decode(std::istream &instream){
            nlohmann::json needle_calibration_data;
            instream >> needle_calibration_data;
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
                    f_needle_homogeneous_transformation(row, col) = calibration_matrix(row, col);
        }

        std::string NeedleCalibrationData::timestamp() const
        {
            return f_timestamp;
        }

        Eigen::Matrix<double, 4, 4> NeedleCalibrationData::needle_calibration() const
        {
            return f_needle_homogeneous_transformation;
        }

        double NeedleCalibrationData::optimization_error() const
        {
            return f_optimization_error;
        }

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

        TrajectorySpecificationData::TrajectorySpecificationData(const std::string &path)
        {
            nlohmann::json trajectory_data;
            std::ifstream in(path);
            if (!in.is_open())
                throw std::runtime_error("failure to open configuration file");
            decode(in);
        }

        TrajectorySpecificationData::TrajectorySpecificationData(std::istream &instream)
        {
            decode(instream);
        }

        void TrajectorySpecificationData::decode(std::istream &instream){
            nlohmann::json trajectory_data;
            instream >> trajectory_data;

            f_timestamp = trajectory_data["timestamp"];

            std::stringstream ss;
            std::string target = trajectory_data["target"];
            ss << target;
            auto eigen_target = curan::utilities::convert_matrix(ss, ',');
            ss = std::stringstream{};
            std::string entry = trajectory_data["entry"];
            ss << entry;
            auto eigen_entry = curan::utilities::convert_matrix(ss, ',');

            ss = std::stringstream{};
            std::string loc_desired_direction = trajectory_data["desired_direction"];
            ss << loc_desired_direction;
            auto eigen_desired_direction = curan::utilities::convert_matrix(ss, ',');

            if (eigen_target.cols() != 1 || eigen_target.rows() != 3)
                throw std::runtime_error("incorrect target dimensions");
            if (eigen_entry.cols() != 1 && eigen_entry.rows() != 3)
                throw std::runtime_error("incorrect entry dimensions");
            if (eigen_desired_direction.cols() != 3 && eigen_desired_direction.rows() != 3)
                throw std::runtime_error("incorrect desired direction dimensions");

            for (size_t i = 0; i < 3; ++i)
            {
                f_target[i] = eigen_target(i, 0);
                f_entrypoint[i] = eigen_entry(i, 0);
            }
            f_path_to_image = trajectory_data["moving_image_directory"];
            f_desired_orientation = eigen_desired_direction;
        }

        std::string TrajectorySpecificationData::timestamp() const
        {
            return f_timestamp;
        }

        Eigen::Matrix<double, 3, 1> TrajectorySpecificationData::target() const
        {
            return f_target;
        }

        Eigen::Matrix<double, 3, 1> TrajectorySpecificationData::entry() const
        {
            return f_entrypoint;
        }

        std::string TrajectorySpecificationData::path_to_image() const
        {
            return f_path_to_image;
        }

        Eigen::Matrix<double, 3, 3> TrajectorySpecificationData::desired_direction() const
        {
            return f_desired_orientation;
        }

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
    
            {
                std::stringstream optimized_values;
                optimized_values << calib.desired_direction().format(desired_matrix_format()) << std::endl;
                calibration_data["desired_direction"] = optimized_values.str();
            }
            calibration_data["moving_image_directory"] = calib.path_to_image();

            os << calibration_data;
            return os;
        }

    }
}
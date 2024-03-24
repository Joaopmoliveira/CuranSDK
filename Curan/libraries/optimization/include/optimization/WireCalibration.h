#ifndef CURAN_WIRE_CALIBRATION_HEADER_FILE_
#define CURAN_WIRE_CALIBRATION_HEADER_FILE_

#include <vector>
#include <Eigen/Core>

// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

namespace curan {
    namespace optim {
        template<typename T, size_t rows, size_t cols>
        struct custom_mat {
            T values[rows * cols];

            custom_mat() {
                for (size_t i = 0; i < rows * cols; ++i)
                    values[i] = 0.0;
            }
        };

        template<typename T>
        void vec2rot(const T* euler_angles, custom_mat<T, 3, 3>& mat) {
            T t1 = cos(euler_angles[2]);
            T t2 = sin(euler_angles[2]);
            T t3 = cos(euler_angles[1]);
            T t4 = sin(euler_angles[1]);
            T t5 = cos(euler_angles[0]);
            T t6 = sin(euler_angles[0]);

            mat.values[0] = t1 * t3;
            mat.values[1] = t2 * t3;
            mat.values[2] = -t4;

            mat.values[3] = t1 * t4 * t6 - t2 * t5;
            mat.values[4] = t1 * t5 + t2 * t4 * t6;
            mat.values[5] = t3 * t6;

            mat.values[6] = t2 * t6 + t1 * t4 * t5;
            mat.values[7] = t2 * t4 * t5 - t1 * t6;
            mat.values[8] = t3 * t5;
        }

        struct Observation {
            custom_mat<double, 3, 1> wire_data;
            custom_mat<double, 3, 4> flange_configuration;
            size_t wire_number = 0;

            Observation() : wire_data{}, flange_configuration{}, wire_number{ 0 } {};
        };

        template<typename T>
        struct PlaneInfo {
            custom_mat<T, 3, 1> plane_theta;
            custom_mat<T, 3, 1> plane_alpha;
            custom_mat<T, 3, 3> flange_rotation;
            custom_mat<T, 3, 1> flange_translation;

            T offset;

            size_t wire_number = 0;

            void  update(const Observation* observation, const T* variables) {
                wire_number = observation->wire_number;
                size_t pointer_offset = 0;
                vec2rot<T>(variables + pointer_offset, flange_rotation);
                pointer_offset += 3;
                flange_translation.values[0] = *(variables + pointer_offset);
                flange_translation.values[1] = *(variables + pointer_offset + 1);
                flange_translation.values[2] = *(variables + pointer_offset + 2);
                pointer_offset += 3;

                //now we need to copy the version related with the position of the robot in cartesian space
                pointer_offset += wire_number * 4;
                custom_mat<T, 3, 3> rot_over_line;
                vec2rot<T>(variables + pointer_offset, rot_over_line);

                plane_theta.values[0] = rot_over_line.values[0];
                plane_theta.values[1] = rot_over_line.values[1];
                plane_theta.values[2] = rot_over_line.values[2];

                plane_alpha.values[0] = rot_over_line.values[3];
                plane_alpha.values[1] = rot_over_line.values[4];
                plane_alpha.values[2] = rot_over_line.values[5];

                offset = *(variables + pointer_offset + 3);
            }
        };

        struct WireData {
            std::vector<Observation> wire_data;

        public:
            std::vector<Eigen::Matrix<double, 4, 4>> parse_flange_data(const std::string& stream_to_record_data, int number_of_observations);

            std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> parse_wire_position(const std::string& stream_to_record_data, int number_of_wires, int number_of_observations);

            friend std::istream& operator>>(std::istream&, WireData&);
        };

        std::istream& operator>>(std::istream& c, WireData& data);

        template<typename T>
        void compute_cost(const PlaneInfo<T>& local_plane, const Observation& observation, T* residual) {
            custom_mat<T, 3, 3> tempmatmat;
            tempmatmat.values[0] = observation.flange_configuration.values[0] * local_plane.flange_rotation.values[0] + observation.flange_configuration.values[3] * local_plane.flange_rotation.values[1] + observation.flange_configuration.values[6] * local_plane.flange_rotation.values[2];
            tempmatmat.values[1] = observation.flange_configuration.values[1] * local_plane.flange_rotation.values[0] + observation.flange_configuration.values[4] * local_plane.flange_rotation.values[1] + observation.flange_configuration.values[7] * local_plane.flange_rotation.values[2];
            tempmatmat.values[2] = observation.flange_configuration.values[2] * local_plane.flange_rotation.values[0] + observation.flange_configuration.values[5] * local_plane.flange_rotation.values[1] + observation.flange_configuration.values[8] * local_plane.flange_rotation.values[2];

            tempmatmat.values[3] = observation.flange_configuration.values[0] * local_plane.flange_rotation.values[3] + observation.flange_configuration.values[3] * local_plane.flange_rotation.values[4] + observation.flange_configuration.values[6] * local_plane.flange_rotation.values[5];
            tempmatmat.values[4] = observation.flange_configuration.values[1] * local_plane.flange_rotation.values[3] + observation.flange_configuration.values[4] * local_plane.flange_rotation.values[4] + observation.flange_configuration.values[7] * local_plane.flange_rotation.values[5];
            tempmatmat.values[5] = observation.flange_configuration.values[2] * local_plane.flange_rotation.values[3] + observation.flange_configuration.values[5] * local_plane.flange_rotation.values[4] + observation.flange_configuration.values[8] * local_plane.flange_rotation.values[5];

            tempmatmat.values[6] = observation.flange_configuration.values[0] * local_plane.flange_rotation.values[6] + observation.flange_configuration.values[3] * local_plane.flange_rotation.values[7] + observation.flange_configuration.values[6] * local_plane.flange_rotation.values[8];
            tempmatmat.values[7] = observation.flange_configuration.values[1] * local_plane.flange_rotation.values[6] + observation.flange_configuration.values[4] * local_plane.flange_rotation.values[7] + observation.flange_configuration.values[7] * local_plane.flange_rotation.values[8];
            tempmatmat.values[8] = observation.flange_configuration.values[2] * local_plane.flange_rotation.values[6] + observation.flange_configuration.values[5] * local_plane.flange_rotation.values[7] + observation.flange_configuration.values[8] * local_plane.flange_rotation.values[8];

            custom_mat<T, 3, 1> tempmatvec;
            tempmatvec.values[0] = tempmatmat.values[0] * observation.wire_data.values[0] + tempmatmat.values[3] * observation.wire_data.values[1] + tempmatmat.values[6] * observation.wire_data.values[2];
            tempmatvec.values[1] = tempmatmat.values[1] * observation.wire_data.values[0] + tempmatmat.values[4] * observation.wire_data.values[1] + tempmatmat.values[7] * observation.wire_data.values[2];
            tempmatvec.values[2] = tempmatmat.values[2] * observation.wire_data.values[0] + tempmatmat.values[5] * observation.wire_data.values[1] + tempmatmat.values[8] * observation.wire_data.values[2];


            custom_mat<T, 3, 1> tempvec;
            tempvec.values[0] = observation.flange_configuration.values[0] * local_plane.flange_translation.values[0] + observation.flange_configuration.values[3] * local_plane.flange_translation.values[1] + observation.flange_configuration.values[6] * local_plane.flange_translation.values[2];
            tempvec.values[1] = observation.flange_configuration.values[1] * local_plane.flange_translation.values[0] + observation.flange_configuration.values[4] * local_plane.flange_translation.values[1] + observation.flange_configuration.values[7] * local_plane.flange_translation.values[2];
            tempvec.values[2] = observation.flange_configuration.values[2] * local_plane.flange_translation.values[0] + observation.flange_configuration.values[5] * local_plane.flange_translation.values[1] + observation.flange_configuration.values[8] * local_plane.flange_translation.values[2];

            custom_mat<T, 3, 1> vec;
            vec.values[0] = observation.flange_configuration.values[9] + tempvec.values[0] + tempmatvec.values[0];
            vec.values[1] = observation.flange_configuration.values[10] + tempvec.values[1] + tempmatvec.values[1];
            vec.values[2] = observation.flange_configuration.values[11] + tempvec.values[2] + tempmatvec.values[2];

            residual[0] = (local_plane.plane_alpha.values[0] * vec.values[0] + local_plane.plane_alpha.values[1] * vec.values[1] + local_plane.plane_alpha.values[2] * vec.values[2]) + local_plane.offset;
            residual[1] = (local_plane.plane_theta.values[0] * vec.values[0] + local_plane.plane_theta.values[1] * vec.values[1] + local_plane.plane_theta.values[2] * vec.values[2]);
        };

        struct WiredResidual {
        private:
            Observation observation;
        public:

            WiredResidual(Observation observation) : observation{ observation }
            { }

            template <typename T>
            bool operator()(const T* const variables, T* residual) const {
                PlaneInfo<T> local_plane;
                local_plane.update(&observation, variables);
                compute_cost<T>(local_plane, observation, residual);
                return true;
            }
        };
    }
}

#endif
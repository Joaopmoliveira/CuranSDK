#ifndef CURAN_WIRE_CALIBRATION_HEADER_FILE_
#define CURAN_WIRE_CALIBRATION_HEADER_FILE_

#include <vector>


namespace curan {
	namespace optim {
		template<typename T, size_t rows, size_t cols>
		struct custom_mat {
			T values[rows * cols];
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
            std::vector<Eigen::Matrix<double, 4, 4>> parse_flange_data(const std::string& stream_to_record_data, int number_of_observations) {
                //get the raw matrix data and then we do post processing
                std::vector<Eigen::Matrix<double, 4, 4>> local_mat;
                local_mat.resize(number_of_observations);
                std::vector<double> values;
                std::string local;
                bool start = false;
                for (const auto& val : stream_to_record_data) {
                    if (std::isdigit(val) || std::isalpha(val) || val == '.' || val == '-') {
                        start = false;
                        local.push_back(val);
                    }
                    else {
                        start = true;
                    }
                    if (start) {
                        double valdouble;
                        const char* const first = local.data();
                        const char* const last = first + local.size();
                        const std::from_chars_result res = std::from_chars(first, last, valdouble);
                        if (res.ec == std::errc{}) {
                            values.push_back(valdouble);
                        }
                        start = false;
                        local = std::string();
                    }
                }
                assertm(values.size() == number_of_observations * 4 * 4, "the values do not match as they should");
                auto begin = values.data();
                size_t offset = 4 * 4;
                for (auto& mat : local_mat) {
                    std::memcpy(mat.data(), begin, 4 * 4 * sizeof(double));
                    begin += offset;
                }
                return local_mat;
            }

            std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> parse_wire_position(const std::string& stream_to_record_data, int number_of_wires, int number_of_observations) {
                //get the raw matrix data and then we do post processing
                       //get the raw matrix data and then we do post processing
                std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> local_mat;
                local_mat.resize(number_of_observations);
                for (auto& mat : local_mat)
                    mat = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, number_of_wires);
                std::vector<double> values;
                std::string local;
                bool start = false;
                for (const auto& val : stream_to_record_data) {
                    if (std::isdigit(val) || std::isalpha(val) || val == '.' || val == '-') {
                        start = false;
                        local.push_back(val);
                    }
                    else {
                        start = true;
                    }
                    if (start) {
                        double valdouble;
                        const char* const first = local.data();
                        const char* const last = first + local.size();
                        const std::from_chars_result res = std::from_chars(first, last, valdouble);
                        if (res.ec == std::errc{}) {
                            values.push_back(valdouble);
                        }
                        start = false;
                        local = std::string();
                    }
                }
                assertm(values.size() == number_of_observations * 3 * number_of_wires, "the values do not match as they should");
                auto begin = values.data();
                const size_t offset = 3 * number_of_wires;
                for (auto& mat : local_mat) {
                    std::memcpy(mat.data(), begin, offset * sizeof(double));
                    begin += offset;
                }
                return local_mat;
            }

            friend std::istream& operator>>(std::istream&, WireData&);
        };

        std::istream& operator>>(std::istream& c, WireData& data) {
            nlohmann::json model_to_read = nlohmann::json::parse(c);
            int number_of_wires = model_to_read["number_of_wires"].get<int>();
            int number_of_observations = model_to_read["number_of_observations"].get<int>();
            assertm(number_of_observations > 0, "expected observations larger than zero");
            assertm(number_of_wires > 0, "expected number of wires larger than zero");
            std::stringstream stream_to_record_data;
            stream_to_record_data << model_to_read["flange_data"];
            auto mat_flange_data = data.parse_flange_data(stream_to_record_data.str(), number_of_observations);
            stream_to_record_data.str("");
            stream_to_record_data.clear();
            stream_to_record_data << model_to_read["wire_image_position"];
            auto mat_wire_data = data.parse_wire_position(stream_to_record_data.str(), number_of_wires, number_of_observations);

            assertm(mat_flange_data.size() == mat_wire_data.size(), "the length of observations and wire geometries must be the same");
            auto begin_mat_flange_data = mat_flange_data.begin();
            auto begin_mat_wire_data = mat_wire_data.begin();

            data.wire_data.reserve(number_of_wires * number_of_observations);

            for (; begin_mat_flange_data != mat_flange_data.end(); ++begin_mat_flange_data, ++begin_mat_wire_data) {
                for (size_t wire_number = 0; wire_number < number_of_wires; ++wire_number) {
                    Observation localobservation;
                    auto temp = *begin_mat_flange_data;

                    localobservation.flange_configuration.values[0] = temp(0, 0);
                    localobservation.flange_configuration.values[1] = temp(1, 0);
                    localobservation.flange_configuration.values[2] = temp(2, 0);

                    localobservation.flange_configuration.values[3] = temp(0, 1);
                    localobservation.flange_configuration.values[4] = temp(1, 1);
                    localobservation.flange_configuration.values[5] = temp(2, 1);

                    localobservation.flange_configuration.values[6] = temp(0, 2);
                    localobservation.flange_configuration.values[7] = temp(1, 2);
                    localobservation.flange_configuration.values[8] = temp(2, 2);

                    localobservation.flange_configuration.values[9] = temp(0, 3);
                    localobservation.flange_configuration.values[10] = temp(1, 3);
                    localobservation.flange_configuration.values[11] = temp(2, 3);

                    localobservation.wire_number = wire_number;

                    auto temp2 = (*begin_mat_wire_data).col(wire_number);

                    localobservation.wire_data.values[0] = temp2(0, 0);
                    localobservation.wire_data.values[1] = temp2(1, 0);
                    localobservation.wire_data.values[2] = temp2(2, 0);
                    data.wire_data.push_back(localobservation);
                }
            }
            return c;
        };

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
        }

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
	}
}

#endif
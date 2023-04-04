#include "optimization/WireCalibration.h"

namespace curan {
namespace optim {

std::vector<Eigen::Matrix<double, 4, 4>> WireData::parse_flange_data(const std::string& stream_to_record_data, int number_of_observations) {
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
		} else {
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

std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> WireData::parse_wire_position(const std::string& stream_to_record_data, int number_of_wires, int number_of_observations) {
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
		} else {
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

}
}
#include "robotutils/FilterRippleFirstHarmonic.h"

namespace curan {
namespace robotic {

void update_filter_properties(FilterProperties& filter_properties, const Observation& observation){
	filter_properties.damper = (std::abs(observation.current_vel) < 0.2) ? std::pow(observation.current_vel / 0.2,2) : 1.0;
	filter_properties.log_filtered_frequency = std::log10(std::abs(observation.current_vel)*filter_properties.frequency);
	filter_properties.delta = std::abs(observation.current_delta);
}

void shift_filter_data(FilterData& data, const double& filtered_torque) {
	data.y[0] = data.y[1];
	data.y[1] = data.y[2];

	data.yf[0] = data.yf[1];
	data.yf[1] = data.yf[2];
	data.yf[2] = filtered_torque;
}

void update_filter_data(FilterData& data, const double& torque) {
	data.y[2] = torque;
}

double filter_implementation(const FilterProperties& properties, FilterData& data) {
	const double A = 2.0 * properties.width * properties.frequency * properties.delta;
	const double B = 4.0 + properties.frequency * properties.frequency * properties.delta * properties.delta;
	data.yf[2] = (properties.damper / (A + B)) * (A * data.y[2] - A * data.y[0] - 2.0 * (B - 8.0) * data.yf[1] - (B - A) * data.yf[0]);
	return data.yf[2];
}

}
}
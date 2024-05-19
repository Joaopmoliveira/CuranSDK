#include "FilterRipple.h"

void update_filter_properties(FilterProperties& filter_properties, const Observation& observation) {
	filter_properties.damper = (std::abs(observation.current_vel) < 0.2) ? std::abs(observation.current_vel) / 0.2 : 1.0;
}

void shift_filter_data(FilterData& data, const Observation& observation) {
	data.y[0] = data.y[1];
	data.y[1] = data.y[2];

	data.yf[0] = data.yf[1];
	data.yf[1] = data.yf[2];
}

void update_filter_data(FilterData& data, const Observation& observation) {
	data.y[2] = observation.current_torque;
	data.delta = std::abs(observation.current_delta);
}

double filter_implementation(const FilterProperties& properties, FilterData& data) {
	const double A = 2.0 * properties.width * properties.frequency * data.delta;
	const double B = 4.0 + properties.frequency * properties.frequency * data.delta * data.delta;
	data.yf[2] = (properties.damper / (A + B)) * (A * data.y[2] - A * data.y[0] - 2.0 * (B - 8.0) * data.yf[1] - (B - A) * data.yf[0]);
	return data.yf[2];
}

double run_filter(FilterData& data, FilterProperties& properties, const Observation& observation) {
	update_filter_properties(properties, observation);
	update_filter_data(data, observation);
	double val = filter_implementation(properties, data);
	shift_filter_data(data, observation);
	return val;
}
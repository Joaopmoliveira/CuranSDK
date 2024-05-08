#include "robotutils/FilterRippleFirstHarmonic.h"

namespace curan {
namespace robotic {

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
	const double damper = properties.damper*properties.crosstalk_damper;
	data.yf[2] = (damper / (A + B)) * (A * data.y[2] - A * data.y[0] - 2.0 * (B - 8.0) * data.yf[1] - (B - A) * data.yf[0]);
	return data.yf[2];
}

}
}
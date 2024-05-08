#ifndef CURAN_FILTER_RIPPLE_
#define CURAN_FILTER_RIPPLE_

#include <array>
#include <vector>
#define _USE_MATH_DEFINES // for C++
#include <cmath>

namespace curan {
namespace robotic {

struct FilterProperties {
	/*
	The damper is the scalling factor responsible for 
	killing the filter when it becomes unstable, e.g. when its stopped
	*/
	double damper;

	/*
	The damper is the scalling factor responsible for 
	killing the filter when it becomes unstable, e.g. when its stopped
	*/
	double crosstalk_damper;

	/*
	This controls the bandwitdh of the fixed sample space filter, 
	the larger the bandwidth the more content of the real filter 
	you end up removing from the signal
	*/
	double width;

	/*
	The frequency is contant in the sample space, and it constrol
	where, from the prespective of the motor we kill the ripple, 
	usually for most joints of the LBR Med we have a gear reduction ratio
	of 160, and because the frequency runs at twice the frequency of 
	the motor then we have 160*2=360 is the center frequency of most filters 
	trying to filter the first harmonic 
	*/
	double frequency;

	/*
	This is a helper value to keep track in the time domain of where we are filtering currently
	*/
	double varying_frequency;
	FilterProperties(double width = 5.0, double frequency = 320.0) : damper{1.0}, crosstalk_damper{1.0}, width{ width }, frequency{ frequency }, varying_frequency{1.0} {};
};

struct FilterData {
	std::array<double, 3> yf = { 0.0,0.0,0.0 };
	std::array<double, 3> y = { 0.0,0.0,0.0 };
	double delta = 0.0;
};

struct Observation {
	double current_vel;
	double current_delta;
	double current_torque;

	Observation(double current_vel, double current_delta, double current_torque) : current_vel{ current_vel }, current_delta{ current_delta }, current_torque{ current_torque }
	{}
};

/*
Cross talk mitigation is a scheme to avoid the delay between multiple 
joints overlaping with each other, which add delay between the signal and the 
filtered signal. All the methods are templated so as to avoid branching depending
on the mitingation strategy
*/
enum CrossTalkMitigation{
	ACTIVE,
	INACTIVE
};

template<CrossTalkMitigation mitigation_strategy>
void update_filter_properties(FilterProperties& filter_properties, const Observation& observation){
	if constexpr (mitigation_strategy==CrossTalkMitigation::INACTIVE){
		filter_properties.damper = (std::abs(observation.current_vel) < 0.2) ? std::pow(observation.current_vel / 0.2,2) : 1.0;
		filter_properties.varying_frequency = std::abs(observation.current_vel)*filter_properties.varying_frequency;
		filter_properties.crosstalk_damper = 1.0;
	} else {
		filter_properties.damper = (std::abs(observation.current_vel) < 0.2) ? std::pow(observation.current_vel / 0.2,2) : 1.0;
		filter_properties.varying_frequency = std::abs(observation.current_vel)*filter_properties.varying_frequency;
		filter_properties.crosstalk_damper = 1.0;
	}
}

void shift_filter_data(FilterData& data, const Observation& observation);

void update_filter_data(FilterData& data, const Observation& observation);

double filter_implementation(const FilterProperties& properties, FilterData& data);

template<CrossTalkMitigation mitigation_strategy>
double run_filter(FilterData& data, FilterProperties& properties, const Observation& observation){
	if constexpr (mitigation_strategy==CrossTalkMitigation::INACTIVE){
		update_filter_properties<mitigation_strategy>(properties, observation);
		update_filter_data(data, observation);
		double val = filter_implementation(properties, data);
		shift_filter_data(data, observation);
		return val;
	} else {
		update_filter_properties<mitigation_strategy>(properties, observation);
		update_filter_data(data, observation);
		double val = filter_implementation(properties, data);
		shift_filter_data(data, observation);
		return val;
	}
}

}
}

#endif
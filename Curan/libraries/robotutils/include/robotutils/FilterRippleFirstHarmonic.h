#ifndef CURAN_FILTER_RIPPLE_
#define CURAN_FILTER_RIPPLE_

#include <array>
#include <vector>
#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <iterator>

namespace curan {
namespace robotic {

constexpr double covariance = 10.0;

struct FilterProperties {

	double damper;
	double cross_overlap;
	double width;
	double frequency;
	double filtered_frequency;

	FilterProperties(double in_width = 5.0, double in_frequency = 320.0) : filtered_frequency{1.0} , damper{1.0}, cross_overlap{1.0} , width{ in_width }, frequency{ in_frequency } {};
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
*/
enum CrossTalkMitigation{
	ACTIVE,
	INACTIVE
};

void update_filter_properties(FilterProperties& filter_properties, const Observation& observation);

template<typename Iter>
void update_filter_properties(FilterProperties& filter_properties, const Observation& observation, Iter first, Iter last){
	filter_properties.damper = (std::abs(observation.current_vel) < 0.2) ? std::pow(observation.current_vel / 0.2,2) : 1.0;
	filter_properties.filtered_frequency = std::abs(observation.current_vel)*filter_properties.frequency;
	filter_properties.cross_overlap = 1.0;
	while(first!=last){
		typename std::iterator_traits<Iter>::value_type tmp = *first;
		filter_properties.cross_overlap = std::min(filter_properties.cross_overlap,1.0-std::exp((-1.0/(covariance*covariance))*std::pow(filter_properties.filtered_frequency-tmp,2)));
		++first;
	}
}

void shift_filter_data(FilterData& data, const Observation& observation);

void update_filter_data(FilterData& data, const Observation& observation);

double filter_implementation(const FilterProperties& properties, FilterData& data);

double run_filter(FilterData& data, FilterProperties& properties, const Observation& observation);

template<typename Iter>
double run_filter(FilterData& data, FilterProperties& properties, const Observation& observation, Iter first, Iter last){
	update_filter_properties(properties, observation,first,last);
	update_filter_data(data, observation);
	double val = filter_implementation(properties, data);
	shift_filter_data(data, observation);
	return val;
}

}
}

#endif
#ifndef _RIPPLE_FILTER_H
#define _RIPPLE_FILTER_H

#include <array>
#include <vector>
#define _USE_MATH_DEFINES // for C++
#include <cmath>

struct FilterProperties {
	double damper = 1;
	double width;
	double frequency;

	FilterProperties(double width, double frequency) : width{ width }, frequency{ frequency } {};
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

void update_filter_properties(FilterProperties& filter_properties, const Observation& observation);

void shift_filter_data(FilterData& data, const Observation& observation);

void update_filter_data(FilterData& data, const Observation& observation);

double filter_implementation(FilterProperties& properties, FilterData& data);

double run_filter(FilterData& data, FilterProperties& properties, const Observation& observation);

#endif
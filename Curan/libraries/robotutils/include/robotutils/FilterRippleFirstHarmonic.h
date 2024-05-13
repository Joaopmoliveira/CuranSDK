#ifndef CURAN_FILTER_RIPPLE_
#define CURAN_FILTER_RIPPLE_

#include <array>
#include <vector>
#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <iterator>

namespace curan {
namespace robotic {

struct FilterProperties {
	/*
	The damper is the scalling factor responsible for 
	killing the filter when it becomes unstable, e.g. when its stopped
	*/
	double damper;

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
	The log filtered frequency is the variable frequency in time which 
	we are currently filtering in log coodinates for fair distance comparisons
	*/
	double log_filtered_frequency;

	/*
	delta is equivalent to sample time with the caviant that it is constantly changing
	*/
	double delta = 0.0;

	FilterProperties(double in_width = 5.0, 
					 double in_frequency = 320.0) : 
							log_filtered_frequency{1.0} , 
							damper{1.0}, 
							delta{0.0},
							width{ in_width }, 
							frequency{ in_frequency } 
	{};
};

/*
Filter data is just a collection of previous observations and filtered results 
*/
struct FilterData {
	std::array<double, 3> yf = { 0.0,0.0,0.0 };
	std::array<double, 3> y = { 0.0,0.0,0.0 };
};

/*
The observation struct is composed of two measurments
the velocity and the delta of the joint we wish to filter
*/
struct Observation {
	double current_vel;
	double current_delta;

	Observation(double in_current_vel, 
				double in_current_delta) : 
							current_vel{ in_current_vel }, 
							current_delta{ in_current_delta }
	{}
};

void update_filter_properties(FilterProperties& filter_properties, const Observation& observation);

void shift_filter_data(FilterData& data, const double& filtered_torque);

void update_filter_data(FilterData& data, const double& torque);

double filter_implementation(const FilterProperties& properties, FilterData& data);

}
}

#endif
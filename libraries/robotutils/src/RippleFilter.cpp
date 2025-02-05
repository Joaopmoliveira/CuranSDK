#include "robotutils/RippleFilter.h"

#include <cmath>

namespace curan{
namespace robotic{
namespace ripple{
namespace detail{

void update_filter_properties(Properties& properties,const Observation& observation){
    double damper_trigger = 0.2*properties.frequency;
    double filtered_frequency = std::abs(observation.current_vel)*properties.frequency;
    properties.damper =  filtered_frequency < damper_trigger ? std::pow(filtered_frequency/damper_trigger,2) : 1.0;
    properties.delta = std::abs(observation.current_delta);
}

void shift_filter_data(Data& data, const double& filtered_torque){
    data.y[0] = data.y[1];
    data.y[1] = data.y[2];

    data.y_f[0] = data.y_f[1];
    data.y_f[1] = data.y_f[2];
    data.y_f[2] = filtered_torque;
}

void update_filter_data(Data& data, const double& torque){
    data.y[2] = torque;
}

double execute(const Properties& props, Data& data){
    const double A = 2.0 * props.width * props.frequency * props.delta;
    const double B = 4.0 + props.frequency * props.frequency * props.delta * props.delta;
    data.y_f[2] = (props.damper/(A+B)) * (A*data.y[2] - A*data.y[0] - 2.0*(B-8.0) * data.y_f[1] - (B-A)*data.y_f[0] );
    return data.y_f[2];
}
}

}
}
}
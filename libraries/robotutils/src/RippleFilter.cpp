#include "robotutils/RippleFilter.h"

#include <cmath>

namespace curan{
namespace robotic{
namespace ripple{

void shift_filter_data(Data& data){
    data.y[0] = data.y[1];
    data.y[1] = data.y[2];

    data.y_f[0] = data.y_f[1];
    data.y_f[1] = data.y_f[2];
    data.y_f[2] = 0.0;
}

void update_filter_data(Data& data, const double& torque){
    data.y[2] = torque;
}

std::tuple<double,double> execute(const Properties& props,const Damper& damper, Data& data){
    const double A = 2.0 * props.width * props.frequency * damper.delta;
    const double B = 4.0 + props.frequency * props.frequency * damper.delta * damper.delta;
    double y_f2 = (damper.value/(A+B)) * (A*data.y[2] - A*data.y[0] - (2.0*(B-8.0) * data.y_f[1] + (B-A)*data.y_f[0]) );
    data.y_f[2] = y_f2;
    return std::make_tuple(y_f2,0.0);
}

}
}
}
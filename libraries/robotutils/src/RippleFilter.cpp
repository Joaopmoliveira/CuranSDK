#include "robotutils/RippleFilter.h"

#include <cmath>

namespace curan{
namespace robotic{
namespace ripple{

void shift_filter_data(Data& data){
    data.y[0] = data.y[1];
    data.y[1] = data.y[2];

    data.dy[0] = data.dy[1];
    data.dy[1] = data.dy[2];

    data.y_f[0] = data.y_f[1];
    data.y_f[1] = data.y_f[2];
    data.y_f[2] = 0.0;

    data.dy_f[0] = data.dy_f[1];
    data.dy_f[1] = data.dy_f[2];
    data.dy_f[2] = 0.0;
}

void update_filter_data(Data& data, const double& torque, const double& dtorque){
    data.y[2] = torque;
    data.dy[2] = dtorque;
}

std::tuple<double,double> execute(const Properties& props,const Damper& damper, Data& data){
    const double A = 2.0 * props.width * props.frequency * damper.delta;
    const double B = 4.0 + props.frequency * props.frequency * damper.delta * damper.delta;
    double y_f2 = (1.0/(A+B)) * (A*data.y[2] - A*data.y[0] - 2.0*(B-8.0) * data.y_f[1] - (B-A)*data.y_f[0] );
    double dy_f2 = (1.0/(A+B)) * (A*data.dy[2] - A*data.dy[0] - 2.0*(B-8.0) * data.dy_f[1] - (B-A)*data.dy_f[0] );
    data.y_f[2] = damper.value*y_f2;
    data.dy_f[2] = damper.value*dy_f2;
    return std::make_tuple(data.y_f[2],data.dy_f[2]);
}

}
}
}
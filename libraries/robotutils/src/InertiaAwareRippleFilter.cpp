#include "robotutils/InertiaAwareRippleFilter.h"

#include <cmath>

namespace curan{
namespace robotic{
namespace massripple{

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

}
}
}
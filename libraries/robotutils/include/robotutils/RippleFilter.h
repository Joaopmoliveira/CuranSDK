#ifndef CURAN_RIPPLE_FILTER_
#define CURAN_RIPPLE_FILTER_

#include <tuple>
#include <array>

namespace curan {
namespace robotic {
namespace ripple{
        struct Properties{
            double damper_prev;
            double damper;
            double width;
            double frequency;
            double delta;

            Properties(double in_width = 5.0, double in_frequency = 320.0) : damper_prev{0.0},damper{0.0}, width{in_width}, frequency{in_frequency}, delta{0.0}  {};

        };

        struct Data{
            std::array<double,3> y_f = {0.0,0.0,0.0};
            std::array<double,3> y = {0.0,0.0,0.0};
        };

        struct Observation{
            double current_vel;
            double current_delta;
        };

        void update_filter_properties(Properties& properties,const Observation& observation);

        void shift_filter_data(Data& data, const double& filtered_torque);

        void update_filter_data(Data& data, const double& torque);

        /*
        this function returns a tuple with the following rules:
        [ filtered_torque , ficticious_torque_deriv_by_activation_function ]

        basically the derivative of the filtered torque has two components, the derivative due to the changes in the real torque
        and the changes due to the scalling function that shuts down the filter
        */
        std::tuple<double,double> execute(const Properties& props, Data& data);

}
}
}

#endif
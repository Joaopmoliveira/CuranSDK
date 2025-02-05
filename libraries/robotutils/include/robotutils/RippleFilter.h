#ifndef CURAN_RIPPLE_FILTER
#define CURAN_RIPPLE_FILTER

#include <array>

namespace curan {
namespace robotic {
namespace ripple{
namespace detail{

        struct Properties{
            double damper;
            double width;
            double frequency;
            double delta;

            Properties(double in_width = 5.0, double in_frequency = 320.0) : damper{0.0}, width{in_width}, frequency{in_frequency}, delta{0.0}  {};

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

        double execute(const Properties& props, Data& data);
} // namespace detail



}
}
}

#endif
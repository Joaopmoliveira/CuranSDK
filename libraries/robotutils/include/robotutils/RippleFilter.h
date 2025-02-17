#ifndef CURAN_RIPPLE_FILTER_
#define CURAN_RIPPLE_FILTER_

#include <tuple>
#include <array>
#include <Eigen/Dense>

namespace curan {
namespace robotic {
namespace ripple{

        struct Damper{
            double damper_derivative = 0.0;
            double value = 0.0; 
            double prev_vel = 0.0; 
            double delta = 0.0;

            template<size_t siz>
            void compute(const Eigen::Matrix<double,siz,1>& dq,double sample_time){
                const double trigger_point = 0.1;
                double max_vel = dq.array().abs().maxCoeff();
                double possible_damper_value = std::pow(max_vel/trigger_point,2.0);
                if(possible_damper_value < 1.0){
                    value = possible_damper_value;
                    damper_derivative = (2.0*max_vel)/(trigger_point*trigger_point)*((max_vel-prev_vel)/sample_time);
                } else {
                    value = 1.0;
                    damper_derivative = 0.0;
                }
                prev_vel = max_vel;
                delta = std::abs(max_vel*sample_time);
            };

        };

        struct Properties{
            double width;
            double frequency;
            Properties(double in_width = 5.0, double in_frequency = 320.0) : width{in_width}, frequency{in_frequency}{};
        };

        struct Data{
            std::array<double,3> y_f = {0.0,0.0,0.0};
            std::array<double,3> y = {0.0,0.0,0.0};
            
            std::array<double,3> dy_f = {0.0,0.0,0.0};
            std::array<double,3> dy = {0.0,0.0,0.0};
        };

        void shift_filter_data(Data& data);

        void update_filter_data(Data& data, const double& torque, const double& dtorque);

        /*
        this function returns a tuple with the following rules:
        [ filtered_torque , ficticious_torque_deriv_by_activation_function ]

        basically the derivative of the filtered torque has two components, the derivative due to the changes in the real torque
        and the changes due to the scalling function that shuts down the filter
        */
        std::tuple<double,double> execute(const Properties& props,const Damper& damper, Data& data);

}
}
}

#endif
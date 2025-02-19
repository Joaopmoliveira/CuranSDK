#ifndef CURAN_RIPPLE_FILTER_
#define CURAN_RIPPLE_FILTER_

#include <tuple>
#include <array>
#include <numbers>
#include <Eigen/Dense>

namespace curan {
namespace robotic {
namespace ripple{

        struct Damper{
            double damper_derivative = 0.0;
            double value = 0.0; 
            double prev_value = 0.0; 
            double delta = 0.0;

            template<size_t siz>
            void compute(const Eigen::Matrix<double,siz,1>& dq,double sample_time){
                double max_vel = dq.array().abs().maxCoeff();
                //value = 0.5-0.5*cos(std::min(3.14159265358979323,std::abs(max_vel)*20.0));
                value = std::min(std::abs(max_vel)*5,1.0);
                damper_derivative = (value-prev_value)/sample_time;
                prev_value = value;
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
            
            //std::array<double,3> dy_f = {0.0,0.0,0.0};
            //std::array<double,3> dy = {0.0,0.0,0.0};
        };

        void shift_filter_data(Data& data);

        void update_filter_data(Data& data, const double& torque);

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
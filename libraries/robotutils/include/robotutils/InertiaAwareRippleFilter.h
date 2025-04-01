#ifndef CURAN_MASSRIPPLE_FILTER_
#define CURAN_MASSRIPPLE_FILTER_

#include <tuple>
#include <array>
#include <numbers>
#include <Eigen/Dense>

namespace curan {
namespace robotic {
namespace massripple{

        template<size_t n_joints>
        struct Damper{
            double value = 0.0; 
            double delta = 0.0;
            size_t critical_index = 0;
            
            Eigen::Matrix<double,n_joints,n_joints> B;
            Eigen::Matrix<double,n_joints,n_joints> K;
            Eigen::Matrix<double,n_joints,n_joints> D;
            Eigen::Matrix<double,n_joints,n_joints> Kt;
            Eigen::Matrix<double,n_joints,n_joints> R;

            Damper(const Eigen::Matrix<double,n_joints,n_joints>& in_B,
                    const Eigen::Matrix<double,n_joints,n_joints>& in_K,
                    const Eigen::Matrix<double,n_joints,n_joints>& in_D,
                    const Eigen::Matrix<double,n_joints,n_joints>& in_Kt,
                    const Eigen::Matrix<double,n_joints,n_joints>& in_R) : B{in_B}, K{in_K}, D{in_D}, Kt{in_Kt}, R{in_R}
            {
            }


            void compute(const Eigen::Matrix<double,n_joints,n_joints>& M,const Eigen::Matrix<double,n_joints,1>& dq,double sample_time){
                Eigen::Matrix<double,n_joints,1> w = 2*R(dq.array().abs().matrix());
                Eigen::Matrix<double,n_joints,1> G = Eigen::Matrix<double,n_joints,1>::Zero();
                for(double& g : G.rowwise()){
                    
                }
                double amplitude = G.array().abs().maxCoeff(critical_index);
                value = std::min((dq[critical_index]*dq[critical_index])/(0.1*0.1),1.0);
                delta = std::abs(dq[critical_index]*sample_time);
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
        };

        void shift_filter_data(Data& data);

        void update_filter_data(Data& data, const double& torque);

        /*
        this function returns a tuple with the following rules:
        [ filtered_torque , ficticious_torque_deriv_by_activation_function ]

        basically the derivative of the filtered torque has two components, the derivative due to the changes in the real torque
        and the changes due to the scalling function that shuts down the filter
        */
        template<size_t n_joints>
        double execute(const std::array<Properties,n_joints>& props,const Damper<n_joints>& damper, Data& data){
            const double A = 2.0 * props[damper.critical_index].width * props[damper.critical_index].frequency * damper.delta;
            const double B = 4.0 + props[damper.critical_index].frequency * props[damper.critical_index].frequency * damper.delta * damper.delta;
            double y_f2 = (damper.value/(A+B)) * (A*data.y[2] - A*data.y[0] - (2.0*(B-8.0) * data.y_f[1] + (B-A)*data.y_f[0]) );
            data.y_f[2] = y_f2;
            return y_f2;
        }

}
}
}

#endif
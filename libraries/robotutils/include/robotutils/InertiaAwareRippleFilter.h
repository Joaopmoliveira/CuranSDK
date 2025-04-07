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
            std::array<double,n_joints> values;
            std::array<double,n_joints> deltas;
            std::array<double,n_joints> critical_indices;
            
            std::array<double,n_joints> B;
            std::array<double,n_joints> K;
            std::array<double,n_joints> D;
            std::array<double,n_joints> Kt;
            Eigen::Matrix<double,n_joints,1> R;

            Damper(const std::array<double,n_joints>& in_B,
                    const std::array<double,n_joints>& in_K,
                    const std::array<double,n_joints>& in_D,
                    const std::array<double,n_joints>& in_Kt,
                    const std::array<double,n_joints>& in_R) : B{in_B}, 
                                                               K{in_K}, 
                                                               D{in_D}, 
                                                               Kt{in_Kt}, 
                                                               R{Eigen::Matrix<double,n_joints,1>::Zero()}
            {
                for(size_t i = 0; i < n_joints ; ++i)
                    R(i,i) = in_R[i];
            }


            void compute(const Eigen::Matrix<double,n_joints,n_joints>& M,const Eigen::Matrix<double,n_joints,1>& dq,double sample_time){
                Eigen::Matrix<double,n_joints,1> w = (2*R.array()*(dq.array().abs())).matrix();
                Eigen::Matrix<double,n_joints,1> G = Eigen::Matrix<double,n_joints,1>::Zero();
                for(size_t crit_j = 0; crit_j < n_joints; ++crit_j ){
                    double largest_G = -1;
                    for(size_t j = 0; j < n_joints; ++j){
                        G[j] = Kt[j]/std::sqrt(std::pow(1+Kt[j]+B[j]/M(crit_j,crit_j)-w[j]*w[j]/(K[j]*B[j]),2)+std::pow(w[j]*D[j]/K[j]+w[j]*B[j]*D[j]/(K[j]*M(crit_j,crit_j)),2));
                        if(G[j]>largest_G){
                            largest_G = G[j];
                            critical_indices[crit_j] = j;
                        }
                    }
                    values[crit_j] = std::min((dq[critical_indices[crit_j]]*dq[critical_indices[crit_j]])/(0.1*0.1),1.0);
                    deltas[crit_j] = std::abs(dq[critical_indices[crit_j]]*sample_time);
                }

            };

        };

        struct Properties{
            double width;
            Properties(double in_width = 5.0) : width{in_width}{}
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
        double execute(const std::array<Properties,n_joints>& props,const Damper<n_joints>& damper, Data& data, size_t joint_index){
            const double A = 2.0 * props[damper.critical_indices[joint_index]].width*2.0* damper.R[damper.critical_indices[joint_index]]*damper.deltas[joint_index];
            const double B = 4.0 + 2.0*damper.R[damper.critical_indices[joint_index]]*2.0*damper.R[damper.critical_indices[joint_index]]*damper.deltas[joint_index]*damper.deltas[joint_index];
            double y_f2 = (damper.values[joint_index]/(A+B)) * (A*data.y[2] - A*data.y[0] - (2.0*(B-8.0) * data.y_f[1] + (B-A)*data.y_f[0]) );
            data.y_f[2] = y_f2;
            return y_f2;
        }

}
}
}

#endif
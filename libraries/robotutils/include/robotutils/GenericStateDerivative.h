#ifndef CURAN_GENERIC_STATE_DERIVATIVE_
#define CURAN_GENERIC_STATE_DERIVATIVE_

#include <Eigen/Dense>
#include <tuple>

namespace curan {
namespace robotic {

template<size_t state_size>
struct LowPassDerivativeFilter{
    Eigen::Matrix<double,state_size,3> raw_state;
    Eigen::Matrix<double,state_size,3> filtered_state;

    Eigen::Matrix<double,state_size,3> deriv_raw_state;
    Eigen::Matrix<double,state_size,3> deriv_filtered_state;
    bool is_first = true;

    std::tuple<Eigen::Matrix<double,state_size,1>,Eigen::Matrix<double,state_size,1>> update(Eigen::Matrix<double,state_size,1> in_raw_state, double sample_time){
        return update(in_raw_state,Eigen::Matrix<double,state_size,1>::Zero(),sample_time);
    }

    std::tuple<Eigen::Matrix<double,state_size,1>,Eigen::Matrix<double,state_size,1>> update(Eigen::Matrix<double,state_size,1> in_raw_state, Eigen::Matrix<double,state_size,1> partial_removal, double sample_time){
        if(is_first){
            is_first = false;
            raw_state.col(2) = in_raw_state;
            raw_state.col(1) = raw_state.col(2);
            raw_state.col(0) = raw_state.col(2);
        } else {
            raw_state.col(2) = in_raw_state;
        }     
        filtered_state.col(2) = 0.222955 * raw_state.col(2)+
                                0.445910 * raw_state.col(1)+
                                0.222955 * raw_state.col(0)+
                                0.295200 * filtered_state.col(1)+
                               -0.187020 * filtered_state.col(0);

        deriv_raw_state.col(2) = (1.0/sample_time)*(raw_state.col(2)-raw_state.col(1))-partial_removal;
        deriv_filtered_state.col(2) = 0.222955 * deriv_raw_state.col(2)+
                                0.445910 * deriv_raw_state.col(1)+
                                0.222955 * deriv_raw_state.col(0)+
                                0.295200 * deriv_filtered_state.col(1)+
                               -0.187020 * deriv_filtered_state.col(0);

        for (size_t i = 0; i < 2; ++i){
            raw_state.col(i) = raw_state.col(i + 1);
            filtered_state.col(i) = filtered_state.col(i + 1);

            deriv_raw_state.col(i) = deriv_raw_state.col(i + 1);
            deriv_filtered_state.col(i) = deriv_filtered_state.col(i + 1);

        }
        return {filtered_state.col(2),deriv_filtered_state.col(2)};
    }
};


}
}

#endif
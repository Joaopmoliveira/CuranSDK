#ifndef CURAN_GENERIC_STATE_DERIVATIVE_
#define CURAN_GENERIC_STATE_DERIVATIVE_

#include <Eigen/Dense>
#include <tuple>

namespace curan {
namespace robotic {

template<size_t filter_size>
struct LowPassDerivativeFilter{
    Eigen::Matrix<double,filter_size,3> raw_state;
    Eigen::Matrix<double,filter_size,3> filtered_state;
    Eigen::Matrix<double,1,5> coefficient;

    bool is_first = true;

    LowPassDerivativeFilter(double nyquist_frequency,double cutoff){
        double c = std::cos(0.5*cutoff/nyquist_frequency)/std::sin(0.5*cutoff/nyquist_frequency);
        coefficient = Eigen::Matrix<double,1,5>::Zero();
        double a0 = c*c*2+1.4142*c+1;
        coefficient << 1/a0 , 2/a0 , 1/a0 , (2-2*c*c)/a0 , (c*c-1.4142*c+1)/a0; 
    }

    Eigen::Matrix<double,filter_size,1> update(Eigen::Matrix<double,filter_size,1> in_raw_state, double sample_time){
        if(is_first){
            is_first = false;
            raw_state.col(2) = in_raw_state;
            raw_state.col(1) = raw_state.col(2);
            raw_state.col(0) = raw_state.col(2);
        } else {
            raw_state.col(2) = in_raw_state;
        }     
        filtered_state.col(2) = coefficient[0] * raw_state.col(2)+
                                coefficient[1] * raw_state.col(1)+
                                coefficient[2] * raw_state.col(0)+
                                coefficient[3] * filtered_state.col(1)+
                                coefficient[4] * filtered_state.col(0);
        for (size_t i = 0; i < 2; ++i){
            raw_state.col(i) = raw_state.col(i + 1);
            filtered_state.col(i) = filtered_state.col(i + 1);
        }
        return filtered_state.col(2);
    }
};


}
}

#endif
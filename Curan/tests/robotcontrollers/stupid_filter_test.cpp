
#include "robotutils/FilterRippleFirstHarmonic.h"

int main(){
    std::pair<curan::robotic::FilterData,curan::robotic::FilterProperties> first_harmonic;
    auto printer = [&](std::string s){
        std::printf("%s",s.data());
        std::printf("\n");
        for(size_t i = 0; i < 3; ++i){
            std::printf("%.2f ",first_harmonic.first.y[i]);
        }
        std::printf("\n");
        for(size_t i = 0; i < 3; ++i){
            std::printf("%.2f ",first_harmonic.first.yf[i]);
        }
        std::printf("\n");
    };

    
    first_harmonic.second.frequency = 320.0;
    double time = 1.0;

    std::array<double,10> y_pre = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0};
    
    for(auto y : y_pre){
        const curan::robotic::Observation obser_i_j{1,0.01};
        update_filter_properties(first_harmonic.second, obser_i_j);
        shift_filter_data(first_harmonic.first,0.0);  
	    update_filter_data(first_harmonic.first, y);
        printer("before:");     
	    double filtered_torque_joint_i = filter_implementation(first_harmonic.second, first_harmonic.first);
	    
        

    }
    return 0;
}
#ifndef CURAN_COMPENSATE_RIPPLE_
#define CURAN_COMPENSATE_RIPPLE_

#include "LBRController.h"
#include "FilterRippleFirstHarmonic.h"

namespace curan {
namespace robotic {
    
enum type_of_compensation{
    ADAPTIVE_FILTERING_SCHEME,
    CLASSIC_APPROACH,
    NO_FILTERING_SCHEME
};

struct CompensateRipple : public UserData{
    CompensateRipple();

    std::atomic<type_of_compensation> activate = NO_FILTERING_SCHEME;
    
    std::array<FilterData,number_of_joints> joint_data_first_harmonic;
    std::array<FilterData,number_of_joints> joint_data_second_harmonic;
    std::array<FilterProperties,number_of_joints> first_harmonic;
    std::array<FilterProperties,number_of_joints> second_harmonic;
    EigenState&& update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
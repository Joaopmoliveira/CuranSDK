#ifndef CURAN_COMPENSATE_RIPPLE_FREEHAND_
#define CURAN_COMPENSATE_RIPPLE_FREEHAND_

#include "LBRController.h"
#include "FilterRippleFirstHarmonic.h"

namespace curan {
namespace robotic {

enum type_of_compensation{
    ADAPTIVE_FILTERING_SCHEME,
    CLASSIC_APPROACH,
    NO_FILTERING_SCHEME
};

struct CompensateRippleFreehand : public UserData{
    CompensateRippleFreehand();

    std::atomic<bool> activate = false;
    
    std::array<FilterData,number_of_joints> joint_data_first_harmonic;
    std::array<FilterData,number_of_joints> joint_data_second_harmonic;
    std::array<FilterProperties,number_of_joints> first_harmonic;
    std::array<FilterProperties,number_of_joints> second_harmonic;
    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
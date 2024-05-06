#ifndef CURAN_COMPENSATE_RIPPLE_
#define CURAN_COMPENSATE_RIPPLE_

#include "LBRController.h"
#include "FilterRippleFirstHarmonic.h"

namespace curan {
namespace robotic {

using joint_filters = std::array<std::pair<FilterData,FilterProperties>,number_of_joints>;

enum type_of_compensation{
    ADAPTIVE_FILTERING_SCHEME,
    CLASSIC_APPROACH,
    NO_FILTERING_SCHEME
};

struct CompensateRipple : public UserData{
    CompensateRipple();

    std::atomic<type_of_compensation> activate = NO_FILTERING_SCHEME;

    std::array<joint_filters,number_of_joints> first_harmonic;
    std::array<joint_filters,number_of_joints> second_harmonic;
    std::array<joint_filters,number_of_joints> third_harmonic;
    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
#ifndef CURAN_EXTRACT_RIPPLE_
#define CURAN_EXTRACT_RIPPLE_

#include "LBRController.h"
#include "FilterRippleFirstHarmonic.h"

namespace curan {
namespace robotic {

using joint_filters = std::array<std::pair<FilterData,FilterProperties>,number_of_joints>;

struct ExtractRipple : public UserData{
    ExtractRipple();

    std::array<joint_filters,number_of_joints> each_joint_filter;
    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
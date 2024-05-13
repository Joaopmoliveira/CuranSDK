#ifndef CURAN_EXTRACT_RIPPLE_CROSS_MITIGATION
#define CURAN_EXTRACT_RIPPLE_CROSS_MITIGATION

#include "LBRController.h"
#include "FilterRippleFirstHarmonic.h"

namespace curan {
namespace robotic {

using joint_filters = std::array<std::pair<FilterData,FilterProperties>,number_of_joints>;

struct ExtractRippleCrossMitigation : public UserData{
    ExtractRippleCrossMitigation();

    std::array<joint_filters,number_of_joints> first_harmonic;
    std::array<joint_filters,number_of_joints> second_harmonic;
    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
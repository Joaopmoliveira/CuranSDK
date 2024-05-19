#ifndef CURAN_EXTRACT_SINGLE_JOINT_RIPPLE_
#define CURAN_EXTRACT_SINGLE_JOINT_RIPPLE_

#include "LBRController.h"
#include "FilterRippleFirstHarmonic.h"

namespace curan {
namespace robotic {

struct ExtractSingleRipple : public UserData{
    ExtractSingleRipple();

    std::pair<FilterData,FilterProperties> first_harmonic;
    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
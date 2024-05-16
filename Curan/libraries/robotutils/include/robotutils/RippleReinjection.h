#ifndef CURAN_RIPPLE_REINJECTION_
#define CURAN_RIPPLE_REINJECTION_

#include "LBRController.h"
#include "FilterRippleFirstHarmonic.h"

namespace curan {
namespace robotic {

struct RippleReinjection : public UserData{
    RippleReinjection();
    std::atomic<bool> compensation_active = true;
    std::array<FilterData,number_of_joints> joint_data_first_harmonic;
    std::array<FilterData,number_of_joints> joint_data_second_harmonic;
    std::array<FilterProperties,number_of_joints> first_harmonic;
    std::array<FilterProperties,number_of_joints> second_harmonic;
    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
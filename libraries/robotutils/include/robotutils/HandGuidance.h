#ifndef CURAN_HAND_GUIDANCE_
#define CURAN_HAND_GUIDANCE_

#include "LBRController.h"

namespace curan {
namespace robotic {

struct HandGuidance : public UserData{
    HandGuidance();
    EigenState&& update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
#ifndef CURAN_JOINT_VELOCITY_CONTROLLER_
#define CURAN_JOINT_VELOCITY_CONTROLLER_

#include "LBRController.h"

namespace curan {
namespace robotic {

struct JointVelocityController : public UserData{
    JointVelocityController();
    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
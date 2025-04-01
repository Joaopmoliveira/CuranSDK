#ifndef CURAN_JOINT_IMPEDANCE_CONTROLLER_
#define CURAN_JOINT_IMPEDANCE_CONTROLLER_

#include "LBRController.h"

namespace curan {
namespace robotic {

struct JointImpedanceController : public UserData{
    JointImpedanceController();
    EigenState&& update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
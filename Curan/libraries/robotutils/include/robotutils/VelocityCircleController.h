#ifndef CURAN_PASSIVE_CONTROLLER_AUDE_
#define CURAN_PASSIVE_CONTROLLER_AUDE_

#include "LBRController.h"
#include "gaussianmixtures/GMR.h"

namespace curan {
namespace robotic {

struct VelocityCircleController : public UserData{
    PassiveControllerData();

    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;

    Eigen::Vector3d circleCenter;
    Eigen::Matrix3d desRotation;
};

}
}

#endif
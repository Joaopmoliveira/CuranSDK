#ifndef CURAN_NEEDLE_CONTROLLER_AUDE_
#define CURAN_NEEDLE_CONTROLLER_AUDE_        

#include "LBRController.h"

namespace curan {
namespace robotic {

struct NeedleController : public UserData{
    NeedleController();

    EigenState&& update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;

    Eigen::Matrix3d desRotation;
    Eigen::Matrix<double,3,1> direction;
    Eigen::Matrix<double,3,1> entry_point;
};

}
}

#endif
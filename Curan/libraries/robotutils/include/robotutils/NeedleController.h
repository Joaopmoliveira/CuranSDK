#ifndef CURAN_NEEDLE_CONTROLLER_AUDE_
#define CURAN_NEEDLE_CONTROLLER_AUDE_        

#include "LBRController.h"

namespace curan {
namespace robotic {

struct NeedleController : public UserData{
    NeedleController();

    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state) override;

    Eigen::Matrix3d desRotation;
    Eigen::Matrix<double,3,1> direction;
    Eigen::Matrix<double,3,1> entry_point;
};

}
}

#endif
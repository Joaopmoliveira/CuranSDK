#ifndef CURAN_SINGULARITY_CONTROLLER_AUDE_
#define CURAN_SINGULARITY_CONTROLLER_AUDE_  

#include "LBRController.h"

namespace curan {
namespace robotic {

struct SingularityAvoidanceData : public UserData{
    SingularityAvoidanceData();

    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;
};

}
}

#endif
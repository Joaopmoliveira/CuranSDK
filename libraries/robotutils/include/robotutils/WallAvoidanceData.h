#ifndef CURAN_SINGULARITY_CONTROLLER_AUDE_
#define CURAN_SINGULARITY_CONTROLLER_AUDE_  

#include "LBRController.h"

namespace curan {
namespace robotic {

struct WallAvoidanceData : public UserData{
    WallAvoidanceData(Eigen::Vector3d plane_point, Eigen::Vector3d direction_along_valid_region,double max_accel = 0.5, double max_vel = 0.25);

    EigenState&& update(const RobotModel<number_of_joints>& iiwa,EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;

    Eigen::Vector3d f_plane_point;
    Eigen::Vector3d f_direction_along_valid_region;
    double max_accel;
    double max_vel;
};

}
}

#endif
#ifndef CURAN_JOINT_VELOCITY_CONTROLLER_
#define CURAN_JOINT_VELOCITY_CONTROLLER_

#include "LBRController.h"

namespace curan {
namespace robotic {

struct Transformation{
    Eigen::Matrix<double,3,3> f_rotation;
    Eigen::Matrix<double,3,1> f_translation;

    Transformation(Eigen::Matrix<double,3,3> rotation,Eigen::Matrix<double,3,1> translation) : f_rotation{rotation},f_translation{translation}{

    }

    Transformation(const Transformation& other) : f_rotation{other.f_rotation},f_translation{other.f_translation}{

    }

    inline auto desired_rotation() const {
        return f_rotation;
    }

    inline auto desired_translation() const {
        return f_translation;
    }
};

struct CartersianVelocityController : public UserData{
    Transformation f_equilibrium; 
    CartersianVelocityController(const Transformation& equilibrium,double asyntotic_reduction_in_orientation_speed, double asyntotic_reduction_in_space_speed);
    EigenState&& update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;

    double f_asyntotic_reduction_in_orientation_speed;
    double f_asyntotic_reduction_in_space_speed;
    Eigen::Matrix<double, 6, 6> velocity_scalling;
};

}
}

#endif
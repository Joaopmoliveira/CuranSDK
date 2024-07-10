#ifndef CURAN_JOINT_VELOCITY_CONTROLLER_
#define CURAN_JOINT_VELOCITY_CONTROLLER_

#include "LBRController.h"
#include <variant>

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

using Generator = std::function<Eigen::Matrix<double,6,1>(const RobotModel<number_of_joints>&)>;

struct CartersianVelocityController : public UserData{
    std::variant<Transformation,Generator> ref_trajectory; 
    CartersianVelocityController(const Transformation& equilibrium,std::initializer_list<double> stiffness_diagonal_gains,std::initializer_list<double> in_diagonal_damping);
    CartersianVelocityController(Generator&& generator,std::initializer_list<double> stiffness_diagonal_gains,std::initializer_list<double> in_diagonal_damping);
    EigenState&& update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;

    Eigen::Matrix<double, 6, 6> stiffness;
    Eigen::Matrix<double, 6, 6> diagonal_damping;
};

}
}

#endif
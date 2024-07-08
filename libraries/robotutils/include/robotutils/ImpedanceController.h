#ifndef CURAN_IMPEDANCE_CONTROLLER_
#define CURAN_IMPEDANCE_CONTROLLER_

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

struct ImpedanceController : public UserData{
    Transformation f_equilibrium; 
    Eigen::Matrix<double,6,6> stiffness;
    Eigen::Matrix<double,6,6> damping;
    Eigen::Matrix<double,6,6> diagonal_damping;
    /*
    The stiffness values are such that {K_x,K_y,K_z,K_ang_1,K_ang_2,K_ang3}
    is translated into 
    K = [K_x  0  0  0  0  0]
        [0  K_y  0  0  0  0]
        [0  0  K_z  0  0  0]
        [0  0  0  K_ang_1  0  0]
        [0  0  0  0  K_ang_2  0]
        [0  0  0  0  0  K_ang3 ]

    The damping is automatically addapted depending on the mass matrix of the robot
    */
    ImpedanceController(Transformation equilibrium,std::initializer_list<double> stiffness_diagonal_gains,std::initializer_list<double> in_diagonal_damping) : f_equilibrium{equilibrium}{
        if(stiffness_diagonal_gains.size()!=6)
            throw std::runtime_error("the dimensions of the diagonal matrix must be 6x6");
        if(in_diagonal_damping.size()!=6)
            throw std::runtime_error("the dimensions of the diagonal matrix must be 6x6");
        stiffness = Eigen::Matrix<double,6,6>::Identity();
        auto stiffness_entry_value = stiffness_diagonal_gains.begin();
        auto damping_entry_value = in_diagonal_damping.begin();
        for(size_t entry = 0; entry< 6; ++entry,++stiffness_entry_value,++damping_entry_value){
            if(*damping_entry_value>=0.0)
                throw std::runtime_error("the damping matrix must be positive definite");
            if(*stiffness_entry_value>=0.0)
                throw std::runtime_error("the stiffness matrix must be positive definite");
            diagonal_damping(entry,entry) = *damping_entry_value;
            stiffness(entry,entry) = *stiffness_entry_value;
        }
    };

    ~ImpedanceController(){
    }

    EigenState&& update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override;

};

}
}

#endif
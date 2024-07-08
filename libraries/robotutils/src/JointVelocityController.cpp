#include "robotutils/JointVelocityController.h"

namespace curan {
namespace robotic {

    JointVelocityController::JointVelocityController(){
    }

    EigenState&& JointVelocityController::update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
        static double timer = 0.0;
        
        //double actuation = (std::sin(5*timer)>0.0) ? 0.2 : -0.2;
        double actuation = 0.2*std::sin(5*timer);


        static Eigen::Matrix<double,7,1> filtered_velocity = iiwa.velocities();
        auto val = 0.8187*filtered_velocity + 0.1813*iiwa.velocities();
        filtered_velocity = val;

        Eigen::Matrix<double,7,1> desired_velocity = Eigen::Matrix<double,7,1>::Ones()*actuation;
        
        //state.cmd_tau =  35*(desired_velocity-filtered_velocity);
        state.cmd_tau =  35*iiwa.mass()*(desired_velocity-filtered_velocity);
        state.user_defined = filtered_velocity;
        timer += iiwa.sample_time();


        /*
        The Java controller has two values which it reads, namely: 
        1) commanded_joint_position 
        2) commanded_torque 
        The torque is computed in the previous line, but the position can remain empty. One problem with this approach is that if the deviation
        between the reference position and the commanded position is larger than 5 degrees the robot triggers a safety stop 1. To avoid this
        the first solution is to set the commanded position to be equal to the current position. This approach has a drawback where the error between
        both commanded and current position is always zero, which results in the friction compensator being "shut off". We avoid this problem
        by adding a small perturbation to the reference position with a relative high frequency. 
        */
        state.cmd_q = iiwa.joints() + Eigen::Matrix<double,number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));

        currentTime += iiwa.sample_time();
        return std::move(state);
    }

}
}
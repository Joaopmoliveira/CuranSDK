#include "robotutils/HandGuidance.h"

namespace curan {
namespace robotic {

    HandGuidance::HandGuidance(){
    }

    EigenState&& HandGuidance::update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state){
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
       
        state.cmd_tau = -iiwa->M * 10 * iiwa->qDot;


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
        state.cmd_q = state.q + Eigen::Matrix<double,number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));

        currentTime += state.sampleTime;
        return std::move(state);
    }

}
}
#include "robotutils/ExtractSingleJoint.h"

namespace curan {
namespace robotic {

    ExtractSingleRipple::ExtractSingleRipple(){
        first_harmonic.second.frequency = 320.0;
    }

    EigenState&& ExtractSingleRipple::update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
       
        //state.cmd_tau = -iiwa->M * 10 * iiwa->qDot;

        auto printer = [&](std::string s){
            std::printf("%s",s.data());
            std::printf("\n");
            for(size_t i = 0; i < 3; ++i){
                std::printf("%.2f ",first_harmonic.first.y[i]);
            }
            std::printf("\n");
            for(size_t i = 0; i < 3; ++i){
                std::printf("%.2f ",first_harmonic.first.yf[i]);
            }
            std::printf("\n");
        };

        static State prev_state = *iiwa.measured_state();
        static State first_state = *iiwa.measured_state();

        state.cmd_tau = Eigen::Matrix<double,7,1>::Zero();
        //state.user_defined[1] = state.dq[4];
        double current_torque_joint_i = iiwa.measured_state()->tau[4];
        const Observation obser_i_j{iiwa.velocities()[4],iiwa.joints()[4] - prev_state.q[4]};
        update_filter_properties(first_harmonic.second, obser_i_j);
        shift_filter_data(first_harmonic.first,0.0);  
	    update_filter_data(first_harmonic.first, current_torque_joint_i);
	    double filtered_torque_joint_i = filter_implementation(first_harmonic.second, first_harmonic.first);                   
        state.user_defined[4] = current_torque_joint_i-filtered_torque_joint_i;
        prev_state = *iiwa.measured_state();

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
        auto perturbations = iiwa.joints() + Eigen::Matrix<double,number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
        state.cmd_q = convert<double,number_of_joints>(first_state.q);
        state.cmd_q[6] = perturbations[6];
        state.cmd_q[4] = perturbations[4];

        currentTime += iiwa.sample_time();
        return std::move(state);
    }

}
}
#include "robotutils/ExtractRippleCrossMitigation.h"

namespace curan {
namespace robotic {

    ExtractRippleCrossMitigation::ExtractRippleCrossMitigation(){
        for(size_t filter_index = 0 ; filter_index < number_of_joints; ++filter_index){
            first_harmonic[filter_index].frequency = (filter_index == 4) ? 320.0 : 320.0;
            second_harmonic[filter_index].frequency = (filter_index == 4) ? 640.0 : 640.0;
        }
    }

    EigenState&& ExtractRippleCrossMitigation::update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
       
        static EigenState prev_state = state;
        static EigenState first_state = state;

        state.cmd_tau = Eigen::Matrix<double,7,1>::Zero();

        double largest_frequency_found = std::numeric_limits<double>::min();
        size_t offset_fastest_filter = 0;

        for(size_t joint_i = 0; joint_i < number_of_joints; ++joint_i){
            const Observation obser_i_j{state.dq[joint_i],state.q[joint_i] - prev_state.q[joint_i]};
            update_filter_properties(first_harmonic[joint_i], obser_i_j);
            update_filter_properties(second_harmonic[joint_i], obser_i_j);
            if(largest_frequency_found < first_harmonic[joint_i].log_filtered_frequency){
                largest_frequency_found =  first_harmonic[joint_i].log_filtered_frequency;
                offset_fastest_filter = joint_i;
            }
        }
        
        for(size_t torque_joint_i = 0; torque_joint_i < number_of_joints; ++torque_joint_i){
            double filtered_torque_value = state.tau[torque_joint_i];
            shift_filter_data(joint_data_first_harmonic[torque_joint_i],0.0);
            update_filter_data(joint_data_first_harmonic[torque_joint_i], filtered_torque_value);
            double filtered_torque_first_harmonic = filter_implementation(first_harmonic[offset_fastest_filter], joint_data_first_harmonic[torque_joint_i]);
            filtered_torque_value -= filtered_torque_first_harmonic;
            shift_filter_data(joint_data_second_harmonic[torque_joint_i],0.0);
            update_filter_data(joint_data_second_harmonic[torque_joint_i], filtered_torque_value);
            double filtered_torque_second_harmonic = filter_implementation(first_harmonic[offset_fastest_filter], joint_data_second_harmonic[torque_joint_i]);
            filtered_torque_value -= filtered_torque_second_harmonic;
            state.user_defined[torque_joint_i] = filtered_torque_value;
        }
        
        prev_state = state;

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
        auto perturbations = state.q + Eigen::Matrix<double,number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
        state.cmd_q = first_state.q;
        state.cmd_q[6] = perturbations[6];
        state.cmd_q[4] = perturbations[4];

        currentTime += state.sampleTime;
        return std::move(state);
    }

}
}
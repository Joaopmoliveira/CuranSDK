#include "robotutils/ExtractRippleCrossMitigation.h"

namespace curan {
namespace robotic {

    ExtractRippleCrossMitigation::ExtractRippleCrossMitigation(){
        for(size_t i = 0 ; i < number_of_joints; ++i)
            for(size_t filter_index = 0 ; filter_index < number_of_joints; ++filter_index)
                first_harmonic[i][filter_index].second.frequency = (filter_index == 4) ? 200.0 : 320.0;
        for(size_t i = 0 ; i < number_of_joints; ++i)
            for(size_t filter_index = 0 ; filter_index < number_of_joints; ++filter_index)
                second_harmonic[i][filter_index].second.frequency = (filter_index == 4) ? 400.0 : 640.0;
    }

    EigenState&& ExtractRippleCrossMitigation::update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
       
        //state.cmd_tau = -iiwa->M * 10 * iiwa->qDot;

        static EigenState prev_state = state;
        static EigenState first_state = state;

        state.cmd_tau = Eigen::Matrix<double,7,1>::Zero();
        
        for(size_t torque_joint_i = 0; torque_joint_i < number_of_joints; ++torque_joint_i){
            // this array is always initialized to zero, and its filled with the 
            // frequencies of the previous filter as they are applied, we then pass an iterator
            // to this array 
            std::array<double,number_of_joints*2> previous_filtered_joint_frequencies;
            std::array<double,number_of_joints*2> joints_to_see_later;
            auto post_recording_stuff = joints_to_see_later.begin();
            auto processed_frequencies = previous_filtered_joint_frequencies.begin();
            double filtered_joint_torque{0.0};
            for(size_t cross_phenomena_j = 0; cross_phenomena_j < number_of_joints;++cross_phenomena_j){
                double filtered_torque_joint_i = run_filter(first_harmonic[torque_joint_i][cross_phenomena_j].first, 
                                                            first_harmonic[torque_joint_i][cross_phenomena_j].second, 
                                                            { state.dq[cross_phenomena_j],state.q[cross_phenomena_j] - prev_state.q[cross_phenomena_j],state.tau[torque_joint_i]},
                                                            previous_filtered_joint_frequencies.begin(),processed_frequencies);
                state.user_defined[cross_phenomena_j] = filtered_torque_joint_i;
                
                *post_recording_stuff = first_harmonic[torque_joint_i][cross_phenomena_j].second.crosstalk_damper*first_harmonic[torque_joint_i][cross_phenomena_j].second.damper;
                filtered_joint_torque += filtered_torque_joint_i;
                *processed_frequencies = first_harmonic[torque_joint_i][cross_phenomena_j].second.filtered_frequency;
                state.user_defined3[cross_phenomena_j] = *processed_frequencies;
                ++processed_frequencies;
                ++post_recording_stuff;
            }

            for(size_t cross_phenomena_j = 0; cross_phenomena_j < number_of_joints;++cross_phenomena_j){
                double filtered_torque_joint_i = run_filter(second_harmonic[torque_joint_i][cross_phenomena_j].first, 
                                                            second_harmonic[torque_joint_i][cross_phenomena_j].second, 
                                                            { state.dq[cross_phenomena_j],state.q[cross_phenomena_j] - prev_state.q[cross_phenomena_j],state.tau[torque_joint_i]},
                                                            previous_filtered_joint_frequencies.begin(),processed_frequencies);
                state.user_defined2[cross_phenomena_j] = filtered_torque_joint_i;
                *post_recording_stuff = second_harmonic[torque_joint_i][cross_phenomena_j].second.crosstalk_damper*second_harmonic[torque_joint_i][cross_phenomena_j].second.damper;
                filtered_joint_torque += filtered_torque_joint_i;
                *processed_frequencies = second_harmonic[torque_joint_i][cross_phenomena_j].second.filtered_frequency;
                state.user_defined4[cross_phenomena_j] = *processed_frequencies;
                ++processed_frequencies;
                ++post_recording_stuff;
            }

            //state.user_defined[torque_joint_i] = state.tau[torque_joint_i]-filtered_joint_torque;
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
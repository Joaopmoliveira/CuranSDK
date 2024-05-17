#include "robotutils/ExtractRipple.h"

namespace curan {
namespace robotic {

    ExtractRipple::ExtractRipple(){
        for(size_t i = 0 ; i < number_of_joints; ++i)
            for(size_t filter_index = 0 ; filter_index < number_of_joints; ++filter_index)
                first_harmonic[i][filter_index].second.frequency = 320.0;
        for(size_t i = 0 ; i < number_of_joints; ++i)
            for(size_t filter_index = 0 ; filter_index < number_of_joints; ++filter_index)
                second_harmonic[i][filter_index].second.frequency = 640.0;
        for(size_t i = 0 ; i < number_of_joints; ++i)
            for(size_t filter_index = 0 ; filter_index < number_of_joints; ++filter_index)
                third_harmonic[i][filter_index].second.frequency = 1280.0;
    }

    EigenState&& ExtractRipple::update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */

        static EigenState prev_state = state;
        static EigenState first_state = state;

        state.cmd_tau = Eigen::Matrix<double,7,1>::Zero();
        
        for(size_t torque_joint_i = 0; torque_joint_i < number_of_joints; ++torque_joint_i){
            double initial_torque_joint_i = state.tau[torque_joint_i];
            double current_torque_joint_i = initial_torque_joint_i;
            for(size_t cross_phenomena_j = 0; cross_phenomena_j < number_of_joints;++cross_phenomena_j){
                const Observation obser_i_j{state.dq[cross_phenomena_j],state.q[cross_phenomena_j] - prev_state.q[cross_phenomena_j]};
                update_filter_properties(first_harmonic[torque_joint_i][cross_phenomena_j].second, obser_i_j);
	            shift_filter_data(first_harmonic[torque_joint_i][cross_phenomena_j].first,0.0);   
                update_filter_data(first_harmonic[torque_joint_i][cross_phenomena_j].first, current_torque_joint_i);
	            double filtered_torque_joint_i = filter_implementation(first_harmonic[torque_joint_i][cross_phenomena_j].second, first_harmonic[torque_joint_i][cross_phenomena_j].first);           
                current_torque_joint_i -= filtered_torque_joint_i;
            }
            for(size_t cross_phenomena_j = 0; cross_phenomena_j < number_of_joints;++cross_phenomena_j){
                const Observation obser_i_j{state.dq[cross_phenomena_j],state.q[cross_phenomena_j] - prev_state.q[cross_phenomena_j]};
                update_filter_properties(second_harmonic[torque_joint_i][cross_phenomena_j].second, obser_i_j);
	            shift_filter_data(second_harmonic[torque_joint_i][cross_phenomena_j].first,0.0);
                update_filter_data(second_harmonic[torque_joint_i][cross_phenomena_j].first, current_torque_joint_i);
	            double filtered_torque_joint_i = filter_implementation(second_harmonic[torque_joint_i][cross_phenomena_j].second, second_harmonic[torque_joint_i][cross_phenomena_j].first);                
                current_torque_joint_i -= filtered_torque_joint_i;
            }
            state.user_defined[torque_joint_i] = current_torque_joint_i;
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
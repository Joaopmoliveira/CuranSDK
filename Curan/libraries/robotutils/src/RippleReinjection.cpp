#include "robotutils/RippleReinjection.h"

namespace curan {
namespace robotic {

    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    RippleReinjection::RippleReinjection(){
        for(size_t filter_index = 0 ; filter_index < number_of_joints; ++filter_index){
            first_harmonic[filter_index].frequency = 320.0;
            second_harmonic[filter_index].frequency = 640.0;
        }
    }

    EigenState&& RippleReinjection::update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
       
        //state.cmd_tau = -iiwa->M * 10 * iiwa->qDot;

        
        static EigenState first_state = state;
        static Eigen::Matrix<double,7,1> reference = state.q;

        state.cmd_tau = Eigen::Matrix<double,7,1>::Zero();
        static EigenState prev_state = state;
        
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
            state.user_defined3[torque_joint_i] = state.tau[torque_joint_i]-filtered_torque_value;
            state.user_defined4[torque_joint_i] = first_harmonic[offset_fastest_filter].damper;
        }
 
        static EigenState prev_state_for_torque_derivative = state;

        constexpr double filter_weight = 0.02;
        static_assert(filter_weight > 0.0 && filter_weight < 1.0);
        
        Eigen::Matrix<double,7,1> derivative_torque = (state.user_defined3-prev_state_for_torque_derivative.user_defined3)*(1/state.sampleTime);
        static Eigen::Matrix<double,7,1> filtered_derivative_torque = derivative_torque;
        filtered_derivative_torque = ((1-filter_weight)*filtered_derivative_torque + filter_weight*derivative_torque).eval();
    
        constexpr double proportional_gain[2] = {-0.7 , -0.7};
        constexpr double derivative_gain[2] = {-0.001, -0.001};

        state.user_defined2[6] = proportional_gain[0]*(state.user_defined3[6])+derivative_gain[0]*derivative_torque[6];
        state.user_defined2[4] = proportional_gain[1]*(state.user_defined3[4])+derivative_gain[1]*derivative_torque[4];

        const Eigen::Matrix<double,7,1> desired_torque = Eigen::Matrix<double,7,1>::Zero();

        if(compensation_active.load()){
            state.cmd_tau[6] = state.user_defined2[6];
            state.cmd_tau[4] = state.user_defined2[4];
        } else {
            state.cmd_tau[6] = 0;
            state.cmd_tau[4] = 0;     
        }

        prev_state = state;
        prev_state_for_torque_derivative = state;

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
        auto perturbations = state.q + Eigen::Matrix<double,number_of_joints,1>::Constant(0.1 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
        state.cmd_q = first_state.q;
        state.cmd_q[4] = perturbations[4];
        state.cmd_q[6] = perturbations[6];

        currentTime += state.sampleTime;
        return std::move(state);
    }

}
}
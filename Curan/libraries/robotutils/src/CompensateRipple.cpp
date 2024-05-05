#include "robotutils/CompensateRipple.h"

namespace curan {
namespace robotic {

    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    CompensateRipple::CompensateRipple(){
        for(size_t i = 0 ; i < number_of_joints; ++i)
            for(size_t filter_index = 0 ; filter_index < number_of_joints; ++filter_index)
                first_harmonic[i][filter_index].second.frequency = (filter_index == 4) ? 200.0 : 320.0;
        for(size_t i = 0 ; i < number_of_joints; ++i)
            for(size_t filter_index = 0 ; filter_index < number_of_joints; ++filter_index)
                second_harmonic[i][filter_index].second.frequency = (filter_index == 4) ? 400.0 : 640.0;
        for(size_t i = 0 ; i < number_of_joints; ++i)
            for(size_t filter_index = 0 ; filter_index < number_of_joints; ++filter_index)
                third_harmonic[i][filter_index].second.frequency = (filter_index == 4) ? 800.0 : 1280.0;
    }

    EigenState&& CompensateRipple::update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
       
        //state.cmd_tau = -iiwa->M * 10 * iiwa->qDot;

        
        static EigenState first_state = state;

        state.cmd_tau = Eigen::Matrix<double,7,1>::Zero();
        static EigenState prev_state = state;
        
        for(size_t torque_joint_i = 0; torque_joint_i < number_of_joints; ++torque_joint_i){
            double initial_torque_joint_i = state.tau[torque_joint_i]; //+state.gravity[torque_joint_i];
            double current_torque_joint_i = initial_torque_joint_i;
            for(size_t cross_phenomena_j = 0; cross_phenomena_j < number_of_joints;++cross_phenomena_j){
                double filtered_torque_joint_i = run_filter(first_harmonic[torque_joint_i][cross_phenomena_j].first, first_harmonic[torque_joint_i][cross_phenomena_j].second, { state.dq[cross_phenomena_j],state.q[cross_phenomena_j] - prev_state.q[cross_phenomena_j],current_torque_joint_i});
                current_torque_joint_i -= filtered_torque_joint_i;
            }
            for(size_t cross_phenomena_j = 0; cross_phenomena_j < number_of_joints;++cross_phenomena_j){
                double filtered_torque_joint_i = run_filter(second_harmonic[torque_joint_i][cross_phenomena_j].first, second_harmonic[torque_joint_i][cross_phenomena_j].second, { state.dq[cross_phenomena_j],state.q[cross_phenomena_j] - prev_state.q[cross_phenomena_j],current_torque_joint_i});
                current_torque_joint_i -= filtered_torque_joint_i;
            }
            state.user_defined[torque_joint_i] = current_torque_joint_i;
        }

        static size_t counter = 0;

        static Eigen::Matrix<double,7,1> filtered_velocity = first_state.dq;
        filtered_velocity = ((0.8)*filtered_velocity+(0.2)*state.dq).eval();
 
        static EigenState prev_state_for_torque_derivative = state;

        constexpr double filter_weight = 0.3;
        static_assert(filter_weight > 0.0 && filter_weight < 1.0);
        
        Eigen::Matrix<double,7,1> derivative_torque = (state.user_defined-prev_state_for_torque_derivative.user_defined)*(1/state.sampleTime);
        static Eigen::Matrix<double,7,1> filtered_derivative_torque = derivative_torque;
        filtered_derivative_torque = ((1-filter_weight)*filtered_derivative_torque + filter_weight*derivative_torque).eval();
        
        Eigen::Matrix<double,7,1> derivative_torque_unclean = (state.tau-prev_state_for_torque_derivative.tau)*(1/state.sampleTime);
        static Eigen::Matrix<double,7,1> filtered_derivative_torque_unclean = derivative_torque_unclean;
        filtered_derivative_torque_unclean = ((1-filter_weight)*filtered_derivative_torque_unclean + filter_weight*derivative_torque_unclean).eval();
        
        constexpr double impedance_proportional_gain = 100;
        constexpr double impedance_derivative_gain = 7;

        constexpr double reduction_ratio = 4.5;
        constexpr double proportional_gain = 1-reduction_ratio;
        //constexpr double derivative_gain = -0.001*proportional_gain;
        constexpr double derivative_gain = -0.001*proportional_gain;

        static Eigen::Matrix<double,7,1> proportional_error = Eigen::Matrix<double,7,1>::Zero();
        static Eigen::Matrix<double,7,1> derivative_error = Eigen::Matrix<double,7,1>::Zero();
        
        switch(activate.load(std::memory_order_acq_rel)){
            case ADAPTIVE_FILTERING_SCHEME:
            {
                if(counter % 5 == 0 ){
                    if(static_cast<int>(std::round(currentTime)) % 6 >= 3){
                        proportional_error = (-0.25*Eigen::Matrix<double,7,1>::Ones())-state.q;
                        derivative_error = -filtered_velocity;
                    } else {
                        proportional_error = 0.25*Eigen::Matrix<double,7,1>::Ones()-state.q;
                        derivative_error = -filtered_velocity;
                    }
                }
                const Eigen::Matrix<double,7,1> desired_torque = -state.gravity-impedance_proportional_gain*proportional_error-impedance_derivative_gain*derivative_error;
                state.user_defined2[6] = desired_torque[6]+proportional_gain*(desired_torque[6]-state.user_defined[6])-derivative_gain*filtered_derivative_torque[6];
                state.user_defined2[4] = desired_torque[4]+proportional_gain*(desired_torque[4]-state.user_defined[4])-derivative_gain*filtered_derivative_torque[4];
                state.cmd_tau[6] = state.user_defined2[6];
                state.cmd_tau[4] = state.user_defined2[4];
                break;
            }
            case CLASSIC_APPROACH:
            {
                if(counter % 5 == 0 ){
                    if(static_cast<int>(std::round(currentTime)) % 6 >= 3){
                        proportional_error = (-0.25*Eigen::Matrix<double,7,1>::Ones())-state.q;
                        derivative_error = -filtered_velocity;
                    } else {
                        proportional_error = 0.25*Eigen::Matrix<double,7,1>::Ones()-state.q;
                        derivative_error = -filtered_velocity;
                    }
                }
                const Eigen::Matrix<double,7,1> desired_torque = -state.gravity-impedance_proportional_gain*proportional_error-impedance_derivative_gain*derivative_error;
                state.user_defined2[6] = desired_torque[6]+proportional_gain*(desired_torque[6]-state.tau[6])-derivative_gain*filtered_derivative_torque_unclean[6];
                state.user_defined2[4] = desired_torque[4]+proportional_gain*(desired_torque[4]-state.tau[4])-derivative_gain*filtered_derivative_torque_unclean[4];
                state.cmd_tau[6] = state.user_defined2[6];
                state.cmd_tau[4] = state.user_defined2[4];
                break;
            }
            case NO_FILTERING_SCHEME:
            default:
            {
                if(counter % 5 == 0 ){
                    if(static_cast<int>(std::round(currentTime)) % 6 >= 3){
                        proportional_error = (-0.25*Eigen::Matrix<double,7,1>::Ones())-state.q;
                        derivative_error = -filtered_velocity;
                    } else {
                        proportional_error = 0.25*Eigen::Matrix<double,7,1>::Ones()-state.q;
                        derivative_error = -filtered_velocity;
                    }
                }
                const Eigen::Matrix<double,7,1> desired_torque = impedance_proportional_gain*proportional_error+impedance_derivative_gain*derivative_error;
                state.user_defined2[6] = desired_torque[6];
                state.user_defined2[4] = desired_torque[4];
                state.cmd_tau[6] = state.user_defined2[6];
                state.cmd_tau[4] = state.user_defined2[4];
                break;
            }
        }

        ++counter;

        //state.user_defined2[6] = -state.gravity[6]+proportional_gain*(-state.gravity[6]-state.user_defined[6])-derivative_gain*filtered_derivative_torque[6];
        //state.user_defined2[4] = -state.gravity[4]+proportional_gain*(-state.gravity[4]-state.user_defined[4])-derivative_gain*filtered_derivative_torque[4];
 
        //state.user_defined2[6] = -state.gravity[6]+proportional_gain*(-state.gravity[6]-state.tau[6])-derivative_gain*filtered_derivative_torque_unclean[6];
        //state.user_defined2[4] = -state.gravity[4]+proportional_gain*(-state.gravity[4]-state.tau[4])-derivative_gain*filtered_derivative_torque_unclean[4];

 
        //state.user_defined2[6] = state.gravity[6];
        //state.user_defined2[4] = state.gravity[4];
        //state.cmd_tau[6] = state.user_defined2[6];
        //state.cmd_tau[4] = state.user_defined2[4];

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
        auto perturbations = state.q + Eigen::Matrix<double,number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
        state.cmd_q = first_state.q;
        state.cmd_q[4] = perturbations[4];
        state.cmd_q[6] = perturbations[6];

        currentTime += state.sampleTime;
        return std::move(state);
    }

}
}
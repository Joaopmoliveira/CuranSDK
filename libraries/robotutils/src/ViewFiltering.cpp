#include "robotutils/ViewFiltering.h"

namespace curan {
namespace robotic {

    ViewFiltering::ViewFiltering(){
        for (size_t filter_index = 0; filter_index < number_of_joints; ++filter_index)
        {
            first_harmonic[filter_index].frequency = (filter_index == 4) ? 320.0 : 320.0;
            second_harmonic[filter_index].frequency = (filter_index == 4) ? 640.0 : 640.0;
            third_harmonic[filter_index].frequency = (filter_index == 4) ? 1280.0 : 1280.0;
        }
    }

    EigenState&& ViewFiltering::update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
        using vector_type = curan::robotic::RobotModel<curan::robotic::number_of_joints>::vector_type;
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
       
        state.cmd_tau = -iiwa.mass() * iiwa.velocities();
        vector_type raw_filtered_torque = vector_type::Zero();
        vector_type partial_derivative_torque = vector_type::Zero();
        if(filtering_mechanism.is_first)
            previous_q = iiwa.joints();
        
        double fastest_velocity_found = std::numeric_limits<double>::min();
        size_t offset_fastest_filter = 0;

        for (size_t joint_i = 0; joint_i < curan::robotic::number_of_joints; ++joint_i)
        {
            const curan::robotic::ripple::Observation obser_i_j{iiwa.velocities()[joint_i], iiwa.joints()[joint_i] - previous_q[joint_i]};
            ripple::update_filter_properties(first_harmonic[joint_i], obser_i_j);
            ripple::update_filter_properties(second_harmonic[joint_i], obser_i_j);
            ripple::update_filter_properties(third_harmonic[joint_i], obser_i_j);
            double vel_modulus_i = std::abs(iiwa.velocities()[joint_i]);
            if (fastest_velocity_found < vel_modulus_i)
            {
                fastest_velocity_found = vel_modulus_i;
                offset_fastest_filter = joint_i;
            }
        }
        previous_q = iiwa.joints();

        for (size_t j = 0; j < curan::robotic::number_of_joints; ++j)
        {
            double filtered_torque_value = iiwa.measured_state()->tau[j];
            ripple::shift_filter_data(joint_data_first_harmonic[j], 0.0);
            ripple::update_filter_data(joint_data_first_harmonic[j], filtered_torque_value);
            auto [filtered_harm_1,partial_derivative_1] = ripple::execute(first_harmonic[offset_fastest_filter], joint_data_first_harmonic[j]);
            filtered_torque_value -= filtered_harm_1;
            ripple::shift_filter_data(joint_data_second_harmonic[j], 0.0);
            ripple::update_filter_data(joint_data_second_harmonic[j], filtered_torque_value);
            auto [filtered_harm_2,partial_derivative_2]= ripple::execute(second_harmonic[offset_fastest_filter], joint_data_second_harmonic[j]);
            filtered_torque_value -= filtered_harm_2;
            ripple::shift_filter_data(joint_data_third_harmonic[j], 0.0);
            ripple::update_filter_data(joint_data_third_harmonic[j], filtered_torque_value);
            auto [ filtered_harm_3,partial_derivative_3] = ripple::execute(third_harmonic[offset_fastest_filter], joint_data_third_harmonic[j]);
            partial_derivative_torque[j] = partial_derivative_1 + partial_derivative_2 +partial_derivative_3;
            filtered_torque_value -= filtered_harm_3;
            raw_filtered_torque[j] = -filtered_torque_value - iiwa.gravity()[j] - iiwa.coriolis()[j];
        }

        const auto& [filtered_torque, filtered_torque_derivative] = filtering_mechanism.update(raw_filtered_torque,partial_derivative_torque,iiwa.sample_time());
        state.cmd_q = iiwa.joints() + Eigen::Matrix<double,number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));

        currentTime += iiwa.sample_time();
        return std::move(state);
    }

}
}
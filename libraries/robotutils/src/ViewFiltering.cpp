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
        
        //We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        
        //state.cmd_tau = -iiwa.mass() * iiwa.velocities();
        state.cmd_tau = vector_type::Zero();
        vector_type raw_filtered_torque = vector_type::Zero();
        vector_type raw_deriv_filtered_torque = vector_type::Zero();

        if(filtering_mechanism.is_first)
            previous_q = iiwa.joints();

        static vector_type init_q = iiwa.joints();
        static vector_type prev_tau = iiwa.measured_torque();

        damper.compute<curan::robotic::number_of_joints>(iiwa.velocities(),iiwa.sample_time());
        previous_q = iiwa.joints();

        for (size_t j = 0; j < curan::robotic::number_of_joints; ++j)
        {
            double filtered_torque_value = iiwa.measured_torque()[j];
            double deriv_filtered_torque_value = (iiwa.measured_torque()[j]-prev_tau[j])/(iiwa.sample_time());
            curan::robotic::ripple::shift_filter_data(joint_data_first_harmonic[j]);
            curan::robotic::ripple::update_filter_data(joint_data_first_harmonic[j], filtered_torque_value,deriv_filtered_torque_value);
            auto [filtered_harm_1,partial_derivative_1] = curan::robotic::ripple::execute(first_harmonic[j],damper, joint_data_first_harmonic[j]);
            filtered_torque_value -= filtered_harm_1;
            deriv_filtered_torque_value -= partial_derivative_1;
            curan::robotic::ripple::shift_filter_data(joint_data_second_harmonic[j]);
            curan::robotic::ripple::update_filter_data(joint_data_second_harmonic[j], filtered_torque_value,deriv_filtered_torque_value);
            auto [filtered_harm_2,partial_derivative_2]= curan::robotic::ripple::execute(second_harmonic[j],damper, joint_data_second_harmonic[j]);
            filtered_torque_value -= filtered_harm_2;
            deriv_filtered_torque_value -= partial_derivative_2;
            curan::robotic::ripple::shift_filter_data(joint_data_third_harmonic[j]);
            curan::robotic::ripple::update_filter_data(joint_data_third_harmonic[j], filtered_torque_value,deriv_filtered_torque_value);
            auto [ filtered_harm_3,partial_derivative_3] = curan::robotic::ripple::execute(third_harmonic[j],damper, joint_data_third_harmonic[j]);
            filtered_torque_value -= filtered_harm_3;
            deriv_filtered_torque_value -= partial_derivative_3;
            raw_filtered_torque[j] = -filtered_torque_value - iiwa.gravity()[j] - iiwa.coriolis()[j];
            raw_deriv_filtered_torque[j] = -deriv_filtered_torque_value;
        }

        prev_tau = iiwa.measured_torque();

        //const auto& [filtered_torque, filtered_torque_derivative] = filtering_mechanism.update(raw_filtered_torque,partial_derivative_torque,iiwa.sample_time());
        const auto& [filtered_torque, filtered_torque_derivative] = filtering_mechanism.update(raw_filtered_torque,raw_deriv_filtered_torque,iiwa.sample_time());
        //state.cmd_q = iiwa.joints() + Eigen::Matrix<double,curan::robotic::number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
        state.cmd_q = init_q;
        state.cmd_q[0] = iiwa.joints()[0]+ (0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
        state.user_defined = raw_filtered_torque;
        state.user_defined2 = filtered_torque_derivative;
        state.user_defined4 = (iiwa.measured_torque()-prev_tau)/iiwa.sample_time();
        currentTime += iiwa.sample_time();
        return std::move(state);
    }

}
}
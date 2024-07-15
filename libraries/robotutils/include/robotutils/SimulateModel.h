#ifndef CURAN_ROBOT_SIMULATION_HEADER_
#define CURAN_ROBOT_SIMULATION_HEADER_

#include "RobotModel.h"

namespace curan
{
    namespace robotic
    {

        /*
        The templated function call advance to next state takes as template arguments the size of the
        robot being simulated and the duration of the sample time we are currently simulating the system.
        as function arguments it receives the model of the robot, the previous state, and the joint torques
        imposed on the robot.
        */
        template <size_t propagated_size, class Rep, class Period>
        State simulate_next_timestamp(RobotModel<propagated_size> &model, UserData* controller, const std::chrono::duration<Rep, Period>& sample_time, const State &previous_state, const typename RobotModel<propagated_size>::vector_type &external_torque)
        {
            State next = previous_state;
            next.sampleTime = std::chrono::duration<double>(sample_time).count();
            model.update(next);
            
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jacobian;
            auto actuation = controller->update(model, curan::robotic::EigenState{}, jacobian);
            next.convertFrom(actuation);

            typename RobotModel<propagated_size>::vector_type ddq = model.invmass() * (external_torque + actuation.cmd_tau);
            typename RobotModel<propagated_size>::vector_type dq = ddq * next.sampleTime + curan::robotic::convert(next.dq);
            typename RobotModel<propagated_size>::vector_type q = dq * next.sampleTime + curan::robotic::convert(next.q);

            next.q = curan::robotic::convert<double, propagated_size>(q);
            next.dq = curan::robotic::convert<double, propagated_size>(dq);
            next.ddq = curan::robotic::convert<double, propagated_size>(ddq);

            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            while (std::chrono::duration<double>(sample_time+begin-end).count() > 0.0)
                end = std::chrono::steady_clock::now();

            return next;
        }

    }
}

#endif
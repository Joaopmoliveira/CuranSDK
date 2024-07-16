#ifndef CURAN_ROBOT_SIMULATION_HEADER_
#define CURAN_ROBOT_SIMULATION_HEADER_

#include "RobotModel.h"
#include <cmath>

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

           auto forces_without_friction = external_torque + actuation.cmd_tau;
            
            /*
            Any proper simulation should have friction in the intermal model of the mechanical system. 
            We employ the model proposed in 
            "A globally stable state feedback controller for flexible joint robots"
            */
            typename RobotModel<propagated_size>::vector_type friction = RobotModel<propagated_size>::vector_type::Zero();
            constexpr double static_friction = 0.01 ;
            constexpr double dynamic_friction = 0.002 ;
            constexpr double viscous_friction = 0.01 ;
            for(size_t i = 0; i<propagated_size ; ++i ){
                if(std::abs(model.velocities()[i])<0.001){
                    friction[i] = (std::abs(forces_without_friction[i]) < static_friction ) ? 
                                                    forces_without_friction[i] : 
                                                    std::copysign(1.0,forces_without_friction[i])*dynamic_friction;
                } else {
                    friction[i] = std::copysign(1.0,model.velocities()[i])*dynamic_friction + viscous_friction*model.velocities()[i];
                }

            }

            typename RobotModel<propagated_size>::vector_type ddq = model.invmass() * ( forces_without_friction - friction );
            typename RobotModel<propagated_size>::vector_type dq = ddq * next.sampleTime + model.velocities();
            typename RobotModel<propagated_size>::vector_type q = dq * next.sampleTime + model.joints();

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
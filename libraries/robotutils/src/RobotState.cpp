#include "robotutils/RobotState.h"
#include "robotutils/ConvertEigenToArray.h"

namespace curan
{
    namespace robotic
    {

        WrappedState::WrappedState(const State &in)
        {
            f_q = Eigen::Matrix<double, number_of_joints, 1>::Zero();
            f_dq = Eigen::Matrix<double, number_of_joints, 1>::Zero();
            f_ddq = Eigen::Matrix<double, number_of_joints, 1>::Zero();
            f_end_effector = Eigen::Matrix<double, 4, 4>::Identity();
            f_measured_torque = Eigen::Matrix<double, number_of_joints, 1>::Zero();
            for (size_t row = 0; row < number_of_joints; ++row)
            {
                f_q[row] = in.q[row];
                f_dq[row] = in.dq[row];
                f_ddq[row] = in.ddq[row];
                f_measured_torque[row] = in.tau[row];
            }
            for (size_t cart_row = 0; cart_row < 3; ++cart_row)
            {
                f_end_effector(cart_row, 3) = in.translation[cart_row];
                for (size_t cart_col = 0; cart_col < 3; ++cart_col)
                    f_end_effector(cart_row, cart_col) = in.rotation[cart_row][cart_col];
            }
        }

        State::State(const KUKA::FRI::LBRState &state, const command_mode &mode)
        {
            sampleTime = state.getSampleTime();
            switch (mode)
            {
            case command_mode::COMMAND:
                for (size_t index = 0; index < number_of_joints; ++index)
                {
                    q[index] = state.getMeasuredJointPosition()[index];
                    cmd_q[index] = state.getCommandedJointPosition()[index];
                    cmd_tau[index] = state.getCommandedTorque()[index];
                    tau[index] = state.getMeasuredTorque()[index];
                    tau_ext[index] = state.getExternalTorque()[index];
                }
                break;
            case command_mode::MONITOR:
                for (size_t index = 0; index < number_of_joints; ++index)
                {
                    q[index] = state.getMeasuredJointPosition()[index];
                    cmd_q[index] = state.getCommandedJointPosition()[index];
                    cmd_tau[index] = state.getCommandedTorque()[index];
                    tau[index] = state.getMeasuredTorque()[index];
                    tau_ext[index] = state.getExternalTorque()[index];
                }
                break;
            case command_mode::WAIT_COMMAND:
            default:
                for (size_t index = 0; index < number_of_joints; ++index)
                {
                    q[index] = state.getMeasuredJointPosition()[index];
                    cmd_q[index] = state.getIpoJointPosition()[index];
                    cmd_tau[index] = state.getCommandedTorque()[index];
                    tau[index] = state.getMeasuredTorque()[index];
                    tau_ext[index] = state.getExternalTorque()[index];
                }
            }
        }

        void State::convertFrom(const EigenState &state)
        {
            cmd_q = convert<double, number_of_joints>(state.cmd_q);
            cmd_tau = convert<double, number_of_joints>(state.cmd_tau);

            user_defined = convert<double, number_of_joints>(state.user_defined);
            user_defined2 = convert<double, number_of_joints>(state.user_defined2);
            user_defined3 = convert<double, number_of_joints>(state.user_defined3);
            user_defined4 = convert<double, number_of_joints>(state.user_defined4);
        }

        EigenState State::converteigen()
        {
            EigenState converted_state;
            converted_state.cmd_q = convert(cmd_q);
            converted_state.cmd_tau = convert(cmd_tau);
            converted_state.user_defined = convert(user_defined);
            converted_state.user_defined2 = convert(user_defined2);
            converted_state.user_defined3 = convert(user_defined3);
            converted_state.user_defined4 = convert(user_defined4);
            return converted_state;
        }

        void State::differential(const State &next)
        {
            sampleTime = next.sampleTime;
            if (initialized)
                for (size_t index = 0; index < number_of_joints; ++index)
                {
                    dq[index] = (next.q[index] - q[index]) / sampleTime;
                    ddq[index] = (next.dq[index] - dq[index]) / sampleTime;
                    q[index] = next.q[index];
                    cmd_q[index] = next.cmd_q[index];
                    cmd_tau[index] = next.cmd_tau[index];
                    tau[index] = next.tau[index];
                    tau_ext[index] = next.tau_ext[index];
                }
            else
                for (size_t index = 0; index < number_of_joints; ++index)
                {
                    q[index] = next.q[index];
                    dq[index] = 0.0;
                    ddq[index] = 0.0;
                    cmd_q[index] = next.cmd_q[index];
                    cmd_tau[index] = next.cmd_tau[index];
                    tau[index] = next.tau[index];
                    tau_ext[index] = next.tau_ext[index];
                }
            initialized = true;
        }

        std::ostream &operator<<(std::ostream &os, const State &cont)
        {
            if(cont.print_state & PrintInfo::Q)
                os << "q: " << convert<double, number_of_joints>(cont.q).transpose() << std::endl;

            if(cont.print_state & PrintInfo::DQ)
                os << "dq: " << convert<double, number_of_joints>(cont.dq).transpose() << std::endl;

            if(cont.print_state & PrintInfo::DDQ)
                os << "ddq: " << convert<double, number_of_joints>(cont.ddq).transpose() << std::endl;

            if(cont.print_state & PrintInfo::CMD_Q)
                os << "cmd_q: " << convert<double, number_of_joints>(cont.cmd_q).transpose() << std::endl;

            if(cont.print_state & PrintInfo::CMD_TAU)
                os << "cmd_tau: " << convert<double, number_of_joints>(cont.cmd_tau).transpose() << std::endl;

            if(cont.print_state & PrintInfo::TAU)
                os << "tau: " << convert<double, number_of_joints>(cont.tau).transpose() << std::endl;

            if(cont.print_state & PrintInfo::GRAVITY)
                os << "gravity: " << convert<double, number_of_joints>(cont.gravity).transpose() << std::endl;
            
            if(cont.print_state & PrintInfo::CORIOLIS)
                os << "coriolis: " << convert<double, number_of_joints>(cont.coriolis).transpose() << std::endl;
            
            if(cont.print_state & PrintInfo::TAU_EXT)
                os << "tau_ext: " << convert<double, number_of_joints>(cont.tau_ext).transpose() << std::endl;

            if(cont.print_state & PrintInfo::USER_DEFINED)
                os << "user_defined: " << convert<double, number_of_joints>(cont.user_defined).transpose() << std::endl;

            if(cont.print_state & PrintInfo::USER_DEFINED_2)
                os << "user_defined_2: " << convert<double, number_of_joints>(cont.user_defined2).transpose() << std::endl;

            if(cont.print_state & PrintInfo::USER_DEFINED_3)
                os << "user_defined_3: " << convert<double, number_of_joints>(cont.user_defined3).transpose() << std::endl;

            if(cont.print_state & PrintInfo::USER_DEFINED_4)
                os << "user_defined_4: " << convert<double, number_of_joints>(cont.user_defined4).transpose() << std::endl;

            Eigen::Matrix<double, 3, 3> f_end_effector_rot = Eigen::Matrix<double, 3, 3>::Zero();
            Eigen::Matrix<double, 3, 1> f_end_effector_transl = Eigen::Matrix<double, 3, 1>::Zero();

            for (size_t cart_row = 0; cart_row < 3; ++cart_row)
            {
                f_end_effector_transl(cart_row, 0) = cont.translation[cart_row];
                for (size_t cart_col = 0; cart_col < 3; ++cart_col)
                    f_end_effector_rot(cart_row, cart_col) = cont.rotation[cart_row][cart_col];
            }

            if(cont.print_state & PrintInfo::TRANSLATION)
                os << "translation: " << f_end_effector_transl.transpose() << std::endl;

            if(cont.print_state & PrintInfo::ROTATION)
                os << "rotation: \n"
                    << f_end_effector_rot << std::endl;

            return os;
        }

    }
}
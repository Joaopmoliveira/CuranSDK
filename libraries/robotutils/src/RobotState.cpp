#include "robotutils/RobotState.h"
#include "robotutils/ConvertEigenToArray.h"

namespace curan {
namespace robotic {

State::State(const  KUKA::FRI::LBRState& state, const command_mode& mode){
    sampleTime = state.getSampleTime();
    switch(mode){
        case command_mode::COMMAND:
        for(size_t index = 0; index < number_of_joints; ++index){
            q[index] = state.getMeasuredJointPosition()[index];
            cmd_q[index] = state.getCommandedJointPosition()[index];
            cmd_tau[index] = state.getCommandedTorque()[index];
            tau[index] = state.getMeasuredTorque()[index];
            tau_ext[index] = state.getExternalTorque()[index];
        }
        break;
        case command_mode::MONITOR:
        for(size_t index = 0; index < number_of_joints; ++index){
            q[index] = state.getMeasuredJointPosition()[index];
            cmd_q[index] = state.getCommandedJointPosition()[index];
            cmd_tau[index] = state.getCommandedTorque()[index];
            tau[index] = state.getMeasuredTorque()[index];
            tau_ext[index] = state.getExternalTorque()[index];
        }
        break;
        case command_mode::WAIT_COMMAND:
        default:
        for(size_t index = 0; index < number_of_joints; ++index){
            q[index] = state.getMeasuredJointPosition()[index];
            cmd_q[index] = state.getIpoJointPosition()[index];
            cmd_tau[index] = state.getCommandedTorque()[index];
            tau[index] = state.getMeasuredTorque()[index];
            tau_ext[index] = state.getExternalTorque()[index];
        }
    }
}

void State::convertFrom(const EigenState& state){
    cmd_q = convert<double,number_of_joints>(state.cmd_q);
    cmd_tau = convert<double,number_of_joints>(state.cmd_tau);
}

EigenState State::converteigen(){
    EigenState converted_state;
    converted_state.cmd_q = convert(cmd_q);
    converted_state.cmd_tau = convert(cmd_tau);
    converted_state.user_defined = convert(user_defined);
    converted_state.user_defined2 = convert(user_defined2);
    converted_state.user_defined3 = convert(user_defined3);
    converted_state.user_defined4 = convert(user_defined4);
    return converted_state;
}

void State::differential(const State& next){
    sampleTime = next.sampleTime;
    if(initialized)
        for(size_t index = 0; index < number_of_joints; ++index){
            dq[index] = (next.q[index]-q[index])/sampleTime;
            ddq[index] = (next.dq[index]-dq[index])/sampleTime;
            q[index] = next.q[index];
            cmd_q[index] = next.cmd_q[index];
            cmd_tau[index] = next.cmd_tau[index];
            tau[index] = next.tau[index];
            tau_ext[index] = next.tau_ext[index];
        } 
    else 
        for(size_t index = 0; index < number_of_joints; ++index){
            q[index] = next.q[index];
            cmd_q[index] = next.cmd_q[index];
            cmd_tau[index] = next.cmd_tau[index];
            tau[index] = next.tau[index];
            tau_ext[index] = next.tau_ext[index];
        } 
    initialized = true;
}

}
}
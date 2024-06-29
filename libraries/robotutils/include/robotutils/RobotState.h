#ifndef CURAN_ROBOT_STATE_HEADER_
#define CURAN_ROBOT_STATE_HEADER_

#include "rbdl/Logging.h"
#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"
#include "utils/Reader.h"
#include "friLBRClient.h"

namespace curan {
namespace robotic {

constexpr size_t number_of_joints = 7;

struct EigenState{
    Eigen::Matrix<double,number_of_joints,1> cmd_q;
    Eigen::Matrix<double,number_of_joints,1> cmd_tau;

    Eigen::Matrix<double,number_of_joints,1> user_defined;
    Eigen::Matrix<double,number_of_joints,1> user_defined2;
    Eigen::Matrix<double,number_of_joints,1> user_defined3;
    Eigen::Matrix<double,number_of_joints,1> user_defined4;

    inline void set_torque_ref( Eigen::Matrix<double,number_of_joints,1> in_tau_cmd){
        cmd_tau = in_tau_cmd;
    }

    inline void set_joint_ref( Eigen::Matrix<double,number_of_joints,1> in_cmd_q){
        cmd_q = in_cmd_q;
    }
};

struct State{

    enum command_mode{
        MONITOR,
        COMMAND,
        WAIT_COMMAND
    };

    State(const KUKA::FRI::LBRState& state, const command_mode& mode);
    State() :q{},
            dq{},
            ddq{},
            cmd_q{},
            cmd_tau{},
            tau{},
            gravity{},
            coriolis{},
            tau_ext{},
            translation{},
            rotation{},
            jacobian{},
            massmatrix{},
            invmassmatrix{}
            {}

    std::array<double,number_of_joints> q;
    std::array<double,number_of_joints> dq;
    std::array<double,number_of_joints> ddq;

    std::array<double,number_of_joints> cmd_q;
    std::array<double,number_of_joints> cmd_tau;
    std::array<double,number_of_joints> tau;

    std::array<double,number_of_joints> gravity;
    std::array<double,number_of_joints> coriolis;

    std::array<double,number_of_joints> tau_ext;
    std::array<double,3> translation;
    std::array<std::array<double,3>,3> rotation;

    std::array<std::array<double,number_of_joints>,6> jacobian;
    std::array<std::array<double,number_of_joints>,number_of_joints> massmatrix;
    std::array<std::array<double,number_of_joints>,number_of_joints> invmassmatrix;

    std::array<double,number_of_joints> user_defined;
    std::array<double,number_of_joints> user_defined2;
    std::array<double,number_of_joints> user_defined3;
    std::array<double,number_of_joints> user_defined4;
    double sampleTime{1e-3};
    bool initialized{false};

    void differential(const State& next);
    EigenState converteigen();
    void convertFrom(const EigenState&);
};

std::ostream& operator<<(std::ostream& os, const std::list<State>& cont);

}
}

#endif
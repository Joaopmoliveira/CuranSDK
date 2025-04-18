#ifndef CURAN_ROBOT_STATE_HEADER_
#define CURAN_ROBOT_STATE_HEADER_

#include "rbdl/Logging.h"
#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"
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

struct State;

struct WrappedState{
    Eigen::Matrix<double,number_of_joints,1> f_q;
    Eigen::Matrix<double,number_of_joints,1> f_dq;
    Eigen::Matrix<double,number_of_joints,1> f_ddq;
    Eigen::Matrix<double,4,4> f_end_effector;
    Eigen::Matrix<double,number_of_joints,1> f_measured_torque;

    WrappedState(const State& in);
};

enum PrintInfo : std::uint32_t {
    Q = 1<<1,
    DQ = 1<<2,
    DDQ = 1<<3,
    CMD_Q = 1<<4,
    CMD_TAU = 1<<5,
    TAU = 1<<6,
    GRAVITY = 1<<7,
    CORIOLIS = 1<<8,
    TAU_EXT = 1<<9,
    TRANSLATION = 1<<10,
    ROTATION = 1<<11,
    JACOBIAN = 1<<12,
    MASS_MATRIX = 1<<13,
    USER_DEFINED = 1<<14,
    USER_DEFINED_2 = 1<<15,
    USER_DEFINED_3 = 1<<16,
    USER_DEFINED_4 = 1<<17,
    PRINT_ALL = std::numeric_limits<std::uint32_t>::max()
};

struct State{

    PrintInfo print_state = PrintInfo::PRINT_ALL;

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

std::ostream& operator<<(std::ostream& os, const State& cont);

std::ostream& operator<<(std::ostream& os, const std::list<State>& cont);

}
}

#endif
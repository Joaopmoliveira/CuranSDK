#ifndef CURAN_LBR_CONTROLLER_
#define CURAN_LBR_CONTROLLER_

#include <array>
#include <ostream>

#include "ConvertEigenToArray.h"
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include "friLBRClient.h"

namespace curan {
namespace robotic {

constexpr size_t number_of_joints = 7;

struct EigenState{
    Eigen::Matrix<double,number_of_joints,1> q;
    Eigen::Matrix<double,number_of_joints,1> dq;
    Eigen::Matrix<double,number_of_joints,1> ddq;
    Eigen::Matrix<double,number_of_joints,1> cmd_q;
    Eigen::Matrix<double,number_of_joints,1> cmd_tau;
    Eigen::Matrix<double,number_of_joints,1> tau;
    Eigen::Matrix<double,number_of_joints,1> tau_ext;
    Eigen::Matrix<double,3,1> translation;
    Eigen::Matrix<double,3,3> rotation;
    Eigen::Matrix<double,number_of_joints,number_of_joints> jacobian;
    double sampleTime{1e-3}; 

    inline void set_torque_ref( Eigen::Matrix<double,number_of_joints,1> in_tau_cmd){
        cmd_tau = in_tau_cmd;
    }

    inline void set_joint_ref( Eigen::Matrix<double,number_of_joints,1> in_cmd_q){
        cmd_q = in_cmd_q;
    }
};

struct UserData{
    virtual EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state);
};

struct State{
    State(const KUKA::FRI::LBRState& state);
    State(){};

    std::array<double,number_of_joints> q;
    std::array<double,number_of_joints> dq;
    std::array<double,number_of_joints> ddq;
    std::array<double,number_of_joints> cmd_q;
    std::array<double,number_of_joints> cmd_tau;
    std::array<double,number_of_joints> tau;
    std::array<double,number_of_joints> tau_ext;
    std::array<double,3> translation;
    std::array<std::array<double,3>,3> rotation;
    std::array<std::array<double,number_of_joints>,number_of_joints> jacobian;
    double sampleTime{1e-3};
    bool initialized{false};

    void differential(const State& next);
    void update_iiwa(RobotParameters* iiwa,kuka::Robot* robot,const Vector3d& pointPosition);
    EigenState converteigen();
};



template<typename container>
std::ostream& operator<<(std::ostream& os, const container& cont)
{
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_q = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zeros(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_dq = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zeros(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_ddq = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zeros(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_cmd_q = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zeros(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_cmd_tau = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zeros(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_tau = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zeros(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_tau_ext = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zeros(number_of_joints,cont.size());
    Eigen::Matrix<double,3,Eigen::Dynamic> arr_translation = Eigen::Matrix<double,3,Eigen::Dynamic>::Zeros(3,cont.size());
    Eigen::Matrix<double,9,Eigen::Dynamic> arr_rotation = Eigen::Matrix<double,9,Eigen::Dynamic>::Zeros(9,cont.size());

    for(size_t i=0; i< cont.size(); ++i){
        arr_q.col(i) = convert(cont[i].q);
        arr_dq.col(i) = convert(cont[i].dq);
        arr_ddq.col(i) = convert(cont[i].ddq);
        arr_cmd_q.col(i) = convert(cont[i].cmd_q);
        arr_cmd_tau.col(i) = convert(cont[i].cmd_tau);
        arr_tau.col(i) = convert(cont[i].tau);
        arr_tau_ext.col(i) = convert(cont[i].tau_ext);
        arr_translation.col(i) = convert(cont[i].translation);
        arr_rotation.col(i) = convert(cont[i].rotation);
    }
    return os;
}

struct RobotLimits
{
    VectorNd qMin;
    VectorNd qMax;
    VectorNd qDotMin;
    VectorNd qDotMax;
    VectorNd qDotDotMax;
    VectorNd qDotDotMin;

    RobotLimits()
    {
        qMin = VectorNd::Zero(number_of_joints, 1);
        qMax = VectorNd::Zero(number_of_joints, 1);
        qDotMin = VectorNd::Zero(number_of_joints, 1);
        qDotMax = VectorNd::Zero(number_of_joints, 1);
        qDotDotMax = VectorNd::Zero(number_of_joints, 1);
        qDotDotMin = VectorNd::Zero(number_of_joints, 1);

    }
};

using AtomicState = std::atomic<State>;



class RobotLBR : public KUKA::FRI::LBRClient
{
public:
    RobotLBR(UserData* data);

    ~RobotLBR();

    virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);

    virtual void monitor();

    virtual void waitForCommand();

    virtual void command();

    const AtomicState& atomic_acess(){
        return atomic_state;
    }

private:
    VectorNd addConstraints(const VectorNd& tauStack, double dt);

    std::unique_ptr<kuka::Robot> robot;
    std::unique_ptr<RobotParameters> iiwa;
    State current_state;
    EigenState eigen_state;
    UserData* user_data = nullptr;
    AtomicState atomic_state;

    RobotLimits myIIWALimits;

    double toolMass;
    Vector3d toolCOM;
    Matrix3d toolInertia;
    ToolData* myTool;

    Vector3d pointPosition = Vector3d(0, 0, 0.045); 

    double sampleTime = 0;
    double currentTime = 0;
};

}
}

#endif
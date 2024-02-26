#ifndef CURAN_LBR_CONTROLLER_
#define CURAN_LBR_CONTROLLER_

#include <array>
#include <ostream>
#include <memory>
#include "ConvertEigenToArray.h"
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include "friLBRClient.h"
#include <nlohmann/json.hpp>

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
    Eigen::Matrix<double,6,number_of_joints> jacobian;
    Eigen::Matrix<double,number_of_joints,number_of_joints> massmatrix;
    Eigen::Matrix<double,number_of_joints,number_of_joints> invmassmatrix;
    Eigen::Matrix<double,number_of_joints,1> user_defined;
    double sampleTime{1e-3}; 

    inline void set_torque_ref( Eigen::Matrix<double,number_of_joints,1> in_tau_cmd){
        cmd_tau = in_tau_cmd;
    }

    inline void set_joint_ref( Eigen::Matrix<double,number_of_joints,1> in_cmd_q){
        cmd_q = in_cmd_q;
    }
};

struct UserData{
    virtual EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians);
    virtual ~UserData(){};
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

    std::array<std::array<double,number_of_joints>,6> jacobian;
    std::array<std::array<double,number_of_joints>,number_of_joints> massmatrix;
    std::array<std::array<double,number_of_joints>,number_of_joints> invmassmatrix;

    std::array<double,number_of_joints> user_defined;
    double sampleTime{1e-3};
    bool initialized{false};

    void differential(const State& next);
    void update_iiwa(RobotParameters* iiwa,kuka::Robot* robot,const Vector3d& pointPosition);
    EigenState converteigen();
    void convertFrom(const EigenState&);
};

std::ostream& operator<<(std::ostream& os, const std::list<State>& cont);

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

    inline const AtomicState& atomic_acess(){
        return atomic_state;
    }

    inline operator bool() const {
        return continue_robot_motion.load(std::memory_order_relaxed); 
    }

    inline void cancel(){
        continue_robot_motion.store(false,std::memory_order_relaxed);
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
    std::atomic<bool> continue_robot_motion{true};

    Vector3d pointPosition = Vector3d(0, 0, 0.045); 

    double sampleTime = 0;
    double currentTime = 0;
};

}
}

#endif
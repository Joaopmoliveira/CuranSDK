#ifndef CURAN_LBR_CONTROLLER_
#define CURAN_LBR_CONTROLLER_

#include <array>
#include <ostream>
#include <memory>
#include "ConvertEigenToArray.h"
#include "RobotModel.h"
#include <nlohmann/json.hpp>

namespace curan {
namespace robotic {

struct UserData{
    virtual EigenState&& update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians);
    virtual ~UserData(){};
};

using AtomicState = std::atomic<State>;

class RobotLBR : public KUKA::FRI::LBRClient
{
public:
    /*
    This constructor assumes no geometric limits on the joints, 
    care should be taken if you don't want to break the robot 
    while controling it
    */
    RobotLBR(UserData* data,const std::filesystem::path& models_data_directory);
    /*
    This constructor assumes geometric limits on the joints, 
    thus if your control law pushes the robot towards a joint, velocity or 
    acceleration limit, it is automatically rate limited
    */
    RobotLBR(UserData* data,const std::filesystem::path& models_data_directory,const std::filesystem::path& kinematic_limits_directory);

    ~RobotLBR();

    virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);

    virtual void monitor();

    virtual void waitForCommand();

    virtual void command();

    inline UserData* view_userdata(){
        return user_data;
    }

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
    State current_state;
    EigenState eigen_state;
    UserData* user_data = nullptr;
    AtomicState atomic_state;
    RobotModel<number_of_joints> robot_model;
    std::atomic<bool> continue_robot_motion{true};

    double sampleTime = 0;
    double currentTime = 0;
};

}
}

#endif
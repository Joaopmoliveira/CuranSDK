#include "robotutils/LBRController.h"
#include "utils/Reader.h"

namespace curan {
namespace robotic {

EigenState&& UserData::update(const RobotModel<number_of_joints>& iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
    return std::move(state);
}

RobotLBR::RobotLBR(UserData* in_struct,const std::filesystem::path& models_data_directory) : user_data{in_struct},robot_model{models_data_directory}{
    if(user_data==nullptr)
        throw std::runtime_error("failed to supply a controller to be used");

}

RobotLBR::RobotLBR(UserData* in_struct,const std::filesystem::path& models_data_directory,const std::filesystem::path& kinematic_limits_directory) : user_data{in_struct},robot_model{models_data_directory,kinematic_limits_directory}{
    if(user_data==nullptr)
        throw std::runtime_error("failed to supply a controller to be used");
    
}

RobotLBR::~RobotLBR(){

}

void RobotLBR::onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState){
    LBRClient::onStateChange(oldState, newState);
    // react on state change events
    switch (newState)
    {
    case KUKA::FRI::MONITORING_WAIT:
    {
        break;
    }
    case KUKA::FRI::MONITORING_READY:
    {
        sampleTime = robotState().getSampleTime();
        break;
    }
    case KUKA::FRI::COMMANDING_WAIT:
    {
        break;
    }
    case KUKA::FRI::COMMANDING_ACTIVE:
    {
        break;
    }
    default:
    {
        break;
    }
    }
}

void RobotLBR::monitor(){
    robotCommand().setJointPosition(robotState().getCommandedJointPosition());
    current_state.differential(State{robotState(),State::MONITOR});
    robot_model.update(current_state);
}

void RobotLBR::waitForCommand(){
    // If we want to command torques, we have to command them all the time; even in
    // waitForCommand(). This has to be done due to consistency checks. In this state it is
    // only necessary, that some torque vlaues are sent. The LBR does not take the
    // specific value into account.
    current_state.differential(State{robotState(),State::WAIT_COMMAND});
    robot_model.update(current_state);
    eigen_state = current_state.converteigen();
    if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
        robotCommand().setTorque(current_state.cmd_tau.data());
        robotCommand().setJointPosition(robotState().getIpoJointPosition());            // Just overlaying same position
    }
}

void RobotLBR::command(){
    current_state.differential(State{robotState(),State::COMMAND});
    robot_model.update(current_state);
    current_state.cmd_tau = std::array<double,7>();
    eigen_state = current_state.converteigen();
    Eigen::MatrixXd task_jacobian = Eigen::MatrixXd::Identity(number_of_joints,number_of_joints);
    eigen_state = std::move(user_data->update(robot_model,std::move(eigen_state),task_jacobian));
    //eigen_state.cmd_tau = add_constraints<number_of_joints>(robot_model,eigen_state.cmd_tau, 0.005);
    current_state.convertFrom(eigen_state);
    atomic_state.store(current_state,std::memory_order_relaxed);
    if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
        robotCommand().setJointPosition(eigen_state.cmd_q.data());
        robotCommand().setTorque(eigen_state.cmd_tau.data());
    }
    currentTime = currentTime + robotState().getSampleTime();
}

std::ostream& operator<<(std::ostream& os, const std::list<State>& cont)
{
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_q = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_dq = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_ddq = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_cmd_q = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_cmd_tau = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_tau = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_gravity = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_coriolis = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());

    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_tau_ext = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    Eigen::Matrix<double,3,Eigen::Dynamic> arr_translation = Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3,cont.size());
    Eigen::Matrix<double,9,Eigen::Dynamic> arr_rotation = Eigen::Matrix<double,9,Eigen::Dynamic>::Zero(9,cont.size());

    Eigen::Matrix<double,number_of_joints*6,Eigen::Dynamic> arr_jacobian = Eigen::Matrix<double,number_of_joints*6,Eigen::Dynamic>::Zero(number_of_joints*6,cont.size());
    Eigen::Matrix<double,number_of_joints*number_of_joints,Eigen::Dynamic> arr_massmatrix = Eigen::Matrix<double,number_of_joints*number_of_joints,Eigen::Dynamic>::Zero(number_of_joints*number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_user_defined = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_user_defined_2 = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_user_defined_3 = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_user_defined_4 = Eigen::Matrix<double,number_of_joints,Eigen::Dynamic>::Zero(number_of_joints,cont.size());
    
    auto begin = cont.begin();
    for(size_t i=0; begin!=cont.end(); ++i,++begin){
        
        arr_q.col(i) = convert((*begin).q);
        arr_dq.col(i) = convert((*begin).dq);
        arr_ddq.col(i) = convert((*begin).ddq);
        arr_cmd_q.col(i) = convert((*begin).cmd_q);
        arr_cmd_tau.col(i) = convert((*begin).cmd_tau);
        arr_tau.col(i) = convert((*begin).tau);
        arr_gravity.col(i) = convert((*begin).gravity);
        arr_coriolis.col(i) = convert((*begin).coriolis);
        arr_tau_ext.col(i) = convert((*begin).tau_ext);
        arr_translation.col(i) = convert((*begin).translation);
        arr_rotation.col(i) = convert((*begin).rotation);
        arr_jacobian.col(i) = convert((*begin).jacobian);
        arr_massmatrix.col(i) = convert((*begin).massmatrix);
        arr_user_defined.col(i) = convert((*begin).user_defined);
        arr_user_defined_2.col(i) = convert((*begin).user_defined2);
        arr_user_defined_3.col(i) = convert((*begin).user_defined3);
        arr_user_defined_4.col(i) = convert((*begin).user_defined4);
    }

    nlohmann::json measurments;
    std::stringstream ss;
    ss << arr_q;
    measurments["q"] = ss.str();
    ss = std::stringstream{};
    ss << arr_dq;
    measurments["dq"] = ss.str();
    ss = std::stringstream{};
    ss << arr_ddq;
    measurments["ddq"] = ss.str();
    ss = std::stringstream{};
    ss << arr_cmd_q;
    measurments["cmd_q"] = ss.str();
    ss = std::stringstream{};
    ss << arr_cmd_tau;
    measurments["cmd_tau"] = ss.str();
    ss = std::stringstream{};
    ss << arr_tau;
    measurments["tau"] = ss.str();
    ss = std::stringstream{};
    ss << arr_gravity;
    measurments["gravity"] = ss.str();
    ss = std::stringstream{};
    ss << arr_coriolis;
    measurments["coriolis"] = ss.str();
    ss = std::stringstream{};
    ss << arr_tau_ext;
    measurments["tau_ext"] = ss.str();
    ss = std::stringstream{};
    ss << arr_translation;
    measurments["translation"] = ss.str();
    ss = std::stringstream{};
    ss << arr_rotation;
    measurments["rotation"] = ss.str();
    ss = std::stringstream{};
    ss << arr_jacobian; 
    measurments["jacobians"] = ss.str();
    ss = std::stringstream{};
    ss << arr_massmatrix;  
    measurments["massmatrix"] = ss.str();
    ss = std::stringstream{};
    ss << arr_user_defined;
    measurments["userdef"] = ss.str();
    ss = std::stringstream{};
    ss << arr_user_defined_2;
    measurments["userdef2"] = ss.str();
    ss = std::stringstream{};
    ss << arr_user_defined_3;
    measurments["userdef3"] = ss.str();
    ss = std::stringstream{};
    ss << arr_user_defined_4;
    measurments["userdef4"] = ss.str();
    os << measurments.dump();
    return os;
} 

std::istream& operator>>(std::istream& os, std::list<State>& cont)
{
    std::string cache;
    nlohmann::json measurments = nlohmann::json::parse(os);
    cache = measurments["q"];
    std::stringstream ss;
    ss.str(cache);
    auto readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_q = readmat;
    cache = measurments["dq"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_dq = readmat;
    cache = measurments["ddq"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_ddq = readmat;
    cache = measurments["cmd_q"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_cmd_q = readmat;
    cache = measurments["cmd_tau"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_cmd_tau = readmat;
    cache = measurments["tau"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_tau = readmat;
    cache = measurments["gravity"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_gravity = readmat;
    cache = measurments["coriolis"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_coriolis = readmat;
    cache = measurments["tau_ext"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_tau_ext = readmat;
    cache = measurments["translation"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,3,Eigen::Dynamic> arr_translation = readmat;
    cache = measurments["rotation"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,9,Eigen::Dynamic> arr_rotation = readmat;
    cache = measurments["jacobians"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints*6,Eigen::Dynamic> arr_jacobian = readmat;
    cache = measurments["massmatrix"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints*number_of_joints,Eigen::Dynamic> arr_massmatrix = readmat;
    cache = measurments["userdef"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_user_defined = readmat;
    cache = measurments["userdef2"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_user_defined_2 = readmat;
    cache = measurments["userdef3"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_user_defined_3 = readmat;
    cache = measurments["userdef4"];
    ss.clear();
    ss.str(cache);
    readmat = curan::utilities::convert_matrix(ss);
    Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> arr_user_defined_4 = readmat;
    
    for(size_t i=0; i < (int)readmat.cols(); ++i){
        curan::robotic::State state_i;
        state_i.q = curan::robotic::convert<double,number_of_joints>(arr_q.col(i));
        state_i.dq = curan::robotic::convert<double,number_of_joints>(arr_dq.col(i));
        state_i.ddq = curan::robotic::convert<double,number_of_joints>(arr_ddq.col(i));
        state_i.cmd_q = curan::robotic::convert<double,number_of_joints>(arr_cmd_q.col(i));
        state_i.cmd_tau = curan::robotic::convert<double,number_of_joints>(arr_cmd_tau.col(i));
        state_i.tau = curan::robotic::convert<double,number_of_joints>(arr_tau.col(i));
        state_i.gravity = curan::robotic::convert<double,number_of_joints>(arr_gravity.col(i));
        state_i.coriolis = curan::robotic::convert<double,number_of_joints>(arr_coriolis.col(i));
        state_i.tau_ext = curan::robotic::convert<double,number_of_joints>(arr_tau_ext.col(i));
        state_i.translation = curan::robotic::convert<double,3>(arr_translation.col(i));
        state_i.user_defined = curan::robotic::convert<double,number_of_joints>(arr_user_defined.col(i));
        state_i.user_defined2 = curan::robotic::convert<double,number_of_joints>(arr_user_defined_2.col(i));
        state_i.user_defined3 = curan::robotic::convert<double,number_of_joints>(arr_user_defined_3.col(i));
        state_i.user_defined4 = curan::robotic::convert<double,number_of_joints>(arr_user_defined_4.col(i));
        cont.push_back(state_i);

        //state_i.rotation = arr_rotation.col(i); TODO these are left for later since I don't currently have much utility for them
        //state_i.jacobian = arr_jacobian.col(i);
        //state_i.massmatrix = arr_massmatrix.col(i);
    }

    return os;
} 

}
}
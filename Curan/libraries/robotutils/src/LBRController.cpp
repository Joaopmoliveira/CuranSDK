#include "robotutils/LBRController.h"

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

EigenState&& UserData::update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians){
    return std::move(state);
}

void State::convertFrom(const EigenState& state){
    q = convert<double,number_of_joints>(state.q);
    dq = convert<double,number_of_joints>(state.dq);
    ddq = convert<double,number_of_joints>(state.ddq);
    cmd_q = convert<double,number_of_joints>(state.cmd_q);
    cmd_tau = convert<double,number_of_joints>(state.cmd_tau);
    tau = convert<double,number_of_joints>(state.tau);
    gravity = convert<double,number_of_joints>(state.gravity);
    coriolis = convert<double,number_of_joints>(state.coriolis);
    tau_ext = convert<double,number_of_joints>(state.tau_ext);
    constexpr size_t translation_size = 3;
    constexpr size_t rotation_size = 3;
    translation = convert<double,translation_size>(state.translation);
    for(size_t i = 0; i< rotation.size(); ++i)
        rotation[i] = convert<double,rotation_size>(state.rotation.row(i).transpose());
    for(size_t i = 0; i< massmatrix.size(); ++i){
        if(i< jacobian.size()) jacobian[i] = convert<double,number_of_joints>(state.jacobian.row(i).transpose());
        massmatrix[i] = convert<double,number_of_joints>(state.massmatrix.row(i).transpose());
        invmassmatrix[i] = convert<double,number_of_joints>(state.invmassmatrix.row(i).transpose());
    }
    user_defined = convert<double,number_of_joints>(state.user_defined);
    user_defined2 = convert<double,number_of_joints>(state.user_defined2);
    user_defined3 = convert<double,number_of_joints>(state.user_defined3);
    user_defined4 = convert<double,number_of_joints>(state.user_defined4);
    sampleTime = state.sampleTime;
}

EigenState State::converteigen(){
    EigenState converted_state;
    converted_state.q = convert(q);
    converted_state.dq = convert(dq);
    converted_state.ddq = convert(ddq);
    converted_state.cmd_q = convert(cmd_q);
    converted_state.cmd_tau = convert(cmd_tau);
    converted_state.tau = convert(tau);
    converted_state.gravity = convert(gravity);
    converted_state.coriolis = convert(coriolis);
    converted_state.tau_ext = convert(tau_ext);
    converted_state.translation = convert(translation);
    for(size_t i = 0; i< rotation.size(); ++i)
        converted_state.rotation.row(i) = convert(rotation[i]).transpose();
    for(size_t i = 0; i< massmatrix.size(); ++i){
        if(i< jacobian.size()) converted_state.jacobian.row(i) = convert(jacobian[i]).transpose();
        converted_state.massmatrix.row(i) = convert(massmatrix[i]).transpose();
        converted_state.invmassmatrix.row(i) = convert(invmassmatrix[i]).transpose();
    }
    converted_state.user_defined = convert(user_defined);
    converted_state.user_defined2 = convert(user_defined2);
    converted_state.user_defined3 = convert(user_defined3);
    converted_state.user_defined4 = convert(user_defined4);
    converted_state.sampleTime = sampleTime;
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

void State::update_iiwa(RobotParameters* iiwa,kuka::Robot* robot,const Vector3d& pointPosition){
    for (int index = 0; index < number_of_joints; ++index) {
        iiwa->q[index] = q[index];
        iiwa->qDot[index] = dq[index];
    }
    Vector3d tmp_p_0_7 = Vector3d::Zero();
    Matrix3d  tmp_R_0_7 = Matrix3d::Identity(); 
    MatrixNd tmp_jacobian = MatrixNd::Zero(number_of_joints,number_of_joints);
    robot->getMassMatrix(iiwa->M, iiwa->q);
    iiwa->M(6, 6) = 45 * iiwa->M(6, 6);   
    iiwa->Minv = iiwa->M.inverse();
    robot->getCoriolisAndGravityVector(iiwa->c, iiwa->g, iiwa->q, iiwa->qDot);
    robot->getWorldCoordinates(tmp_p_0_7, iiwa->q, pointPosition, 7);  
    robot->getRotationMatrix(tmp_R_0_7, iiwa->q, number_of_joints); 
    robot->getJacobian(tmp_jacobian, iiwa->q, pointPosition, 7);    
    for(size_t row = 0; row < number_of_joints; ++row){
        gravity[row] = iiwa->g[row];
        coriolis[row] = iiwa->c[row];
        for(size_t col = 0; col < number_of_joints; ++col){
            if(row < 6) jacobian[row][col] = tmp_jacobian(row,col);
            massmatrix[row][col] = iiwa->M(row,col);
            invmassmatrix[row][col] = iiwa->Minv(row,col);
        }
    }
    for(size_t cart_row = 0; cart_row < 3; ++cart_row){
        translation[cart_row] = tmp_p_0_7[cart_row];
        for(size_t cart_col = 0; cart_col < 3; ++cart_col)
            rotation[cart_row][cart_col] = tmp_R_0_7(cart_row,cart_col);
    }
}

RobotLBR::RobotLBR(UserData* in_struct) : user_data{in_struct}{
    if(user_data==nullptr)
        throw std::runtime_error("failed to supply a controller to be used");
       // Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
    kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here

    robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
    iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

    // Initialize Limits
    myIIWALimits = RobotLimits();
    myIIWALimits.qMax[0] = 163 * M_PI / 180;
    myIIWALimits.qMax[1] = 113 * M_PI / 180;
    myIIWALimits.qMax[2] = 163 * M_PI / 180;
    myIIWALimits.qMax[3] = 115 * M_PI / 180;
    myIIWALimits.qMax[4] = 160 * M_PI / 180;
    myIIWALimits.qMax[5] = 110 * M_PI / 180;
    myIIWALimits.qMax[6] = 165 * M_PI / 180;

    myIIWALimits.qMin[0] = -163 * M_PI / 180;
    myIIWALimits.qMin[1] = -113 * M_PI / 180;
    myIIWALimits.qMin[2] = -163 * M_PI / 180;
    myIIWALimits.qMin[3] = -115 * M_PI / 180;
    myIIWALimits.qMin[4] = -160 * M_PI / 180;
    myIIWALimits.qMin[5] = -110 * M_PI / 180;
    myIIWALimits.qMin[6] = -165 * M_PI / 180;

    myIIWALimits.qDotMax[0] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[1] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[2] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[3] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[4] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[5] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[6] = 155 * M_PI / 180;

    myIIWALimits.qDotMin[0] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[1] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[2] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[3] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[4] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[5] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[6] = -155 * M_PI / 180;

    myIIWALimits.qDotDotMax[0] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[1] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[2] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[3] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[4] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[5] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[6] = 300 * M_PI / 180;

    myIIWALimits.qDotDotMin[0] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[1] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[2] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[3] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[4] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[5] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[6] = -300 * M_PI / 180;

    toolMass = 0.0;
    toolCOM = Vector3d::Zero(3, 1);
    toolInertia = Matrix3d::Zero(3, 3);
    myTool = new ToolData(toolMass, toolCOM, toolInertia);
    robot->attachToolToRobotModel(myTool);
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
    current_state.update_iiwa(iiwa.get(),robot.get(),pointPosition);
}

void RobotLBR::waitForCommand(){
    // If we want to command torques, we have to command them all the time; even in
    // waitForCommand(). This has to be done due to consistency checks. In this state it is
    // only necessary, that some torque vlaues are sent. The LBR does not take the
    // specific value into account.
    current_state.differential(State{robotState(),State::WAIT_COMMAND});
    current_state.update_iiwa(iiwa.get(),robot.get(),pointPosition);
    if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
        robotCommand().setTorque(current_state.cmd_tau.data());
        robotCommand().setJointPosition(robotState().getIpoJointPosition());            // Just overlaying same position
    }
}

void RobotLBR::command(){
    current_state.differential(State{robotState(),State::COMMAND});
    current_state.update_iiwa(iiwa.get(),robot.get(),pointPosition);
    current_state.cmd_tau = std::array<double,7>();
    eigen_state = current_state.converteigen();
    Eigen::MatrixXd task_jacobian = Eigen::MatrixXd::Identity(number_of_joints,number_of_joints);
    eigen_state = std::move(user_data->update(robot.get(),iiwa.get(),std::move(eigen_state),task_jacobian));
    eigen_state.cmd_tau = addConstraints(eigen_state.cmd_tau, 0.005);
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

VectorNd RobotLBR::addConstraints(const VectorNd& tauStack, double dt)
{
    VectorNd dt2 = VectorNd::Zero(number_of_joints, 1);
    VectorNd dtvar = VectorNd::Zero(number_of_joints, 1);
    VectorNd qDownBar = VectorNd::Zero(number_of_joints, 1);
    VectorNd qTopBar = VectorNd::Zero(number_of_joints, 1);

    VectorNd qDotMaxFromQ = VectorNd::Zero(number_of_joints, 1);
    VectorNd qDotMinFromQ = VectorNd::Zero(number_of_joints, 1);
    VectorNd qDotMaxFormQDotDot = VectorNd::Zero(number_of_joints, 1);
    VectorNd qDotMinFormQDotDot = VectorNd::Zero(number_of_joints, 1);
    VectorNd vMaxVector = Vector3d::Zero(3);
    VectorNd vMinVector = Vector3d::Zero(3);
    VectorNd qDotMaxFinal = VectorNd::Zero(number_of_joints, 1);
    VectorNd qDotMinFinal = VectorNd::Zero(number_of_joints, 1);
    VectorNd aMaxqDot = VectorNd::Zero(number_of_joints, 1);
    VectorNd aMinqDot = VectorNd::Zero(number_of_joints, 1);
    VectorNd aMaxQ = VectorNd::Zero(number_of_joints, 1);
    VectorNd aMinQ = VectorNd::Zero(number_of_joints, 1);
    VectorNd aMaxVector = Vector3d::Zero(3);
    VectorNd aMinVector = Vector3d::Zero(3);
    VectorNd qDotDotMaxFinal = VectorNd::Zero(number_of_joints, 1);
    VectorNd qDotDotMinFinal = VectorNd::Zero(number_of_joints, 1);
    MatrixNd Iden = MatrixNd::Identity(number_of_joints, number_of_joints);
    VectorNd TauBar = VectorNd::Zero(number_of_joints, 1);
    VectorNd qDotDotGot = VectorNd::Zero(number_of_joints, 1);
    MatrixNd Js = MatrixNd::Zero(3, number_of_joints);

    double lowestdtFactor = 10;

    qDownBar = iiwa->q - myIIWALimits.qMin;
    qTopBar = myIIWALimits.qMax - iiwa->q;
    dtvar[0] = 3 * dt;
    dtvar[1] = 3 * dt;
    dtvar[2] = 2 * dt;
    dtvar[3] = 3 * dt;
    dtvar[4] = dt;
    dtvar[5] = dt;
    dtvar[6] = dt;

    for (int i = 0; i < number_of_joints; i++)
    {
        dt2[i] = dtvar[i];
        if (qDownBar[i] < 10 * M_PI / 180)
        {

            if (qDownBar[i] < 0)
                qDownBar[i] = 0;

            dt2[i] = ((lowestdtFactor) + (sqrt(lowestdtFactor) * sqrt(qDownBar[i] * 180 / M_PI))) * dtvar[i];

            if (dt2[i] < lowestdtFactor * dtvar[i])
                dt2[i] = lowestdtFactor * dtvar[i];
        }
        if (qTopBar[i] < 10 * M_PI / 180)
        {

            if (qTopBar[i] < 0)
                qTopBar[i] = 0;

            dt2[i] = (lowestdtFactor + (sqrt(lowestdtFactor) * sqrt(qTopBar[i] * 180 / M_PI))) * dtvar[i];
            if (dt2[i] < lowestdtFactor * dtvar[i])
                dt2[i] = lowestdtFactor * dtvar[i];

        }

        qDotMaxFromQ[i] = (myIIWALimits.qMax[i] - iiwa->q[i]) / dt2[i];
        qDotMinFromQ[i] = (myIIWALimits.qMin[i] - iiwa->q[i]) / dt2[i];
        qDotMaxFormQDotDot[i] = sqrt(2 * myIIWALimits.qDotDotMax[i] * (myIIWALimits.qMax[i] - iiwa->q[i]));
        qDotMinFormQDotDot[i] = -sqrt(2 * myIIWALimits.qDotDotMax[i] * (iiwa->q[i] - myIIWALimits.qMin[i]));

        if (myIIWALimits.qMax[i] - iiwa->q[i] < 0)
            qDotMaxFormQDotDot[i] = 1000000;

        if (iiwa->q[i] - myIIWALimits.qMin[i] < 0)
            qDotMinFormQDotDot[i] = -1000000;

        vMaxVector = Vector3d(myIIWALimits.qDotMax[i], qDotMaxFromQ[i], qDotMaxFormQDotDot[i]);
        qDotMaxFinal[i] = vMaxVector.minCoeff();


        vMinVector = Vector3d(myIIWALimits.qDotMin[i], qDotMinFromQ[i], qDotMinFormQDotDot[i]);
        qDotMinFinal[i] = vMinVector.maxCoeff();

        aMaxqDot[i] = (qDotMaxFinal[i] - iiwa->qDot[i]) / dtvar[i];
        aMinqDot[i] = (qDotMinFinal[i] - iiwa->qDot[i]) / dtvar[i];

        aMaxQ[i] = 2 * (myIIWALimits.qMax[i] - iiwa->q[i] - iiwa->qDot[i] * dt2[i]) / pow(dt2[i], 2);
        aMinQ[i] = 2 * (myIIWALimits.qMin[i] - iiwa->q[i] - iiwa->qDot[i] * dt2[i]) / pow(dt2[i], 2);

        aMaxVector = Vector3d(aMaxQ[i], aMaxqDot[i], 10000000);
        qDotDotMaxFinal[i] = aMaxVector.minCoeff();
        aMinVector = Vector3d(aMinQ[i], aMinqDot[i], -10000000);
        qDotDotMinFinal[i] = aMinVector.maxCoeff();

        if (qDotDotMaxFinal[i] < qDotDotMinFinal[i])
        {
            vMaxVector = Vector3d(INFINITY, qDotMaxFromQ[i], qDotMaxFormQDotDot[i]);
            qDotMaxFinal[i] = vMaxVector.minCoeff();

            vMinVector = Vector3d(-INFINITY, qDotMinFromQ[i], qDotMinFormQDotDot[i]);
            qDotMinFinal[i] = vMinVector.maxCoeff();

            aMaxqDot[i] = (qDotMaxFinal[i] - iiwa->qDot[i]) / dtvar[i];
            aMinqDot[i] = (qDotMinFinal[i] - iiwa->qDot[i]) / dtvar[i];

            aMaxVector = Vector3d(aMaxQ[i], aMaxqDot[i], 10000000);
            qDotDotMaxFinal[i] = aMaxVector.minCoeff();
            aMinVector = Vector3d(aMinQ[i], aMinqDot[i], -10000000);
            qDotDotMinFinal[i] = aMinVector.maxCoeff();
        }
    }


    VectorNd qDotDotS = VectorNd::Zero(number_of_joints);
    VectorNd tauS = VectorNd::Zero(number_of_joints);
    MatrixNd Psat = Iden;
    bool LimitedExceeded = true;
    bool CreateTaskSat = false;
    int NumSatJoints = 0;
    Eigen::Vector<Eigen::Index,Eigen::Dynamic> theMostCriticalOld = Eigen::Vector<Eigen::Index,Eigen::Dynamic>::Zero(number_of_joints);
    theMostCriticalOld.conservativeResize(1);
    theMostCriticalOld[0] = 100;
    bool isThere = false;
    int iO = 0;
    int cycle = 0;
    while (LimitedExceeded)
    {
        LimitedExceeded = false;
        if (CreateTaskSat)
        {
            Js.conservativeResize(NumSatJoints, number_of_joints);
            for (int i = 0; i < NumSatJoints; i++)
            {
                
                for (int k = 0; k < number_of_joints; k++)
                {
                    Js(i, k) = 0;
                }
                Js(i, (int)theMostCriticalOld[i]) = 1;
            }

            MatrixNd LambdaSatInv = Js * iiwa->Minv * Js.transpose();
            MatrixNd LambdaSatInv_aux = LambdaSatInv * LambdaSatInv.transpose();
            MatrixNd LambdaSat_aux = LambdaSatInv_aux.inverse();
            MatrixNd LambdaSat = LambdaSatInv.transpose() * LambdaSat_aux;

            MatrixNd JsatBar = iiwa->Minv * Js.transpose() * LambdaSat;
            Psat = Iden - Js.transpose() * JsatBar.transpose();
            VectorNd xDotDot_s = Js * qDotDotS;
            tauS = Js.transpose() * (LambdaSat * xDotDot_s);
        }

        TauBar = tauS + Psat * tauStack;
        qDotDotGot = iiwa->Minv * (TauBar); // it should -g -c

        isThere = false;
        for (int i = 0; i < number_of_joints; i++)
        {
            if ((qDotDotMaxFinal[i] + 0.001 < qDotDotGot[i]) || (qDotDotGot[i] < qDotDotMinFinal[i] - 0.001))
            {
                LimitedExceeded = true;
                CreateTaskSat = true;

                for (int k = 0; k < theMostCriticalOld.size(); k++)
                {
                    if (i == theMostCriticalOld[k])
                    {
                        isThere = true;
                    }
                }
                if (isThere == false)
                {

                    theMostCriticalOld.conservativeResize(iO + 1);
                    theMostCriticalOld[iO] = i;
                    iO += 1;
                }
            }
        }

        if (LimitedExceeded == true)
        {
            NumSatJoints = iO;
            theMostCriticalOld.conservativeResize(iO);
            cycle += 1;
            if (cycle > 8)
                LimitedExceeded = false;

            for (int i = 0; i < theMostCriticalOld.size(); i++)
            {
                Eigen::Index jM = theMostCriticalOld[i];

                if (qDotDotGot[jM] > qDotDotMaxFinal[jM])
                    qDotDotS[jM] = qDotDotMaxFinal[jM];

                if (qDotDotGot[jM] < qDotDotMinFinal[jM])
                    qDotDotS[jM] = qDotDotMinFinal[jM];
            }
        }
    }

    VectorNd SJSTorque = TauBar;
    return SJSTorque;
};

}
}
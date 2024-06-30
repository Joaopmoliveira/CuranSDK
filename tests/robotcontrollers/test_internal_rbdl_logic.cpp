#include "rbdl/Logging.h"
#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include "utils/Reader.h"
#include "robotutils/LBRController.h"
#include <fstream>
#include <sstream>

int main(){
    curan::robotic::RobotModel<7> robot_model{"C:/Dev/Curan/resources/models/lbrmed/robot_mass_data.json","C:/Dev/Curan/resources/models/lbrmed/robot_kinematic_limits.json"};
    return 0;
}

int main2(){
    try{
        curan::robotic::State state;
        state.q = std::array<double,7>{};
        curan::robotic::RobotModel<7> robot_model{"C:/Dev/Curan/resources/models/lbrmed/robot_mass_data.json","C:/Dev/Curan/resources/models/lbrmed/robot_kinematic_limits.json"};
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        robot_model.update(state);
        auto computed_torque = curan::robotic::add_constraints<7>(robot_model,Eigen::Matrix<double,7,1>::Ones(),0.005);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        auto first_round = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        begin = std::chrono::steady_clock::now();
        computed_torque = curan::robotic::add_constraints<7>(robot_model,Eigen::Matrix<double,7,1>::Ones(),0.002);
        end = std::chrono::steady_clock::now();

        auto second_round = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        begin = std::chrono::steady_clock::now();
        computed_torque = curan::robotic::add_constraints<7>(robot_model,Eigen::Matrix<double,7,1>::Ones(),0.001);
        end = std::chrono::steady_clock::now();

        auto third_round = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        std::cout << "joint: " << robot_model.joints().transpose() << std::endl;
        std::cout << "vel: " << robot_model.velocities().transpose() << std::endl;
        std::cout << "accel: " << robot_model.accelerations().transpose() << std::endl;
        std::cout << "translation: " << robot_model.translation().transpose() << std::endl;
        std::cout << "mass: " << robot_model.mass() << std::endl;
        std::cout << "time taken (first round): " << first_round << std::endl;
        std::cout << "time taken (second round): " << second_round << std::endl;
        std::cout << "time taken (third round): " << third_round << std::endl;
        //std::cout << "torque constrained: " << computed_torque.transpose() << std::endl;
    } catch(std::runtime_error& e){
        std::cout << "exception thrown : " << e.what() << std::endl;
        return 1; 
    } catch(...){
        std::cout << "unknown exception thrown" << std::endl;
        return 2;
    }
    return 0;
}
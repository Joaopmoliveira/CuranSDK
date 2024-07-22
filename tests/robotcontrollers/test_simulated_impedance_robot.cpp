#include "robotutils/LBRController.h"
#include "robotutils/ImpedanceController.h"
#include "robotutils/SimulateModel.h"

#include "utils/Logger.h"
#include "utils/TheadPool.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include <csignal>

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

curan::robotic::RobotLBR *robot_pointer = nullptr;

int main()
{
    using namespace curan::robotic;
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{2000, 1200};
    info.window_size = size;
    curan::renderable::Window window{info};

    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    auto robot = curan::renderable::SequencialLinks::make(create_info);
    window << robot;

    std::atomic<curan::robotic::State> atomic_state;
    std::atomic<bool> keep_running = true;
    auto pool = curan::utilities::ThreadPool::create(1);
    pool->submit(curan::utilities::Job{"value", [&]()
                                       {
                                           Eigen::Matrix<double, 3, 3> desired_rotation = Eigen::Matrix<double, 3, 3>::Identity();
                                           desired_rotation << 1.0, 0.0, 0.0,
                                                                0.0, 1.0, 0.0,
                                                                0.0, 0.0, 1.0;
                                           Eigen::Matrix<double, 3, 1> desired_translation = Eigen::Matrix<double, 3, 1>::Zero();
                                           desired_translation << -0.66809, -0.00112052, 0.443678;
                                           Transformation equilibrium{desired_rotation, desired_translation};
                                           std::unique_ptr<ImpedanceController> handguinding_controller = std::make_unique<ImpedanceController>(equilibrium,
                                                                                                                                                std::initializer_list<double>({800.0, 800.0, 800.0, 100.0, 100.0, 100.0}),
                                                                                                                                                std::initializer_list<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));

                                           curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
                                           constexpr auto sample_time = std::chrono::milliseconds(1);
                                           curan::robotic::State state;
                                           Eigen::Matrix<double, 7, 1> external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                           double time = 0.0;
                                           while (keep_running.load())
                                           {
                                               std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                                               if(time>10 && time < 15){
                                                    external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                                    external_torque[6] = 10;
                                               } else if(time> 15){
                                                    time = 0.0;
                                                    external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                               } else {
                                                    external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                               }
                                               state = curan::robotic::simulate_next_timestamp(robot_model,handguinding_controller.get(),sample_time,state,external_torque);
                                               atomic_state.store(state,std::memory_order_relaxed);
                                               time += std::chrono::duration<double>(sample_time).count();
                                           }
                                       }});

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::list<curan::robotic::State> list_of_robot_states;
    while (window.run_once())
    {
        auto current_state = atomic_state.load();
        list_of_robot_states.push_back(current_state);
        for (size_t joint_index = 0; joint_index < curan::robotic::number_of_joints; ++joint_index)
            robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, current_state.q[joint_index]);
    }
    keep_running = false;


    return 0;
}
#include "robotutils/LBRController.h"
#include "robotutils/ImpedanceController.h"
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
                                           double delta_time = 0.001;
                                           Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jacobian;
                                           Eigen::Matrix<double, 7, 1> dq = Eigen::Matrix<double, 7, 1>::Zero();
                                           Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
                                           curan::robotic::State state;
                                           state.sampleTime = delta_time;
                                           state.q = std::array<double, 7>{};
                                           curan::robotic::State next = state;
                                           double time = 0;
                                           Eigen::Matrix<double, 7, 1> external_torque = Eigen::Matrix<double, 7, 1>::Zero();

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
                                               
                                               state.differential(next);
                                               robot_model.update(state);
                                               auto actuation = handguinding_controller->update(robot_model, curan::robotic::EigenState{}, jacobian);
                                               Eigen::Matrix<double, 7, 1> ddq = robot_model.invmass() * (external_torque + actuation.cmd_tau);
                                               dq = ddq * delta_time + dq;
                                               q = dq * delta_time + q;
                                               next = state;
                                               next.q = curan::robotic::convert<double, 7>(q);
                                               atomic_state.store(state);
                                               std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                                               std::printf("time %f duration: %llu\n",time,std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());
                                               std::this_thread::sleep_for(std::chrono::milliseconds(1));
                                               end = std::chrono::steady_clock::now();
                                               time += 1e-6*std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
                                           }
                                       }});
    while (window.run_once())
    {
        auto current_state = atomic_state.load();
        for (size_t joint_index = 0; joint_index < curan::robotic::number_of_joints; ++joint_index)
            robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, current_state.q[joint_index]);
    }
    keep_running = false;
    return 0;
}
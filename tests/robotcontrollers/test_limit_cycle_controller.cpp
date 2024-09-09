#include "robotutils/LBRController.h"
#include "robotutils/CartersianVelocityController.h"
#include "utils/Logger.h"
#include "utils/TheadPool.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include <csignal>

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

curan::robotic::RobotLBR *robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal)
{
    if (robot_pointer)
        robot_pointer->cancel();
}

void custom_interface(vsg::CommandBuffer &cb, curan::robotic::RobotLBR &client)
{
    static size_t counter = 0;
    static const auto &atomic_access = client.atomic_acess();
    auto state = atomic_access.load(std::memory_order_relaxed);

    static float history = 10.0f;
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;
    {
        ImGui::Begin("Error"); // Create a window called "Hello, world!" and append into it.
        static std::array<curan::renderable::ScrollingBuffer, 6> buffers;

        ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

        static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

        if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
        {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
            for (size_t index = 0; index < 6; ++index)
            {
                std::string loc = "cmd_tau" + std::to_string(index);
                buffers[index].AddPoint(t, (float)state.user_defined2[index]);
                ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
            }

            ImPlot::EndPlot();
        }
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }

    {
        ImGui::Begin("Actuation"); // Create a window called "Hello, world!" and append into it.
        static std::array<curan::renderable::ScrollingBuffer, 6> buffers;

        ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

        static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

        if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
        {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
            for (size_t index = 0; index < 6; ++index)
            {
                std::string loc = "userdef" + std::to_string(index);
                buffers[index].AddPoint(t, (float)state.user_defined[index]);
                ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
            }

            ImPlot::EndPlot();
        }
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }

    ++counter;
}

int main()
{
    std::signal(SIGINT, signal_handler);
    using namespace curan::robotic;
    Eigen::Matrix<double, 3, 1> desired_translation = Eigen::Matrix<double, 3, 1>::Zero();
    desired_translation << -0.66809, -0.00112052, 0.443678;
    Eigen::Matrix<double, 3, 3> desired_rotation = Eigen::Matrix<double, 3, 3>::Identity();
    desired_rotation << -0.718163, -0.00186162, -0.695873,
        -0.00329559, 0.999994, 0.000725931,
        0.695868, 0.00281465, -0.718165;

    std::unique_ptr<CartersianVelocityController> handguinding_controller = std::make_unique<CartersianVelocityController>([&](const RobotModel<number_of_joints> &iiwa)
                                                                                                                           {
                                                        Eigen::Matrix<double,3,1> translated_position = iiwa.translation()-desired_translation;
                                                        Eigen::Matrix<double,3,1> desired_translated_velocity_local_frame;
                                                        double radius = std::sqrt(translated_position[0]*translated_position[0] + translated_position[1]*translated_position[1]);
                                                        double angular_velocity = 0.5;
                                                        double radial_equilibrium = 0.1;
                                                        desired_translated_velocity_local_frame[2] = -2.0*translated_position[2];
                                                        double angle_theta = (radius < 0.001) ? 0.0 : std::atan2(translated_position[1],translated_position[0]); 
                                                        Eigen::Matrix<double,2,1> rotator = Eigen::Matrix<double,2,1>::Zero();
                                                        rotator << (radial_equilibrium-radius) , angular_velocity;
                                                        Eigen::Matrix<double,2,2> jacobian_of_limit_cycle = Eigen::Matrix<double,2,2>::Zero();
                                                        jacobian_of_limit_cycle << std::cos(angle_theta) , -radius*std::sin(angle_theta) , std::sin(angle_theta) , radius*std::cos(angle_theta);
                                                        desired_translated_velocity_local_frame.block<2,1>(0,0) = jacobian_of_limit_cycle*rotator;
                                                        Eigen::Matrix<double,6,1> desired_velocity;
                                                        desired_velocity.block<3, 1>(0, 0) = desired_translated_velocity_local_frame;
                                                        Eigen::AngleAxisd E_AxisAngle(iiwa.rotation().transpose() * desired_rotation);
                                                        desired_velocity.block<3, 1>(3, 0) = E_AxisAngle.angle() * iiwa.rotation() * E_AxisAngle.axis();
                                                        return desired_velocity; },
                                                                                                                           std::initializer_list<double>({500.0, 500.0, 500.0, 25.0, 25.0, 25.0}),
                                                                                                                           std::initializer_list<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
    RobotLBR client{handguinding_controller.get(),
                    CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json",
                    CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    robot_pointer = &client;

    curan::renderable::Window::Info info;
    curan::renderable::ImGUIInterface::Info info_gui{[&](vsg::CommandBuffer &cb)
                                                     { custom_interface(cb, client); }};
    auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.imgui_interface = ui_interface;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{2000, 1200};
    info.window_size = size;
    curan::renderable::Window window{info};

    std::list<curan::robotic::State> recordings;

    {
        auto thread_pool = curan::utilities::ThreadPool::create(3);
        thread_pool->submit(curan::utilities::Job{"running controller", [&]()
                                                  {
                                                      try
                                                      {
                                                          curan::utilities::cout << "Lauching robot control thread\n";
                                                          KUKA::FRI::UdpConnection connection{200};
                                                          KUKA::FRI::ClientApplication app(connection, client);
                                                          bool success = app.connect(DEFAULT_PORTID, NULL);
                                                          while(client){
                                                              success = app.step();
                                                              if(recordings.size()< 199999)
                                                                recordings.push_back(client.atomic_acess().load(std::memory_order_relaxed));
                
                                                          }
                                                          app.disconnect();
                                                          curan::utilities::cout << "Terminating robot control thread\n";
                                                          return 0;
                                                      }
                                                      catch (...)
                                                      {
                                                          std::cout << "robot control exception\n";
                                                          return 1;
                                                      }
                                                  }});

        while(window.run_once() && client){}
        std::cout << "terminating window\n";
        client.cancel();
    } 
    if(recordings.size()==0)
        return 0;
    auto now = std::chrono::system_clock::now();
    auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    std::string filename{CURAN_COPIED_RESOURCE_PATH "/rhytmic_measurments" + std::to_string(UTC) + ".json"};
    std::cout << "creating filename with measurments :" << filename << std::endl;
    std::ofstream o(filename);
    o << recordings;
    return 0;
}
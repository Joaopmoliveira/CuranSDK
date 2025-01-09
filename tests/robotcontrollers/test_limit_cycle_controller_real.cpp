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

#include "gaussianmixtures/GMR.h"

curan::robotic::RobotLBR *robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal)
{
    if (robot_pointer)
        robot_pointer->cancel();
}

struct ScrollingBuffer
{
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000)
    {
        MaxSize = max_size;
        Offset = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y)
    {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x, y));
        else
        {
            Data[Offset] = ImVec2(x, y);
            Offset = (Offset + 1) % MaxSize;
        }
    }
    void Erase()
    {
        if (Data.size() > 0)
        {
            Data.shrink(0);
            Offset = 0;
        }
    }
};

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
        static std::array<ScrollingBuffer, 6> buffers;

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
        static std::array<ScrollingBuffer, 6> buffers;

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
    desired_translation << -0.632239 , 0.0123415 , 0.337702;
    Eigen::Matrix<double, 3, 3> desired_rotation = Eigen::Matrix<double, 3, 3>::Identity();
    desired_rotation <<   -0.998815  ,  0.0486651 ,-0.000655218,
   						   0.0486561   ,  0.998765   , 0.0100775,
  						   0.00114483  ,  0.0100337  ,  -0.999949 ;

    curan::gaussian::GMR<2,2> model;
    std::fstream gmr_model_file{CURAN_COPIED_RESOURCE_PATH"/gaussianmixtures_testing/model.txt"};
    gmr_model_file >> model;

    std::unique_ptr<CartersianVelocityController> handguinding_controller = std::make_unique<CartersianVelocityController>([&](const RobotModel<number_of_joints> &iiwa)
                                                                                                                           {
                                                        Eigen::Matrix<double,6,1> desired_velocity = Eigen::Matrix<double,6,1>::Zero();
                                                        desired_velocity.block<2, 1>(0, 0) = 2*model.likeliest(iiwa.translation().block<2,1>(0,0));
                                                        desired_velocity[2] = desired_translation[2]-iiwa.translation()[2];
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
                                                          curan::utilities::print<curan::utilities::info>("Lauching robot control thread\n");
                                                          KUKA::FRI::UdpConnection connection{200};
                                                          KUKA::FRI::ClientApplication app(connection, client);
                                                          bool success = app.connect(DEFAULT_PORTID, NULL);
                                                          while(client){
                                                              success = app.step();
                                                              if(recordings.size()< 60000)
                                                                recordings.push_back(client.atomic_acess().load(std::memory_order_relaxed));
                
                                                          }
                                                          app.disconnect();
                                                          curan::utilities::print<curan::utilities::info>("Terminating robot control thread\n");
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
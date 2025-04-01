#include "robotutils/LBRController.h"
#include "robotutils/HandGuidance.h"
#include "robotutils/RippleFilter.h"
#include "robotutils/GenericStateDerivative.h"

#include "utils/Logger.h"
#include "utils/TheadPool.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include <csignal>

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include <array>


curan::robotic::RobotLBR *robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal)
{
    std::cout << "canceling signal\n";
    if (robot_pointer)
        robot_pointer->cancel();
}

void custom_interface(vsg::CommandBuffer &cb, curan::robotic::RobotLBR &client)
{
    static size_t counter = 0;
    static const auto &atomic_access = client.atomic_acess();
    auto state = atomic_access.load(std::memory_order_relaxed);
    static float t = 0;
    static bool stop_time = false;

    if(ImGui::Begin("Torque")) // Create a window called "Hello, world!" and append into it.
    {
        static std::array<curan::renderable::ScrollingBuffer, 7> buffers;
        static std::array<curan::renderable::ScrollingBuffer, 7> buffers_tau;
        static std::array<curan::renderable::ScrollingBuffer, 7> deriv;

        ImGui::Checkbox("stop time", &stop_time);
    
    
        static float history = 10.0f;
        ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");
    
        static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;
    
        if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
        {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
            for (size_t index = 0; index < buffers.size(); ++index)
            {
                std::string loc = "pose_" + std::to_string(index);
                //if(!stop_time)
                //    buffers[index].AddPoint(t, (float)state.user_defined2[index]);
                //ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
    
                loc = "cmdtau_" + std::to_string(index);
                if(!stop_time)
                    deriv[index].AddPoint(t, (float)state.cmd_tau[index]);
                ImPlot::PlotLine(loc.data(), &deriv[index].Data[0].x, &deriv[index].Data[0].y, deriv[index].Data.size(), 0, deriv[index].Offset, 2 * sizeof(float));

                loc = "accel_" + std::to_string(index);
                if(!stop_time)
                    buffers_tau[index].AddPoint(t, (float)state.ddq[index]);
                ImPlot::PlotLine(loc.data(), &buffers_tau[index].Data[0].x, &buffers_tau[index].Data[0].y, buffers_tau[index].Data.size(), 0, buffers_tau[index].Offset, 2 * sizeof(float));


            }
    
            ImPlot::EndPlot();
        }
    
        if(!stop_time)
            t += ImGui::GetIO().DeltaTime;
    
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }    

    ++counter;
}

struct ViewFiltering : public curan::robotic::UserData
{
    
    std::array<curan::robotic::ripple::Data, curan::robotic::number_of_joints> joint_data_first_harmonic;
    std::array<curan::robotic::ripple::Data, curan::robotic::number_of_joints> joint_data_second_harmonic;
    std::array<curan::robotic::ripple::Data, curan::robotic::number_of_joints> joint_data_third_harmonic;
    std::array<curan::robotic::ripple::Properties, curan::robotic::number_of_joints> first_harmonic;
    std::array<curan::robotic::ripple::Properties, curan::robotic::number_of_joints> second_harmonic;
    std::array<curan::robotic::ripple::Properties, curan::robotic::number_of_joints> third_harmonic;
    
    curan::robotic::ripple::Damper damper;


    std::array<curan::robotic::ripple::Data, curan::robotic::number_of_joints> deriv_joint_data_first_harmonic;
    std::array<curan::robotic::ripple::Data, curan::robotic::number_of_joints> deriv_joint_data_second_harmonic;
    std::array<curan::robotic::ripple::Data, curan::robotic::number_of_joints> deriv_joint_data_third_harmonic;
    std::array<curan::robotic::ripple::Properties, curan::robotic::number_of_joints> deriv_first_harmonic;
    std::array<curan::robotic::ripple::Properties, curan::robotic::number_of_joints> deriv_second_harmonic;
    std::array<curan::robotic::ripple::Properties, curan::robotic::number_of_joints> deriv_third_harmonic;

    curan::robotic::LowPassDerivativeFilter<curan::robotic::number_of_joints> filtering_mechanism;

    Eigen::Matrix<double,curan::robotic::number_of_joints,curan::robotic::number_of_joints> KT;
    Eigen::Matrix<double,curan::robotic::number_of_joints,curan::robotic::number_of_joints> KS;

    bool is_first_loop = true;

    ViewFiltering()
    {
        for (size_t filter_index = 0; filter_index < curan::robotic::number_of_joints; ++filter_index)
        {
            first_harmonic[filter_index].frequency = (filter_index == 4) ? 320.0 : 320.0;
            first_harmonic[filter_index].width = 5.0;
            second_harmonic[filter_index].frequency = (filter_index == 4) ? 640.0 : 640.0;
            second_harmonic[filter_index].width = 5.0;
            third_harmonic[filter_index].frequency = (filter_index == 4) ? 1280.0 : 1280.0;
            third_harmonic[filter_index].width = 5.0;

            deriv_first_harmonic[filter_index].frequency = (filter_index == 4) ? 320.0 : 320.0;
            deriv_first_harmonic[filter_index].width = 5.0;
            deriv_second_harmonic[filter_index].frequency = (filter_index == 4) ? 640.0 : 640.0;
            deriv_second_harmonic[filter_index].width = 5.0;
            deriv_third_harmonic[filter_index].frequency = (filter_index == 4) ? 1280.0 : 1280.0;
            deriv_third_harmonic[filter_index].width = 5.0;
        }

        KT = Eigen::Matrix<double,curan::robotic::number_of_joints,curan::robotic::number_of_joints>::Zero();
        /*
        KT(0,0) = 0.8;
        KT(1,1) = 0.8;
        KT(2,2) = 0.8;
        KT(3,3) = 0.2;
        KT(4,4) = 0.2;
        KT(5,5) = 0.2;
        KT(6,6) = 0.2;
        */
        /*
        KT(0,0) = 0.8;
        KT(1,1) = 0.8;
        KT(2,2) = 0.8;
        KT(3,3) = 0.8;
        KT(4,4) = 0.8;
        KT(5,5) = 0.8;
        KT(6,6) = 0.8;
        */
        //KT *= 0.0;

        KS = Eigen::Matrix<double,curan::robotic::number_of_joints,curan::robotic::number_of_joints>::Zero();
        /*
        KS(0,0) = 0.0001;
        KS(1,1) = 0.0001;
        KS(2,2) = 0.0001;
        KS(3,3) = 0.00005;
        KS(4,4) = 0.00005;
        KS(5,5) = 0.00005;
        KS(6,6) = 0.00005;
        KS *= 1.0;
        */
    }

    virtual ~ViewFiltering(){};

    curan::robotic::EigenState &&update(const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &&state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians) override
    {
        using vector_type = curan::robotic::RobotModel<curan::robotic::number_of_joints>::vector_type;
        static double currentTime = 0.0;
        
        vector_type raw_filtered_torque = vector_type::Zero();
        vector_type raw_deriv_filtered_torque = vector_type::Zero();

        static vector_type init_q = iiwa.joints();
        static vector_type prev_tau = iiwa.measured_torque();

        damper.compute<curan::robotic::number_of_joints>(iiwa.velocities(),iiwa.sample_time());

        for (size_t j = 0; j < curan::robotic::number_of_joints; ++j)
        {   
            
            double filtered_torque_value = iiwa.measured_torque()[j];
            curan::robotic::ripple::shift_filter_data(joint_data_first_harmonic[j]);
            curan::robotic::ripple::update_filter_data(joint_data_first_harmonic[j], filtered_torque_value);
            auto [filtered_harm_1,partial_derivative_1] = curan::robotic::ripple::execute(first_harmonic[j],damper, joint_data_first_harmonic[j]);
            filtered_torque_value -= filtered_harm_1;
            curan::robotic::ripple::shift_filter_data(joint_data_second_harmonic[j]);
            curan::robotic::ripple::update_filter_data(joint_data_second_harmonic[j], filtered_torque_value);
            auto [filtered_harm_2,partial_derivative_2]= curan::robotic::ripple::execute(second_harmonic[j],damper, joint_data_second_harmonic[j]);
            filtered_torque_value -= filtered_harm_2;
            curan::robotic::ripple::shift_filter_data(joint_data_third_harmonic[j]);
            curan::robotic::ripple::update_filter_data(joint_data_third_harmonic[j], filtered_torque_value);
            auto [ filtered_harm_3,partial_derivative_3] = curan::robotic::ripple::execute(third_harmonic[j],damper, joint_data_third_harmonic[j]);
            filtered_torque_value -= filtered_harm_3;
            raw_filtered_torque[j] = -filtered_torque_value - iiwa.gravity()[j] - iiwa.coriolis()[j];
            
            double deriv_filtered_torque_value = (iiwa.measured_torque()[j]-prev_tau[j])/iiwa.sample_time();
            curan::robotic::ripple::shift_filter_data(deriv_joint_data_first_harmonic[j]);
            curan::robotic::ripple::update_filter_data(deriv_joint_data_first_harmonic[j], deriv_filtered_torque_value);
            auto [deriv_filtered_harm_1,deriv_partial_derivative_1] = curan::robotic::ripple::execute(deriv_first_harmonic[j],damper, deriv_joint_data_first_harmonic[j]);
            deriv_filtered_torque_value -= deriv_filtered_harm_1;
            curan::robotic::ripple::shift_filter_data(deriv_joint_data_second_harmonic[j]);
            curan::robotic::ripple::update_filter_data(deriv_joint_data_second_harmonic[j], deriv_filtered_torque_value);
            auto [deriv_filtered_harm_2,deriv_partial_derivative_2]= curan::robotic::ripple::execute(deriv_second_harmonic[j],damper, deriv_joint_data_second_harmonic[j]);
            deriv_filtered_torque_value -= deriv_filtered_harm_2;
            curan::robotic::ripple::shift_filter_data(deriv_joint_data_third_harmonic[j]);
            curan::robotic::ripple::update_filter_data(deriv_joint_data_third_harmonic[j], deriv_filtered_torque_value);
            auto [ deriv_filtered_harm_3,deriv_partial_derivative_3] = curan::robotic::ripple::execute(deriv_third_harmonic[j],damper, deriv_joint_data_third_harmonic[j]);
            deriv_filtered_torque_value -= deriv_filtered_harm_3;
            raw_deriv_filtered_torque[j] = -deriv_filtered_torque_value;

        }

        static vector_type prev_filtered_torque = raw_filtered_torque;
        const auto& deriv_filtered_torque = filtering_mechanism.update(raw_deriv_filtered_torque,iiwa.sample_time());

        vector_type reference = init_q;
        vector_type impedance_controller = 100.0*(reference-iiwa.joints())-10*iiwa.velocities();
        vector_type actuation = impedance_controller+ KT* (impedance_controller - raw_filtered_torque);// - KS * raw_deriv_filtered_torque;

        state.cmd_q = iiwa.joints() + Eigen::Matrix<double, curan::robotic::number_of_joints, 1>::Constant(0.25 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
        state.cmd_tau = vector_type::Zero();

        currentTime += iiwa.sample_time();
        state.user_defined2 = actuation;
        state.user_defined3 = iiwa.gravity() + iiwa.coriolis();
        prev_tau = iiwa.measured_torque();
        prev_filtered_torque = raw_filtered_torque;
        for (size_t index = 0; index < curan::robotic::number_of_joints; ++index){
            if (state.cmd_tau[index] > 30.0)
                state.cmd_tau[index] = 30.0;
            if (state.cmd_tau[index] < -30.0)
                state.cmd_tau[index] = -30.0;
        }
        return std::move(state);
    }
};

int main()
{
    constexpr bool store = true;
    std::string file_appendix = "/freehand_";
    std::list<curan::robotic::State> recording_of_states;
    std::mutex mut;
    std::signal(SIGINT, signal_handler);
    {

        std::unique_ptr<ViewFiltering> handguinding_controller = std::make_unique<ViewFiltering>();
        curan::robotic::RobotLBR client{handguinding_controller.get(), 
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

        auto thread_pool = curan::utilities::ThreadPool::create(3);
        thread_pool->submit(curan::utilities::Job{"running controller", [&]()
                                                  {
                                                      try
                                                      {
                                                          curan::utilities::print<curan::utilities::info>("Lauching robot control thread\n");
                                                          KUKA::FRI::UdpConnection connection{20};
                                                          KUKA::FRI::ClientApplication app(connection, client);
                                                          bool success = app.connect(DEFAULT_PORTID, NULL);
													      if(success) curan::utilities::print<curan::utilities::info>("Connected successefully\n");
													      else curan::utilities::print<curan::utilities::info>("Failure to connect\n");
                                                          success = app.step();
                                                          while (client)
                                                          {
                                                              success = app.step();
                                                              if(store){
                                                                std::lock_guard<std::mutex> g{mut};
                                                                recording_of_states.push_back(client.atomic_acess().load());
                                                              }

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
        while(window.run_once()){
            if(!client)
                break;
        }
        client.cancel();
    }
    if(store){
        auto now = std::chrono::system_clock::now();
        auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        std::string filename{CURAN_COPIED_RESOURCE_PATH + file_appendix + std::to_string(UTC) + ".json"}; 
        std::cout << "creating filename with measurments :" << filename << std::endl;
        std::ofstream o(filename);
        o << recording_of_states;
    }
    return 0;
}
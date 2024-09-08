#include "robotutils/LBRController.h"
#include "robotutils/RippleReinjection.h"
#include "utils/Logger.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <chrono>
#include <csignal>

void custom_interface(vsg::CommandBuffer &cb,curan::robotic::RobotLBR& client,curan::robotic::RippleReinjection* reinjection)
{
	static size_t counter = 0;
    static const auto& atomic_access = client.atomic_acess();
    auto state = atomic_access.load(std::memory_order_relaxed);
	ImGui::Begin("Torques"); // Create a window called "Hello, world!" and append into it.
	static std::array<curan::renderable::ScrollingBuffer, 2> buffers;
    static std::array<curan::renderable::ScrollingBuffer, 2> filtered_buffers;
	static std::array<curan::renderable::ScrollingBuffer, 2> actuation_buffers;
	static std::array<curan::renderable::ScrollingBuffer, 2> reference;

	static float t = 0;

	t += ImGui::GetIO().DeltaTime;

	static float history = 10.0f;
	ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

	static bool local_record_data = true;
   	ImGui::Checkbox("Activate Filtering ", &local_record_data); 
   	reinjection->compensation_active.store(local_record_data);
        
    const char* items[] = {"None","Adaptive","Classic"};
    static const char* current_item = items[1];

    //static bool local_record_data = false;
    //ImGui::Checkbox("New Box Selection", &local_record_data); 
    //reinjection->compensation_active.store(local_record_data,std::memory_order_relaxed);

	/*
    if (ImGui::BeginCombo("##combo", current_item)) // The second parameter is the label previewed before opening the combo.
    {
    	for (int n = 0; n < IM_ARRAYSIZE(items); n++)
        {   
            bool is_selected = (current_item == items[n]); // You can store your selection however you want, outside or inside your objects
            if (ImGui::Selectable(items[n], is_selected))
                current_item = items[n];
            if (is_selected){
                ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for keyboard navigation support)
				if(client.view_userdata()){
					curan::robotic::CompensateRipple* ripple_compensation = dynamic_cast<curan::robotic::CompensateRipple*>(client.view_userdata());
					switch(n){
						case 0:
						ripple_compensation->activate.store(curan::robotic::NO_FILTERING_SCHEME,std::memory_order_acq_rel);
						break;
						case 1:
						ripple_compensation->activate.store(curan::robotic::ADAPTIVE_FILTERING_SCHEME,std::memory_order_acq_rel);
						break;
						case 2:
						ripple_compensation->activate.store(curan::robotic::CLASSIC_APPROACH,std::memory_order_acq_rel);
					}
				}
			}
		}
        ImGui::EndCombo();
    }*/

	static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

	if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
	{
		ImPlot::SetupAxes(NULL, NULL, flags, flags);
		ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
		ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
		size_t index = 0;
		std::string loc = "tau " + std::to_string(index);
		buffers[index].AddPoint(t, (float)state.tau[4]);
		ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		loc = "filtered " + std::to_string(index);
		filtered_buffers[index].AddPoint(t, (float)state.user_defined[4]);
		ImPlot::PlotLine(loc.data(), &filtered_buffers[index].Data[0].x, &filtered_buffers[index].Data[0].y, filtered_buffers[index].Data.size(), 0, filtered_buffers[index].Offset, 2 * sizeof(float));
		loc = "acuation " + std::to_string(index);
		actuation_buffers[index].AddPoint(t, (float)state.cmd_tau[4]);
		ImPlot::PlotLine(loc.data(), &actuation_buffers[index].Data[0].x, &actuation_buffers[index].Data[0].y, actuation_buffers[index].Data.size(), 0, actuation_buffers[index].Offset, 2 * sizeof(float));
		/*
		loc = "prop "+ std::to_string(index);
		proportional_buffers[index].AddPoint(t, (float)state.user_defined4[2]);
		ImPlot::PlotLine(loc.data(), &proportional_buffers[index].Data[0].x, &proportional_buffers[index].Data[0].y, proportional_buffers[index].Data.size(), 0, proportional_buffers[index].Offset, 2 * sizeof(float));
		loc = "deriv "+ std::to_string(index);
		derivative_buffers[index].AddPoint(t, (float)state.user_defined4[3]);
		ImPlot::PlotLine(loc.data(), &derivative_buffers[index].Data[0].x, &derivative_buffers[index].Data[0].y, derivative_buffers[index].Data.size(), 0, derivative_buffers[index].Offset, 2 * sizeof(float));
		*/
		loc = "ref "+ std::to_string(index);
		reference[index].AddPoint(t, (float)state.user_defined4[4]);
		ImPlot::PlotLine(loc.data(), &reference[index].Data[0].x, &reference[index].Data[0].y, reference[index].Data.size(), 0, reference[index].Offset, 2 * sizeof(float));


		index = 1;
		loc = "tau " + std::to_string(index);
		buffers[index].AddPoint(t, (float)state.tau[6]);
		ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		loc = "filtered " + std::to_string(index);
		filtered_buffers[index].AddPoint(t, (float)state.user_defined[6]);
		ImPlot::PlotLine(loc.data(), &filtered_buffers[index].Data[0].x, &filtered_buffers[index].Data[0].y, filtered_buffers[index].Data.size(), 0, filtered_buffers[index].Offset, 2 * sizeof(float));
		loc = "acuation " + std::to_string(index);
		actuation_buffers[index].AddPoint(t, (float)state.cmd_tau[6]);
		ImPlot::PlotLine(loc.data(), &actuation_buffers[index].Data[0].x, &actuation_buffers[index].Data[0].y, actuation_buffers[index].Data.size(), 0, actuation_buffers[index].Offset, 2 * sizeof(float));
		/*
		loc = "prop "+ std::to_string(index);
		proportional_buffers[index].AddPoint(t, (float)state.user_defined4[0]);
		ImPlot::PlotLine(loc.data(), &proportional_buffers[index].Data[0].x, &proportional_buffers[index].Data[0].y, proportional_buffers[index].Data.size(), 0, proportional_buffers[index].Offset, 2 * sizeof(float));
		loc = "deriv "+ std::to_string(index);
		derivative_buffers[index].AddPoint(t, (float)state.user_defined4[1]);
		ImPlot::PlotLine(loc.data(), &derivative_buffers[index].Data[0].x, &derivative_buffers[index].Data[0].y, derivative_buffers[index].Data.size(), 0, derivative_buffers[index].Offset, 2 * sizeof(float));
		*/
		loc = "ref "+ std::to_string(index);
		reference[index].AddPoint(t, (float)state.user_defined4[6]);
		ImPlot::PlotLine(loc.data(), &reference[index].Data[0].x, &reference[index].Data[0].y, reference[index].Data.size(), 0, reference[index].Offset, 2 * sizeof(float));


		ImPlot::EndPlot();
	}
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();

	++counter;
}

curan::robotic::RobotLBR* robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal){
	if(robot_pointer)
        robot_pointer->cancel();
}

void rendering(curan::robotic::RobotLBR& client,curan::robotic::RippleReinjection* reinjection){

    auto interface_callable = [&](vsg::CommandBuffer &cb){
        custom_interface(cb,client,reinjection);
    };

    curan::renderable::Window::Info info;
	curan::renderable::ImGUIInterface::Info info_gui{interface_callable};
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

	std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
	curan::renderable::SequencialLinks::Info create_info;
	create_info.convetion = vsg::CoordinateConvention::Y_UP;
	create_info.json_path = robot_path;
	create_info.number_of_links = 8;
	auto robot = curan::renderable::SequencialLinks::make(create_info);
	window << robot;

    const auto& atomic_state = client.atomic_acess();

    while(client && window.run_once()){
        auto state = atomic_state.load(std::memory_order_relaxed);
        for (size_t joint_index = 0; joint_index < curan::robotic::number_of_joints; ++joint_index)
			robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, state.q[joint_index]);
    }
}

int main(){
	std::signal(SIGINT, signal_handler);
    std::unique_ptr<curan::robotic::RippleReinjection> handguinding_controller = std::make_unique<curan::robotic::RippleReinjection>();
    auto reinjection_pointer = handguinding_controller.get();
	constexpr size_t n_weights = 0;
	curan::robotic::RobotLBR client{handguinding_controller.get(),"C:/Dev/Curan/resources/models/lbrmed/robot_mass_data.json","C:/Dev/Curan/resources/models/lbrmed/robot_kinematic_limits.json"};
	robot_pointer = &client;
	const auto& access_point = client.atomic_acess();
    std::thread robot_renderer{[&](){rendering(client,reinjection_pointer);}};
	//std::list<curan::robotic::State> list_of_recorded_states;
	try
	{
		curan::utilities::cout << "Lauching robot control thread\n";
		
		KUKA::FRI::UdpConnection connection{20};
		KUKA::FRI::ClientApplication app(connection, client);
		bool success = app.connect(DEFAULT_PORTID, NULL);
		success = app.step();
		while (client){
			success = app.step();
			//list_of_recorded_states.push_back(access_point.load());
		}
		app.disconnect();
        robot_renderer.join();
		/*
		auto now = std::chrono::system_clock::now();
		auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
		//std::string filename{CURAN_COPIED_RESOURCE_PATH"/d_m0s0001_p_0s5.json"};
        std::string filename{CURAN_COPIED_RESOURCE_PATH"/d_m0s00005_p_0s5.json"};
		std::cout << "creating filename with measurments :" << filename << std::endl;
		std::ofstream o(filename);
		o << list_of_recorded_states;
        */
		return 0;
	}
	catch (...)
	{
		std::cout << "robot control exception\n";
		return 1;
	}
}
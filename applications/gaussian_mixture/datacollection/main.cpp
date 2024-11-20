#include "robotutils/LBRController.h"
#include "robotutils/HandGuidance.h"
#include "utils/Logger.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <chrono>
#include <csignal>


void common_interface(vsg::CommandBuffer& cb, curan::robotic::RobotLBR& client,std::list<std::list<curan::robotic::State>>& demonstrations)
{
    static const auto& atomic_access = client.atomic_acess();
    auto state = atomic_access.load(std::memory_order_relaxed);
    ImGui::Begin("Joint Angles"); // Create a window called "Hello, world!" and append into it.
	static std::array<curan::renderable::ScrollingBuffer,curan::robotic::number_of_joints> buffers;
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;

	static std::list<curan::robotic::State> list_of_recorded_states;
	
	static bool local_record_data = false;
	static bool previous_recording_state = local_record_data;
	ImGui::Checkbox("Active Data Collection", &local_record_data); // Edit bools storing our window open/close state
	if(previous_recording_state!=local_record_data){
		if(local_record_data){
			list_of_recorded_states.push_back(state);
		} else {
			demonstrations.push_back(list_of_recorded_states);
			list_of_recorded_states = std::list<curan::robotic::State>{};
		}
	} else if(local_record_data){
		list_of_recorded_states.push_back(state);
	}
	previous_recording_state = local_record_data;

    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");

    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1,150))) {
        ImPlot::SetupAxes(NULL, NULL, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		for(size_t index = 0; index < curan::robotic::number_of_joints ; ++index){
			std::string loc = "joint "+std::to_string(index);
            buffers[index].AddPoint(t,(float)state.q[index]);
			ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		}
        ImPlot::EndPlot();
    }
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
}


curan::robotic::RobotLBR* robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal){
	if(robot_pointer)
        robot_pointer->cancel();
}

void rendering(curan::robotic::RobotLBR& client,std::list<std::list<curan::robotic::State>>& demonstrations){

    auto interface_callable = [&](vsg::CommandBuffer &cb){
        common_interface(cb,client,demonstrations);
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

	std::raise(SIGINT);

}

int main(int argc, char* argv[]) {
	std::signal(SIGINT, signal_handler);

	std::list<std::list<curan::robotic::State>> demonstrations;

    std::unique_ptr<curan::robotic::HandGuidance> handguinding_controller = std::make_unique<curan::robotic::HandGuidance>();
    curan::robotic::RobotLBR client{handguinding_controller.get(),
					CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json",
					CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};

	robot_pointer = &client;
	const auto& access_point = client.atomic_acess();
    std::thread robot_renderer{[&](){rendering(client,demonstrations);}};

	try
	{
		curan::utilities::cout << "Lauching robot control thread\n";
		
		KUKA::FRI::UdpConnection connection{20};
		KUKA::FRI::ClientApplication app(connection, client);
		app.connect(DEFAULT_PORTID, NULL);
		while (client)
			app.step();

		app.disconnect();
        robot_renderer.join();
		auto now = std::chrono::system_clock::now();
		auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
		size_t index = 0;
		for(const auto& demonstration : demonstrations){
			std::string filename{CURAN_COPIED_RESOURCE_PATH"/measurments"+std::to_string(index)+"_"+std::to_string(UTC)+".json"};
			std::cout << "creating filename with measurments :" << filename << std::endl;
			std::ofstream o(filename);
			o << demonstration;
			++index;
		}
		return 0;
	}
	catch (...)
	{
		std::cout << "robot control exception\n";
		return 1;
	}
}

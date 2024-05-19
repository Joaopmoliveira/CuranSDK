#include "robotutils/LBRController.h"
#include "robotutils/HandGuidance.h"
#include "robotutils/FilterRippleFirstHarmonic.h"
#include "utils/Logger.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <chrono>
#include <csignal>


// utility structure for realtime plot
struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};

std::atomic<bool> record_data = false;

void common_interface(vsg::CommandBuffer& cb, curan::robotic::RobotLBR& client)
{
    static const auto& atomic_access = client.atomic_acess();
    auto state = atomic_access.load(std::memory_order_relaxed);
    ImGui::Begin("Joint Angles"); // Create a window called "Hello, world!" and append into it.
	static std::array<ScrollingBuffer,LBR_N_JOINTS> buffers;
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;

	static bool local_record_data = false;
	ImGui::Checkbox("Active Data Collection", &local_record_data); // Edit bools storing our window open/close state
    record_data.store(local_record_data);

    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");

    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1,150))) {
        ImPlot::SetupAxes(NULL, NULL, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		for(size_t index = 0; index < LBR_N_JOINTS ; ++index){
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

void rendering(curan::robotic::RobotLBR& client){

    auto interface_callable = [&](vsg::CommandBuffer &cb){
        common_interface(cb,client);
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
        for (size_t joint_index = 0; joint_index < LBR_N_JOINTS; ++joint_index)
			robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, state.q[joint_index]);
    }

	std::raise(SIGINT);

}

int main(int argc, char* argv[]) {
	std::signal(SIGINT, signal_handler);
    std::unique_ptr<curan::robotic::HandGuidance> handguinding_controller = std::make_unique<curan::robotic::HandGuidance>();
    curan::robotic::RobotLBR client{handguinding_controller.get()};
	robot_pointer = &client;
	const auto& access_point = client.atomic_acess();
    std::thread robot_renderer{[&](){rendering(client);}};
	std::list<curan::robotic::State> list_of_recorded_states;
	std::list<std::list<curan::robotic::State>> demonstrations;
	try
	{
		curan::utilities::cout << "Lauching robot control thread\n";
		
		KUKA::FRI::UdpConnection connection{20};
		KUKA::FRI::ClientApplication app(connection, client);
		bool success = app.connect(DEFAULT_PORTID, NULL);
		success = app.step();
		
		size_t current_index_timestep = 0;
		while (client){
			success = app.step();
			auto snapshot_record_data = record_data.load();
			static bool previous_state = snapshot_record_data;
			bool state_changed = snapshot_record_data!=previous_state;
			
			if( current_index_timestep % 15 == 0 && snapshot_record_data)
				list_of_recorded_states.push_back(access_point.load());
			

			if(state_changed){
				previous_state = snapshot_record_data;
				if(snapshot_record_data){
					demonstrations.emplace_back(list_of_recorded_states);				
					std::printf("new %lu size\n",list_of_recorded_states.size());
					list_of_recorded_states = std::list<curan::robotic::State>{};
				}
			}
			++current_index_timestep;
		}
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

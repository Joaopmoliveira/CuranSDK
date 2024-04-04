#include "robotutils/LBRController.h"
#include "robotutils/JointVelocityController.h"
#include "utils/Logger.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <csignal>

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

void custom_interface(vsg::CommandBuffer &cb,curan::robotic::RobotLBR& client)
{
    static const auto& atomic_access = client.atomic_acess();
    auto state = atomic_access.load(std::memory_order_relaxed);
	ImGui::Begin("Control Torques"); // Create a window called "Hello, world!" and append into it.
	static std::array<ScrollingBuffer, LBR_N_JOINTS> buffers;
	static float t = 0;
	t += ImGui::GetIO().DeltaTime;

	static float history = 10.0f;
	ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

	static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

	if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
	{
		ImPlot::SetupAxes(NULL, NULL, flags, flags);
		ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
		ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
		for (size_t index = 0; index < LBR_N_JOINTS; ++index)
		{
			std::string loc = "cmd" + std::to_string(index);
			buffers[index].AddPoint(t, (float)state.cmd_tau[index]);
			ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		}
		ImPlot::EndPlot();
	}
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();

    static std::array<ScrollingBuffer, LBR_N_JOINTS> dqref;
    ImGui::Begin("Velocities"); // Create a window called "Hello, world!" and append into it.
    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
	{
		ImPlot::SetupAxes(NULL, NULL, flags, flags);
		ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
		ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
		for (size_t index = 0; index < LBR_N_JOINTS; ++index)
		{
			std::string loc = "dqref" + std::to_string(index);
			dqref[index].AddPoint(t, (float)state.user_defined[index]);
			ImPlot::PlotLine(loc.data(), &dqref[index].Data[0].x, &dqref[index].Data[0].y, dqref[index].Data.size(), 0, dqref[index].Offset, 2 * sizeof(float));
		}
		ImPlot::EndPlot();
	}
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
        custom_interface(cb,client);
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

    client.cancel();
}

int main(){
    std::unique_ptr<curan::robotic::JointVelocityController> handguinding_controller = std::make_unique<curan::robotic::JointVelocityController>();
    curan::robotic::RobotLBR client{handguinding_controller.get()};
    std::thread robot_renderer{[&](){rendering(client);}};
    const auto& access_point = client.atomic_acess();
	try
	{
        std::list<curan::robotic::State> list_of_recorded_states;
		curan::utilities::cout << "Lauching robot control thread\n";
		
		KUKA::FRI::UdpConnection connection;
		KUKA::FRI::ClientApplication app(connection, client);
		bool success = app.connect(DEFAULT_PORTID, NULL);
		success = app.step();
		while (success && client){
			success = app.step();
            list_of_recorded_states.push_back(access_point.load());
        }
		app.disconnect();
        robot_renderer.join();
        auto now = std::chrono::system_clock::now();
		auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
		std::string filename{CURAN_COPIED_RESOURCE_PATH"/measurments"+std::to_string(UTC)+".json"};
		std::cout << "creating filename with measurments :" << filename << std::endl;
		std::ofstream o(filename);
		o << list_of_recorded_states;
		return 0;
	}
	catch (...)
	{
		std::cout << "robot control exception\n";
		return 1;
	}
}
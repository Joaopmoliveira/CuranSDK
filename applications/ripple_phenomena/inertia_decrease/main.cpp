#include "robotutils/LBRController.h"
#include "robotutils/InertiaReduction.h"
#include "utils/Logger.h"
#include "utils/TheadPool.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include <csignal>

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

struct ScrollingBuffer
{
	int MaxSize;
	int Offset;
	ImVector<ImVec2> Data;
	ScrollingBuffer(int max_size = 4000)
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

curan::robotic::RobotLBR* robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal){
	if(robot_pointer)
        robot_pointer->cancel();
}


void custom_interface(vsg::CommandBuffer &cb,curan::robotic::RobotLBR& client)
{
	static size_t counter = 0;
    static const auto& atomic_access = client.atomic_acess();
    auto state = atomic_access.load(std::memory_order_relaxed);
	ImGui::Begin("Torques"); // Create a window called "Hello, world!" and append into it.
	static std::array<ScrollingBuffer, 7> buffers;
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
		for(size_t index = 0; index< buffers.size(); ++index){
			std::string loc = "userdef" + std::to_string(index);
			buffers[index].AddPoint(t, (float)state.user_defined[index]);
			ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		}

		ImPlot::EndPlot();
	}
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();

	++counter;
}


int main(){
    std::unique_ptr<curan::robotic::InertiaReduction> handguinding_controller = std::make_unique<curan::robotic::InertiaReduction>();
    curan::robotic::RobotLBR client{handguinding_controller.get(),CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_mass_data.json",CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_kinematic_limits.json"};
	robot_pointer = &client;

	curan::renderable::Window::Info info;
	curan::renderable::ImGUIInterface::Info info_gui{[&](vsg::CommandBuffer &cb){custom_interface(cb,client);}};
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
	thread_pool->submit(curan::utilities::Job{"running controller",[&](){
		try
		{
			curan::utilities::cout << "Lauching robot control thread\n";
			KUKA::FRI::UdpConnection connection;
			KUKA::FRI::ClientApplication app(connection, client);
			bool success = app.connect(DEFAULT_PORTID, NULL);
			curan::utilities::cout << (success ? "Connected successefully\n" : "Failure to connect\n");
			success = app.step();
			while (success && client){
				success = app.step();
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

	window.run();
	client.cancel();
	return 0;
}
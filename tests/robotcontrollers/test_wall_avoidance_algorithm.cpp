#include "robotutils/LBRController.h"
#include "robotutils/WallAvoidanceData.h"
#include "utils/Logger.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <csignal>


void custom_interface(vsg::CommandBuffer &cb,curan::robotic::RobotLBR& client)
{
    static const auto& atomic_access = client.atomic_acess();
    auto state = atomic_access.load(std::memory_order_relaxed);
	ImGui::Begin("Control Torques"); // Create a window called "Hello, world!" and append into it.
	static std::array<curan::renderable::ScrollingBuffer, curan::robotic::number_of_joints> buffers;
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
		for (size_t index = 0; index < curan::robotic::number_of_joints; ++index)
		{
			std::string loc = "tau_ext " + std::to_string(index);
			buffers[index].AddPoint(t, (float)state.tau_ext[index]);
			ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		}
		ImPlot::EndPlot();
	}
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();

    const std::array<std::string,3> names{{"D","dotD","dotdotDMaxFinal"}};

    static std::array<curan::renderable::ScrollingBuffer, 3> wall_avoindance_buffers;
    ImGui::Begin("Wall Avoindance Algorithm"); // Create a window called "Hello, world!" and append into it.
    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
	{
		ImPlot::SetupAxes(NULL, NULL, flags, flags);
		ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
		ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
		for (size_t index = 0; index < 3; ++index)
		{
			std::string loc = names[index].data();
			wall_avoindance_buffers[index].AddPoint(t, (float)state.user_defined[index]);
			ImPlot::PlotLine(loc.data(), &wall_avoindance_buffers[index].Data[0].x, &wall_avoindance_buffers[index].Data[0].y, wall_avoindance_buffers[index].Data.size(), 0, wall_avoindance_buffers[index].Offset, 2 * sizeof(float));
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
        for (size_t joint_index = 0; joint_index < curan::robotic::number_of_joints; ++joint_index)
			robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, state.q[joint_index]);
    }

}

int main(){
    Eigen::Vector3d plane_point{{-0.658657 , 0.276677 , 0.404548}};
    Eigen::Vector3d plane_direction{{0.0 , 1.0 , 0.0}};
    std::unique_ptr<curan::robotic::WallAvoidanceData> handguinding_controller = std::make_unique<curan::robotic::WallAvoidanceData>(plane_point,plane_direction);
    curan::robotic::RobotLBR client{handguinding_controller.get(),"C:/Dev/Curan/resources/models/lbrmed/robot_mass_data.json","C:/Dev/Curan/resources/models/lbrmed/robot_kinematic_limits.json"};
    std::thread robot_renderer{[&](){rendering(client);}};
	try
	{
		curan::utilities::print<curan::utilities::info>("Lauching robot control thread\n");
		
		KUKA::FRI::UdpConnection connection;
		KUKA::FRI::ClientApplication app(connection, client);
		bool success = app.connect(DEFAULT_PORTID, NULL);
		success = app.step();
		while (success && client)
			success = app.step();
		app.disconnect();
        robot_renderer.join();
		return 0;
	}
	catch (...)
	{
		std::cout << "robot control exception\n";
		return 1;
	}
}
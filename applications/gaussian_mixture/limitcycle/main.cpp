#include "robotutils/LBRController.h"
#include "robotutils/CartersianVelocityController.h"
#include "utils/Logger.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <chrono>
#include <csignal>

void display_interface(vsg::CommandBuffer& cb, curan::robotic::RobotLBR& client)
{
    static const auto& atomic_access = client.atomic_acess();
    auto state = atomic_access.load(std::memory_order_relaxed);
    ImGui::Begin("Joint Angles"); // Create a window called "Hello, world!" and append into it.
	static std::array<curan::renderable::ScrollingBuffer,3> buffers;
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;

    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");

    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1,-1))) {
        ImPlot::SetupAxes(NULL, NULL, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		for(size_t index = 0; index < 3 ; ++index){
			std::string loc = "dot x"+std::to_string(index);
            buffers[index].AddPoint(t,(float)state.user_defined[index]);
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
        display_interface(cb,client);
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
    Eigen::Matrix<double, 3, 3> desired_rotation = Eigen::Matrix<double, 3, 3>::Identity();
	desired_rotation << -0.718163, -0.00186162, -0.695873,
		                -0.00329559, 0.999994, 0.000725931,
		                0.695868, 0.00281465, -0.718165;
	Eigen::Matrix<double, 3, 1> desired_translation = Eigen::Matrix<double, 3, 1>::Zero();
	desired_translation << -0.66809, -0.00112052, 0.443678;
	curan::robotic::Transformation equilibrium{desired_rotation, desired_translation};

	std::unique_ptr<curan::robotic::CartersianVelocityController> handguinding_controller = std::make_unique<curan::robotic::CartersianVelocityController>(equilibrium,
																										 std::initializer_list<double>({500.0, 500.0, 500.0, 10.0, 10.0, 10.0}),
																										 std::initializer_list<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
    curan::robotic::RobotLBR client{handguinding_controller.get(),
                                    CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_mass_data.json",
                                    CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_kinematic_limits.json"};
	robot_pointer = &client;
    std::thread robot_renderer{[&](){rendering(client);}};

	try
	{
		curan::utilities::print<curan::utilities::info>("Lauching robot control thread\n");
		
		KUKA::FRI::UdpConnection connection{20};
		KUKA::FRI::ClientApplication app(connection, client);
		app.connect(DEFAULT_PORTID, NULL);
		while (client){
			app.step();
		}
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

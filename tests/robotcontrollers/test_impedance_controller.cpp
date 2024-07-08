#include "robotutils/LBRController.h"
#include "robotutils/ImpedanceController.h"
#include "utils/Logger.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include <csignal>

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

curan::robotic::RobotLBR* robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal){
	if(robot_pointer)
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

void custom_interface(vsg::CommandBuffer &cb,curan::robotic::RobotLBR& client)
{
	static size_t counter = 0;
    static const auto& atomic_access = client.atomic_acess();
    auto state = atomic_access.load(std::memory_order_relaxed);
	ImGui::Begin("Torques"); // Create a window called "Hello, world!" and append into it.
	static std::array<ScrollingBuffer, curan::robotic::number_of_joints> buffers;
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
		for(size_t index = 0; index< curan::robotic::number_of_joints; ++index){
			std::string loc = "tau_cmd" + std::to_string(index);
			buffers[index].AddPoint(t, (float)state.cmd_tau[index]);
			ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		}

		ImPlot::EndPlot();
	}
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();

	++counter;
}

int main(){
	Eigen::Matrix<double,3,3> desired_rotation = Eigen::Matrix<double,3,3>::Identity();
	Eigen::Matrix<double,3,1> desired_translation = Eigen::Matrix<double,3,1>::Zero();
	curan::robotic::Transformation equilibrium{desired_rotation,desired_translation};
	
    std::unique_ptr<curan::robotic::ImpedanceController> handguinding_controller =  std::make_unique<curan::robotic::ImpedanceController>(equilibrium,std::initializer_list<double>({100.0,100.0,100.0,10.0,10.0,10.0}),std::initializer_list<double>({1.0,1.0,1.0,1.0,1.0,1.0}));
    curan::robotic::RobotLBR client{handguinding_controller.get(),CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_mass_data.json",CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_kinematic_limits.json"};
	robot_pointer = &client;
	try
	{
		curan::utilities::cout << "Lauching robot control thread\n";
		
		KUKA::FRI::UdpConnection connection;
		KUKA::FRI::ClientApplication app(connection, client);
		bool success = app.connect(DEFAULT_PORTID, NULL);
		success = app.step();
		while (success && client)
			success = app.step();
		app.disconnect();
		curan::utilities::cout << "Terminating robot control thread\n";
		return 0;
	}
	catch (...)
	{
		std::cout << "robot control exception\n";
		return 1;
	}
}
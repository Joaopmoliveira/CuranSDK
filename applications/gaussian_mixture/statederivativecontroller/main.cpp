#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <iostream>
#include <thread>
#include "utils/Logger.h"
#include "utils/Flag.h"
#include "utils/SafeQueue.h"

#include "MyLBRClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <csignal>

constexpr unsigned short DEFAULT_PORTID = 30200;

std::atomic<bool> progress = true;

void signal_handler(int signal)
{
	progress.store(false);
}

// utility structure for realtime plot
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

std::atomic<std::array<double, LBR_N_JOINTS>> robot_torques;

void interface(vsg::CommandBuffer &cb)
{
	ImGui::Begin("Joint Angles"); // Create a window called "Hello, world!" and append into it.
	static std::array<ScrollingBuffer, LBR_N_JOINTS> buffers;
	static float t = 0;
	t += ImGui::GetIO().DeltaTime;
	auto local_copy = robot_torques.load();

	static float history = 10.0f;
	ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

	static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

	if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, 150)))
	{
		ImPlot::SetupAxes(NULL, NULL, flags, flags);
		ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
		ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
		for (size_t index = 0; index < LBR_N_JOINTS; ++index)
		{
			std::string loc = "torque " + std::to_string(index);
			buffers[index].AddPoint(t, (float)local_copy[index]);
			ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		}
		ImPlot::EndPlot();
	}
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();
}

void robot_control(std::shared_ptr<SharedState> shared_state, curan::utilities::Flag& flag)
{
	try
	{
		curan::utilities::cout << "Lauching robot control thread\n";
		MyLBRClient client = MyLBRClient(shared_state);
		KUKA::FRI::UdpConnection connection{2};
		KUKA::FRI::ClientApplication app(connection, client);
		app.connect(DEFAULT_PORTID, NULL);
		bool success = true;
		while (success && flag.value())
			success = app.step();
		app.disconnect();
		return;
	}
	catch (...)
	{
		std::cout << "robot control exception\n";
		return;
	}
}

int main(int argc, char *argv[])
{
	// Install a signal handler
	std::signal(SIGINT, signal_handler);
	try
	{
		curan::utilities::Flag robot_flag;
		robot_flag.set(true);

		auto shared_state = std::make_shared<SharedState>();
		shared_state->is_initialized.store(false);

		auto robot_functional_control = [shared_state, &robot_flag]()
		{
			robot_control(shared_state, robot_flag);
		};

		std::thread thred_robot_control{robot_functional_control};

		curan::renderable::Window::Info info;
		curan::renderable::ImGUIInterface::Info info_gui{interface};
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

		// Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
		kuka::Robot::robotName myName(kuka::Robot::LBRiiwa); // Select the robot here

		auto robot_control = std::make_unique<kuka::Robot>(myName); // myLBR = Model
		auto iiwa = std::make_unique<RobotParameters>();			// myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

		Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric
														// Positions and orientations
		Vector3d p_0_cur = Vector3d::Zero(3, 1);
		Matrix3d R_0_7 = Matrix3d::Zero(3, 3);

		while (progress.load())
		{
			if (!window.run_once())
				progress = false;
			if (shared_state->is_initialized)
			{
				auto local_state = shared_state->robot_state.load();
				for (size_t joint_index = 0; joint_index < LBR_N_JOINTS; ++joint_index)
				{
					robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, local_state.getMeasuredJointPosition()[joint_index]);
					robot_torques.store(shared_state->joint_torques.load());
				}

				// Get robot measurements
				shared_state->is_initialized.store(true);
				static bool first_time = true;
				if (first_time){
					first_time = false;
					for (int i = 0; i < LBR_N_JOINTS; i++)
					{
						iiwa->q[i] = local_state.getMeasuredJointPosition()[i];
						iiwa->qDot[i] = 0.0;
					}
				} else {
					for (int i = 0; i < LBR_N_JOINTS; i++)
					{
						iiwa->qDot[i] = (local_state.getMeasuredJointPosition()[i] - iiwa->q[i]) / local_state.getSampleTime();
						iiwa->q[i] = local_state.getMeasuredJointPosition()[i];
					}
				}

				robot_control->getMassMatrix(iiwa->M, iiwa->q);
				iiwa->M(6, 6) = 45 * iiwa->M(6, 6); // Correct mass of last body to avoid large accelerations
				iiwa->Minv = iiwa->M.inverse();
				robot_control->getCoriolisAndGravityVector(iiwa->c, iiwa->g, iiwa->q, iiwa->qDot);
				robot_control->getWorldCoordinates(p_0_cur, iiwa->q, pointPosition, 7); // 3x1 position of flange (body = 7), expressed in base coordinates
				robot_control->getRotationMatrix(R_0_7, iiwa->q, LBR_N_JOINTS);		// 3x3 rotation matrix of flange, expressed in base coordinates
			}
		}
		robot_flag.set(false);
		thred_robot_control.join();
		return 0;
	}
	catch (std::exception &e){
		std::cout << "main Exception : " << e.what() << std::endl;
	}
}
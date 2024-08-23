#include "robotutils/LBRController.h"
#include "robotutils/SimulateModel.h"

#include "utils/Logger.h"
#include "utils/TheadPool.h"

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

void custom_interface(vsg::CommandBuffer &cb, curan::robotic::RobotLBR &client)
{
	static const auto &atomic_access = client.atomic_acess();
	auto state = atomic_access.load(std::memory_order_relaxed);
	ImGui::Begin("Control Torques"); // Create a window called "Hello, world!" and append into it.
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
		for (size_t index = 0; index < curan::robotic::number_of_joints; ++index)
		{
			std::string loc = "cmd" + std::to_string(index);
			buffers[index].AddPoint(t, (float)state.cmd_tau[index]);
			ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		}
		ImPlot::EndPlot();
	}
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();

	static std::array<ScrollingBuffer, curan::robotic::number_of_joints> dqref;
	ImGui::Begin("Velocities"); // Create a window called "Hello, world!" and append into it.
	if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
	{
		ImPlot::SetupAxes(NULL, NULL, flags, flags);
		ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
		ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
		for (size_t index = 0; index < curan::robotic::number_of_joints; ++index)
		{
			std::string loc = "dqref" + std::to_string(index);
			dqref[index].AddPoint(t, (float)state.user_defined[index]);
			ImPlot::PlotLine(loc.data(), &dqref[index].Data[0].x, &dqref[index].Data[0].y, dqref[index].Data.size(), 0, dqref[index].Offset, 2 * sizeof(float));
		}
		ImPlot::EndPlot();
	}
	ImGui::End();
}

curan::robotic::RobotLBR *robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal)
{
	if (robot_pointer)
		robot_pointer->cancel();
}

void rendering(curan::robotic::RobotLBR &client)
{

	auto interface_callable = [&](vsg::CommandBuffer &cb)
	{
		custom_interface(cb, client);
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

	const auto &atomic_state = client.atomic_acess();

	while (client && window.run_once())
	{
		auto state = atomic_state.load(std::memory_order_relaxed);
		for (size_t joint_index = 0; joint_index < curan::robotic::number_of_joints; ++joint_index)
			robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, state.q[joint_index]);
	}

	client.cancel();
}

bool free_hand_control(bool is_transitioning,const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians)
{	
		static double currentTime = 0.0;
        state.cmd_tau = -iiwa.mass() * iiwa.velocities();
        state.cmd_q = iiwa.joints() + Eigen::Matrix<double,curan::robotic::number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
		currentTime += iiwa.sample_time();
		return false;
}

bool position_hold_control(bool is_transitioning,const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians)
{
	static double currentTime = 0.0;

	state.cmd_q = iiwa.joints() + Eigen::Matrix<double,curan::robotic::number_of_joints,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
	currentTime += iiwa.sample_time();
	return false;
}

struct ControllerSwitcher : public curan::robotic::UserData
{

	enum ControlModes{
		POSITION_HOLD,
		FREE_HAND
	};

	ControlModes current_mode = ControlModes::FREE_HAND;

	ControllerSwitcher()
	{
	}

	curan::robotic::EigenState &&update(const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &&state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians) override
	{
		static ControlModes previous_mode = current_mode;
		static bool transitioning = false;

		switch (current_mode)
		{
		case POSITION_HOLD:
			if (previous_mode != current_mode)
			{
				transitioning = true;
				transitioning = position_hold_control(transitioning,iiwa, state, composed_task_jacobians);
			}
			else if (transitioning)
				transitioning = position_hold_control(transitioning, iiwa, state, composed_task_jacobians);
			else
				position_hold_control<false>(iiwa, state, composed_task_jacobians);
			break;
		case FREE_HAND:
			if (previous_mode != current_mode)
			{
				transitioning = true;
				transitioning = free_hand_control(transitioning, iiwa, state, composed_task_jacobians);
			}
			else if (transitioning)
				transitioning = free_hand_control(transitioning, iiwa, state, composed_task_jacobians);
			else
				transitioning = free_hand_control(transitioning, iiwa, state, composed_task_jacobians);
			break;
		default:
			throw std::runtime_error("selected a control mode that is unavailable");
			break;
		}
		
		previous_mode = current_mode;
	}

	void async_control_switch(){

	}
};

int main()
{
    using namespace curan::robotic;
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
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

    std::atomic<curan::robotic::State> atomic_state;
    std::atomic<bool> keep_running = true;
    auto pool = curan::utilities::ThreadPool::create(1);
    pool->submit(curan::utilities::Job{"value", [&]()
                                       {
                                           std::unique_ptr<ControllerSwitcher> handguinding_controller = std::make_unique<ControllerSwitcher>();

                                           curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
                                           constexpr auto sample_time = std::chrono::milliseconds(1);
                                           curan::robotic::State state;
                                           Eigen::Matrix<double, 7, 1> external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                           double time = 0.0;
                                           while (keep_running.load())
                                           {
                                               std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                                               if(time>10 && time < 15){
                                                    external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                                    //external_torque[6] = 10;
                                               } else if(time> 15){
                                                    time = 0.0;
                                                    external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                               } else {
                                                    external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                               }
                                               state = curan::robotic::simulate_next_timestamp(robot_model,handguinding_controller.get(),sample_time,state,external_torque);
                                               atomic_state.store(state,std::memory_order_relaxed);
                                               time += std::chrono::duration<double>(sample_time).count();
                                           }
                                       }});

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::list<curan::robotic::State> list_of_robot_states;
    while (window.run_once())
    {
        auto current_state = atomic_state.load();
        list_of_robot_states.push_back(current_state);
        for (size_t joint_index = 0; joint_index < curan::robotic::number_of_joints; ++joint_index)
            robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, current_state.q[joint_index]);
    }
    keep_running = false;
}
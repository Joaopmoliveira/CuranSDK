#include "robotutils/LBRController.h"
#include "robotutils/CartersianVelocityController.h"
#include "utils/Logger.h"
#include "utils/TheadPool.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include <csignal>

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

using AtomicState = std::atomic<curan::robotic::State>;

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

void custom_interface(vsg::CommandBuffer &cb, AtomicState& atomic_state)
{
	static size_t counter = 0;
	auto state = atomic_state.load(std::memory_order_relaxed);
	static float history = 10.0f;
	static float t = 0;
	t += ImGui::GetIO().DeltaTime;
	{
		ImGui::Begin("Forces"); // Create a window called "Hello, world!" and append into it.
		static std::array<ScrollingBuffer, 6> buffers;

		ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

		static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

		if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
		{
			ImPlot::SetupAxes(NULL, NULL, flags, flags);
			ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
			ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
			ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
			for (size_t index = 0; index < 6; ++index)
			{
				std::string loc = "userdef3" + std::to_string(index);
				buffers[index].AddPoint(t, (float)state.user_defined3[index]);
				ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
			}

			ImPlot::EndPlot();
		}
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();
	}

    {
		ImGui::Begin("EigenValues"); // Create a window called "Hello, world!" and append into it.
		static std::array<ScrollingBuffer, 6> buffers;

		ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

		static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

		if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
		{
			ImPlot::SetupAxes(NULL, NULL, flags, flags);
			ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
			ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
			ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
			for (size_t index = 0; index < 6; ++index)
			{
				std::string loc = "userdef3" + std::to_string(index);
				buffers[index].AddPoint(t, (float)state.user_defined4[index]);
				ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
			}

			ImPlot::EndPlot();
		}
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();
	}

	++counter;
}

int main()
{
    using namespace curan::robotic;
    AtomicState atomic_state;

	curan::renderable::Window::Info info;
	curan::renderable::ImGUIInterface::Info info_gui{[&](vsg::CommandBuffer &cb)
													 { custom_interface(cb, atomic_state); }};
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

    std::atomic<bool> keep_running = true;
    auto pool = curan::utilities::ThreadPool::create(1);
    pool->submit(curan::utilities::Job{"value", [&]()
                                       {
                                           Eigen::Matrix<double, 3, 1> desired_translation = Eigen::Matrix<double, 3, 1>::Zero();
                                           desired_translation << -0.66809, -0.00112052, 0.443678;
                                            Eigen::Matrix<double, 3, 3> desired_rotation = Eigen::Matrix<double, 3, 3>::Identity();
                                           desired_rotation << -0.718163, -0.00186162, -0.695873,
                                                                -0.00329559, 0.999994, 0.000725931,
                                                                0.695868, 0.00281465, -0.718165;
                                           std::unique_ptr<CartersianVelocityController> handguinding_controller = std::make_unique<CartersianVelocityController>([&](const RobotModel<number_of_joints>& iiwa){
                                                        Eigen::Matrix<double,3,1> translated_position = iiwa.translation()-desired_translation;
                                                        Eigen::Matrix<double,3,1> desired_translated_velocity_local_frame;
                                                        double radius = std::sqrt(translated_position[0]*translated_position[0] + translated_position[1]*translated_position[1]);
                                                        double angular_velocity = 0.3;
                                                        double radial_equilibrium = 0.1;
                                                        static double previous = radius;
                                                        static double filtered_radius_derivative = (radius-previous)/iiwa.sample_time();
                                                        filtered_radius_derivative = 0.8*filtered_radius_derivative+0.2*(radius-previous)/iiwa.sample_time();
                                                        previous = radius;
                                                        desired_translated_velocity_local_frame[2] = -translated_position[2];
                                                        double angle_theta = (radius < 0.001) ? 0.0 : std::atan2(translated_position[1],translated_position[0]); 
                                                        Eigen::Matrix<double,2,1> rotator = Eigen::Matrix<double,2,1>::Zero();
                                                        rotator << (radial_equilibrium-radius) , angular_velocity;
                                                        Eigen::Matrix<double,2,1> dot_rotator = Eigen::Matrix<double,2,1>::Zero();
                                                        dot_rotator << -filtered_radius_derivative , 0;
                                                        Eigen::Matrix<double,2,2> jacobian_of_limit_cycle = Eigen::Matrix<double,2,2>::Zero();
                                                        jacobian_of_limit_cycle << std::cos(angle_theta) , -radius*std::sin(angle_theta) , std::sin(angle_theta) , radius*std::cos(angle_theta);
                                                        Eigen::Matrix<double,2,2> dot_jacobian_of_limit_cycle = Eigen::Matrix<double,2,2>::Zero();
                                                        dot_jacobian_of_limit_cycle << -angular_velocity*std::sin(angle_theta) , -angular_velocity*radius*std::cos(angle_theta) , angular_velocity*std::cos(angle_theta) , -angular_velocity*radius*std::sin(angle_theta);
                                                        Eigen::Matrix<double,2,1> desired_acceleration = dot_jacobian_of_limit_cycle*rotator+jacobian_of_limit_cycle*dot_rotator;
                                                        desired_translated_velocity_local_frame.block<2,1>(0,0) = jacobian_of_limit_cycle*rotator;
                                                        Eigen::Matrix<double,6,1> desired_velocity;
                                                        desired_velocity.block<3, 1>(0, 0) = desired_translated_velocity_local_frame;
                                                        Eigen::AngleAxisd E_AxisAngle(iiwa.rotation().transpose() * desired_rotation);
                                                        desired_velocity.block<3, 1>(3, 0) = E_AxisAngle.angle() * iiwa.rotation() * E_AxisAngle.axis();
                                                        return desired_velocity;
                                                    },
														std::initializer_list<double>({100.0, 100.0, 100.0, 10.0, 10.0, 10.0}),
														std::initializer_list<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));

                                           curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
                                           double delta_time = 0.001;
                                           Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jacobian;
                                           Eigen::Matrix<double, 7, 1> dq = Eigen::Matrix<double, 7, 1>::Zero();
                                           Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
                                           curan::robotic::State state;
                                           state.sampleTime = delta_time;
                                           state.q = std::array<double, 7>{};
                                           curan::robotic::State next = state;
                                           double time = 0;
                                           Eigen::Matrix<double, 7, 1> external_torque = Eigen::Matrix<double, 7, 1>::Zero();

                                           while (keep_running.load())
                                           {
                                               std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                                               if (time > 10 && time < 15)
                                               {
                                                   external_torque = 0.0 * Eigen::Matrix<double, 7, 1>::Ones();
                                               }
                                               else if (time > 15)
                                               {
                                                   time = 0.0;
                                                   external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                               }
                                               else
                                               {
                                                   external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                               }

                                               state.differential(next);
                                               robot_model.update(state);
                                               auto actuation = handguinding_controller->update(robot_model, curan::robotic::EigenState{}, jacobian);
                                               state.convertFrom(actuation);
                                               Eigen::Matrix<double, 7, 1> ddq = robot_model.invmass() * (external_torque + actuation.cmd_tau);
                                               dq = ddq * delta_time + dq;
                                               q = dq * delta_time + q;
                                               next = state;
                                               next.q = curan::robotic::convert<double, 7>(q);
                                               atomic_state.store(state);
                                               std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                                               std::this_thread::sleep_for(std::chrono::milliseconds(1));
                                               end = std::chrono::steady_clock::now();
                                               time += 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
                                           }
                                       }});
    while (window.run_once())
    {
        auto current_state = atomic_state.load();
        for (size_t joint_index = 0; joint_index < curan::robotic::number_of_joints; ++joint_index)
            robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, current_state.q[joint_index]);
    }
    keep_running = false;
    return 0;
}
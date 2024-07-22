#include "robotutils/LBRController.h"
#include "robotutils/CartersianVelocityController.h"
#include "robotutils/SimulateModel.h"

#include "utils/Logger.h"
#include "utils/TheadPool.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include <csignal>

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include "gaussianmixtures/GMR.h"

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
		ImGui::Begin("Error Decay"); // Create a window called "Hello, world!" and append into it.
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
				std::string loc = "userdef" + std::to_string(index);
				buffers[index].AddPoint(t, (float)state.user_defined2[index]);
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

    curan::gaussian::GMR<2,2> model;
    std::fstream gmr_model_file{CURAN_COPIED_RESOURCE_PATH"/gaussianmixtures_testing/model.txt"};
    gmr_model_file >> model;

    std::atomic<bool> keep_running = true;
    auto pool = curan::utilities::ThreadPool::create(1);
    pool->submit(curan::utilities::Job{"value", [&]()
                                       {
                                            Eigen::Matrix<double, 3, 1> desired_translation = Eigen::Matrix<double, 3, 1>::Zero();
                                           desired_translation << -0.632239 , 0.0123415 , 0.337702;
                                          Eigen::Matrix<double, 3, 3> desired_rotation = Eigen::Matrix<double, 3, 3>::Identity();
                                           desired_rotation <<   -0.998815  ,  0.0486651 ,-0.000655218,
   																0.0486561   ,  0.998765   , 0.0100775,
  																0.00114483  ,  0.0100337  ,  -0.999949 ;

                                           std::unique_ptr<CartersianVelocityController> handguinding_controller = std::make_unique<CartersianVelocityController>([&](const RobotModel<number_of_joints>& iiwa){
                                                        Eigen::Matrix<double,6,1> desired_velocity = Eigen::Matrix<double,6,1>::Zero();
                                                        desired_velocity.block<2, 1>(0, 0) = 2*model.likeliest(iiwa.translation().block<2,1>(0,0));
                                                        desired_velocity[2] = desired_translation[2]-iiwa.translation()[2];
                                                        std::cout << desired_velocity.transpose() << std::endl;
                                                        Eigen::AngleAxisd E_AxisAngle(iiwa.rotation().transpose() * desired_rotation);
                                                        desired_velocity.block<3, 1>(3, 0) = E_AxisAngle.angle() * iiwa.rotation() * E_AxisAngle.axis();
                                                        return desired_velocity;
                                                    },
														std::initializer_list<double>({100.0, 100.0, 100.0, 30.0, 30.0, 30.0}),
														std::initializer_list<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));

                                           curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
                                           constexpr auto sample_time = std::chrono::milliseconds(1);
                                           curan::robotic::State state;
                                           double time = 0;
                                           Eigen::Matrix<double, 7, 1> external_torque = Eigen::Matrix<double, 7, 1>::Zero();

                                           while (keep_running.load())
                                           {
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

                                               state = curan::robotic::simulate_next_timestamp(robot_model,handguinding_controller.get(),sample_time,state,external_torque);
                                               atomic_state.store(state,std::memory_order_relaxed);
                                               time += std::chrono::duration<double>(sample_time).count();
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
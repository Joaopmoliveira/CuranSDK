#include "rendering/SequencialLinks.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Sphere.h"
#include <iostream>

constexpr size_t n_joints = 7;

std::atomic<std::array<double,n_joints>> robot_joint_config;

void interface(vsg::CommandBuffer& cb){
    ImGui::Begin("Angle Display"); // Create a window called "Hello, world!" and append into it.
    ImGui::BulletText("Move your mouse to change the data!");
    ImGui::BulletText("This example assumes 60 FPS. Higher FPS requires larger buffer size.");
	static std::array<curan::renderable::ScrollingBuffer,n_joints> buffers;
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;
	auto local_copy = robot_joint_config.load();
    
    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");

    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1,150))) {
        ImPlot::SetupAxes(NULL, NULL, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		for(size_t index = 0; index < n_joints ; ++index){
			std::string loc = "joint "+std::to_string(index);
            buffers[index].AddPoint(t,(float)local_copy[index]);
			ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		}
        ImPlot::EndPlot();
    }
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
}

int main(int argc, char **argv) {
    try {
        curan::renderable::ImGUIInterface::Info info_gui{interface};
        auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = true;
        info.screen_number = 0;
        info.title = "myviewer";
        info.imgui_interface = ui_interface;
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info};

        std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
        curan::renderable::SequencialLinks::Info create_info;
        create_info.convetion = vsg::CoordinateConvention::Y_UP;
        create_info.json_path = robot_path;
        create_info.number_of_links = 8;
        vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks::make(create_info);
        window << robotRenderable;

        std::atomic<bool> continue_updating = true;

        auto updater = [robotRenderable,&continue_updating](){
            double time = 0.0;
            std::array<double,n_joints> local_joint_config;
            while(continue_updating.load()){
                auto robot = robotRenderable->cast<curan::renderable::SequencialLinks>();
                for(size_t index = 0; index < n_joints ; ++index){
                    double angle = std::sin(time+index*0.04)*1.5;
                    robot->set(index,angle);
                    local_joint_config[index] = angle;
                }
                robot_joint_config.store(local_joint_config);
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
                time += 0.016;
            }
        };
        std::thread local_thread{updater};

        window.run();
        continue_updating.store(false);
        local_thread.join();

        window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable>>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }

            });

    } catch (const std::exception& e) {
        std::cerr << "Exception thrown : " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

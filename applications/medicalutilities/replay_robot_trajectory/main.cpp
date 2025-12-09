#include "rendering/Box.h"
#include "rendering/DynamicTexture.h"
#include "rendering/ImGUIInterface.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/Sphere.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "robotutils/RobotModel.h"
#include "utils/DateManipulation.h"
#include "utils/FileStructures.h"
#include "utils/Reader.h"
#include "utils/TheadPool.h"
#include <iostream>
#include <map>
#include <string>
#include <vsg/all.h>
#include <vsgXchange/all.h>

struct Parameters
{
    std::shared_ptr<curan::utilities::ThreadPool> pool = curan::utilities::ThreadPool::create(1);
    
    std::atomic<bool> replay_start = false;
    std::atomic<bool> replay_running = false;

    std::atomic<bool> singlejoint_start = false;
    std::atomic<bool> singlejoint_running = false;

    std::list<curan::robotic::State> measurments_to_add;

} params;

void plot_single_joint_data(){

}

void plot_joint_data(){
    ImGui::Begin("Joint Angles"); // Create a window called "Hello, world!" and append into it.
	static std::array<curan::renderable::ScrollingBuffer,curan::robotic::number_of_joints> buffers;
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;

}

void interface(vsg::CommandBuffer &cb)
{
    ImGui::Begin("Control Panel"); // Create a window called "Hello, world!" and

    if (!params.singlejoint_running && !params.singlejoint_running)
        if (ImGui::Button("Start Single Joint Motion")) // Buttons return true when clicked (most widgets
        {
            params.singlejoint_start = true;
            params.singlejoint_running = true;
        }

    if (!params.singlejoint_running && !params.singlejoint_running)
        if (ImGui::Button("Replay Joint Motion")) // Buttons return true when clicked (most widgets
        {
            params.replay_running = true;
            params.replay_start = true;
        }

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
}

int main(int argc, char **argv)
{
    try
    {

        std::ifstream file{"C:/Dev/FilterE/data_hipo/freehand_1743255401.json"};
        std::list<curan::robotic::State> recorded_values;
        file >> recorded_values;

        curan::renderable::ImGUIInterface::Info info_gui{interface};
        auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = true;
        info.screen_number = 0;
        info.imgui_interface = ui_interface;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info};

        std::filesystem::path robot_path =
            CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
        curan::renderable::SequencialLinks::Info create_info;
        create_info.convetion = vsg::CoordinateConvention::Y_UP;
        create_info.json_path = robot_path;
        create_info.number_of_links = 8;
        vsg::ref_ptr<curan::renderable::Renderable> robotRenderable =
            curan::renderable::SequencialLinks::make(create_info);
        window << robotRenderable;

        std::atomic<bool> continue_updating = true;

        while (window.run_once())
        {
            if(params.replay_start){
                params.replay_start = false;
                params.pool->submit("runner",[recorded_values, &continue_updating, robotRenderable](){
                    auto robot = robotRenderable->cast<curan::renderable::SequencialLinks>();
                    size_t index_current = 0;
                    for (auto abstract_ptr = recorded_values.begin();;) {
                        auto begin = std::chrono::high_resolution_clock::now();
                        if (!continue_updating){
                            params.replay_running = false;
                            return;
                        }
                        for (size_t index = 0; index < curan::robotic::number_of_joints;++index)
                            robot->set(index, abstract_ptr->q[index]);
                        std::this_thread::sleep_for(std::chrono::microseconds(500));
                        std::printf("frame: %d/%d\n", (int)index_current,(int)recorded_values.size());
                        auto end = std::chrono::high_resolution_clock::now();
                        double duration = std::round(1e-3 *std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());
                        std::advance(abstract_ptr, (int)duration);
                        index_current += (int)duration;
                        if (index_current >= recorded_values.size())
                            break;
                    } 
                    params.replay_running = false;
                });
            }
        }

        continue_updating.store(false);

        window.transverse_identifiers(
            [](const std::unordered_map<
                std::string, vsg::ref_ptr<curan::renderable::Renderable>> &map)
            {
                for (auto &p : map)
                {
                    std::cout << "Object contained: " << p.first << '\n';
                }
            });
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception thrown : " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

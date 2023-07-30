#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/ImGUIInterface.h"
#include <iostream>


struct Parameters{
    bool showGui = true; // you can toggle this with your own EventHandler and key
    bool showDemoWindow = false;
    bool showSecondWindow = false;
    bool showImPlotDemoWindow = false;
    bool showLogoWindow = true;
    bool showImagesWindow = false;
    float clearColor[3]{0.2f, 0.2f, 0.4f}; // Unfortunately, this doesn't change dynamically in vsg
    uint32_t counter = 0;
    float dist = 0.f;
} params;

void interface(vsg::CommandBuffer& cb){
    std::printf("ran once!");
    ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.

    ImGui::Text("Some useful message here.");                 // Display some text (you can use a format strings too)
    ImGui::Checkbox("Demo Window", &params.showDemoWindow); // Edit bools storing our window open/close state
    ImGui::Checkbox("Another Window", &params.showSecondWindow);
    ImGui::Checkbox("ImPlot Demo Window", &params.showImPlotDemoWindow);

    ImGui::SliderFloat("float", &params.dist, 0.0f, 1.0f);        // Edit 1 float using a slider from 0.0f to 1.0f
    ImGui::ColorEdit3("clear color", (float*)&params.clearColor); // Edit 3 floats representing a color

    if (ImGui::Button("Button")) // Buttons return true when clicked (most widgets return true when edited/activated)
        params.counter++;

    ImGui::SameLine();
    ImGui::Text("counter = %d", params.counter);

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
        info.is_debug = false;
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

        auto async_attacher = [&](){
            double angle = 0.0;
            double time = 0.0;
            while(continue_updating.load()){
                auto robot = robotRenderable->cast<curan::renderable::SequencialLinks>();
                for(size_t index = 0; index < 7 ; ++index)
                    robot->set(index,angle);
                angle = std::sin(time)*1.5;
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
                time += 0.016;
            }
        };
        std::thread local_thread_attacher{async_attacher};

        window.run();
        continue_updating.store(false);
        local_thread_attacher.join();

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

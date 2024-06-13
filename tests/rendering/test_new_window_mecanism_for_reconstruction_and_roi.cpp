#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/ImGUIInterface.h"
#include <iostream>

struct ReconstructionData{

};

void interface(vsg::CommandBuffer& cb,std::atomic<bool>& optimization_running){
    ImGui::Begin("Box Specification Selection");
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;
    static bool local_record_data = false;
    if (optimization_running.load()){
        ImGui::TextColored(ImVec4{1.0,0.0,0.0,1.0},"Optimization Currently Running...Please Wait");
        local_record_data = false;
    } else {
        ImGui::TextColored(ImVec4{0.0,1.0,0.0,1.0},"Can initialize solution with the LBR Med");
    }
    ImGui::Checkbox("Start Robot Positioning", &local_record_data);
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
}

int main(int argc, char **argv) {
    try {
        std::atomic<bool> variable = false;
        curan::renderable::ImGUIInterface::Info info_gui{[&](vsg::CommandBuffer& cb){interface(cb,variable);}};
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
        
        window.run();

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

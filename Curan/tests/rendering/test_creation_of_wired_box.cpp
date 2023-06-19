#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"
#include "rendering/PhaseWiredBox.h"
#include <iostream>

int main(int argc, char **argv) {
    try {
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = true;
        info.is_debug = false;
        info.screen_number = 0;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info};

        auto attach_special_box = [&window](){
            auto box = curan::renderable::PhaseWiredBox::make();
            window << box;
            float time = 0.0;
            vsg::vec3 origin = vsg::vec3(std::cos(time),std::sin(time),0.1f*time);
            auto casted_box = box->cast<curan::renderable::PhaseWiredBox>();
            casted_box->update_frame(origin);
            vsg::vec3 origin_fixed = origin;

            while(time < 5){
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
                time += 0.016f;
                origin = vsg::vec3(std::cos(time),std::sin(time),0.1f*time);
                casted_box->update_frame(origin_fixed,origin);
            }

            auto xdir = origin;

            while(time < 10){
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
                time += 0.016f;
                origin = vsg::vec3(std::cos(time),std::sin(time),0.1f*time);
                casted_box->update_frame(origin_fixed,xdir,origin);
            }

            auto ydir = origin;

            while(time < 15){
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
                time += 0.016f;
                origin = vsg::vec3(std::cos(time),std::sin(time),0.1f*time);
                casted_box->update_frame(origin_fixed,xdir,ydir,origin);
            }
        };
        std::thread attach_lines{attach_special_box};

        window.run();
        attach_lines.join();

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

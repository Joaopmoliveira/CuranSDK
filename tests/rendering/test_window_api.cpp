#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Box.h"
#include <iostream>

int main(int argc, char **argv) {
    try {
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = false;
        info.screen_number = 0;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info}; 

        curan::renderable::Box::Info create_info;
        create_info.geomInfo.dx = vsg::vec3(0.5,0.0,0.0);
        create_info.geomInfo.dy = vsg::vec3(0.0,0.5,0.0);
        create_info.geomInfo.dz = vsg::vec3(0.0,0.0,0.5);
        create_info.geomInfo.position = vsg::vec3(0.0,0.0,0.0);
        create_info.geomInfo.color = vsg::vec4(1.0,0.0,0.0,1.0);
        create_info.builder = vsg::Builder::create();
        vsg::ref_ptr<curan::renderable::Renderable> box = curan::renderable::Box::make(create_info);
        window << box;

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

#include "rendering/Robot.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Sphere.h"
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

        std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/testing/arm.json";
        vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks::make(robot_path);
        robotRenderable->update_transform(vsg::MatrixTransform::create(vsg::translate(vsg::vec3(0.0, 0.0, 0.0))));
        window << robotRenderable;

        window.run();

        window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable>>
                   &map) {
                for (auto &p : map)
                    std::cout << "Object contained: " << p.first << '\n';
            });

    } catch (...) {
        std::cerr << "Exception thrown" << std::endl;
        return 1;
    }
    return 0;
}

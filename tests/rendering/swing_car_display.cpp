#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Mesh.h"
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

        std::filesystem::path swingcar_path = CURAN_COPIED_RESOURCE_PATH"/swing/R4F_stable_II.glb";
        curan::renderable::Mesh::Info create_info;
        create_info.convetion = vsg::CoordinateConvention::Y_UP;
        create_info.mesh_path = swingcar_path;
        vsg::ref_ptr<curan::renderable::Renderable> swingcar = curan::renderable::Mesh::make(create_info);
        window << swingcar;

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
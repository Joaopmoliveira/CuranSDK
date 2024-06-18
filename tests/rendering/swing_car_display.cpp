#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Mesh.h"
#include <iostream>
#include "utils/TheadPool.h"

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

        std::filesystem::path swingcar_path = CURAN_COPIED_RESOURCE_PATH"/swing/R4F_stable.glb";
        curan::renderable::Mesh::Info create_info;
        create_info.convetion = vsg::CoordinateConvention::Y_UP;
        create_info.mesh_path = swingcar_path;
        vsg::ref_ptr<curan::renderable::Renderable> swingcar = curan::renderable::Mesh::make(create_info);
        window << swingcar;
        volatile bool running = true; 
        auto pool = curan::utilities::ThreadPool::create(2);

        pool->submit(curan::utilities::Job{"camera update",[&](){
            auto lookat = window.look_at();
            vsg::dvec3 equilibrium = lookat->center;
            double t = 0.0;
            while(running){

                for(size_t i = 0; i< 3 ; ++i)
                    lookat->center[i]=-0.016*(equilibrium[i]-lookat->center[i]);


                lookat->eye[0] = 2.0+std::sin(t);
                lookat->eye[1] = 2.0+std::sin(t);
                lookat->eye[2] = 2.0+std::sin(t);
                t += 0.016;
                std::this_thread::sleep_for(std::chrono::milliseconds(16));

                lookat->up[0] = 0;
                lookat->up[1] = 0;
                lookat->up[2] = 1;
            }
            
        }});
        window.run();
        running = false;
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

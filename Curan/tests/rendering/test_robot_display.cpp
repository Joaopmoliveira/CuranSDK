#include "rendering/SequencialLinks.h"
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
        vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks::make(robot_path,8);
        robotRenderable->update_transform(vsg::MatrixTransform::create(vsg::rotate(vsg::radians(90.0),1.0,0.0,0.0)));
        window << robotRenderable;

        std::atomic<bool> continue_updating = true;

        auto updater = [robotRenderable,&continue_updating](){
            double angle = 0.0;
            while(continue_updating.load()){
                auto robot = robotRenderable->cast<curan::renderable::SequencialLinks>();
                robot->set(0,angle);
                angle += 0.05;
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
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

    } catch (...) {
        std::cerr << "Exception thrown" << std::endl;
        return 1;
    }
    return 0;
}

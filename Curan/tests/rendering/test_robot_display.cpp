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

        auto builder = vsg::Builder::create();
        curan::renderable::Sphere::Info infosphere;
        infosphere.builder = builder;
        auto sphere = curan::renderable::Sphere::make(infosphere);
        auto sphere_scalling = vsg::MatrixTransform::create(vsg::scale(0.01,0.01,0.01));
        sphere->update_transform(sphere_scalling);
        window << sphere;

        auto sphere2 = curan::renderable::Sphere::make(infosphere);
        auto sphere_scalling2 = vsg::MatrixTransform::create(vsg::scale(0.01,0.01,0.01));
        sphere_scalling2->matrix = sphere_scalling2->transform(vsg::translate(0.0,0.0,1.0));
        sphere2->update_transform(sphere_scalling2);
        window << sphere2;

        auto sphere3 = curan::renderable::Sphere::make(infosphere);
        auto sphere_scalling3 = vsg::MatrixTransform::create(vsg::scale(0.01,0.01,0.01));
        sphere_scalling3->matrix = sphere_scalling3->transform(vsg::translate(1.0,0.0,0.0));
        sphere3->update_transform(sphere_scalling3);
        window << sphere3;

        std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/testing_with_moved_mesh/arm.json";
        curan::renderable::SequencialLinks::Info create_info;
        create_info.convetion = vsg::CoordinateConvention::X_UP;
        create_info.json_path = robot_path;
        create_info.number_of_links = 3;
        vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks::make(create_info);
        window << robotRenderable;

        std::atomic<bool> continue_updating = true;

        auto updater = [robotRenderable,&continue_updating](){
            double angle = 0.0;
            while(continue_updating.load()){
                auto robot = robotRenderable->cast<curan::renderable::SequencialLinks>();
                robot->set(2,angle);
                angle += 0.01;
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

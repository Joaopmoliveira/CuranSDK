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
        vsg::ref_ptr<curan::renderable::Renderable> box = curan::renderable::Box::make(create_info);
        window << box;

        std::atomic<bool> continue_updating = true;

        auto updater = [box,&continue_updating](){
            auto local_mat = vsg::MatrixTransform::create(vsg::rotate(vsg::radians(0.0),0.0,0.0,1.0));
            while(continue_updating.load()){
                box->update_transform(local_mat);
                local_mat->matrix = local_mat->transform(vsg::rotate(vsg::radians(2.0),0.0,0.0,1.0));
            }
        };
        std::thread local_thread{updater};

        auto async_attacher = [box](){
            std::this_thread::sleep_for(std::chrono::seconds(2));
            curan::renderable::Box::Info create_info;
            vsg::ref_ptr<curan::renderable::Renderable> box2 = curan::renderable::Box::make(create_info);
            box2->update_transform(vsg::MatrixTransform::create(vsg::translate(1.0,1.0,1.0)));
            box->append(box2);
        };
        std::thread local_thread_attacher{async_attacher};

        window.run();
        continue_updating.store(false);
        local_thread.join();
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
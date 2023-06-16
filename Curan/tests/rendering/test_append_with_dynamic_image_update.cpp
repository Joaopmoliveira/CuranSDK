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
        create_info.stateInfo.two_sided = true;
        create_info.builder = vsg::Builder::create();
        vsg::ref_ptr<curan::renderable::Renderable> box = curan::renderable::Box::make(create_info);
        window << box;
        std::atomic<bool> continue_updating = true;

        auto updater = [box,&continue_updating](){
            double angle = 0.0;
            while(continue_updating.load()){
                box->update_transform(vsg::rotate(vsg::radians(angle),0.0,0.0,1.0));
                angle += 1;
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
            }
        };
        std::thread local_thread{updater};

        auto async_attacher = [box](){
            std::this_thread::sleep_for(std::chrono::seconds(2));
            curan::renderable::DynamicTexture::Info infotexture;
            infotexture.height = 100;
            infotexture.width = 100;
            infotexture.builder = vsg::Builder::create();
            auto dynamic_texture = curan::renderable::DynamicTexture::make(infotexture);
            dynamic_texture->update_transform(vsg::translate(1.0,1.0,1.0));
            box->append(dynamic_texture);
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

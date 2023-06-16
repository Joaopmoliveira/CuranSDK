#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Box.h"
#include "rendering/DynamicTexture.h"
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
            float value = 1.0;
            auto updateBaseTexture = [value](vsg::vec4Array2D& image)
            {
                using value_type = typename vsg::vec4Array2D::value_type;
                for (size_t r = 0; r < image.height(); ++r)
                {
                    float r_ratio = static_cast<float>(r) / static_cast<float>(image.height() - 1);
                    value_type* ptr = &image.at(0, r);
                    for (size_t c = 0; c < image.width(); ++c)
                    {
                        float c_ratio = static_cast<float>(c) / static_cast<float>(image.width() - 1);

                        vsg::vec2 delta((r_ratio - 0.5f), (c_ratio - 0.5f));

                        float angle = atan2(delta.x, delta.y);

                        float distance_from_center = vsg::length(delta);

                        float intensity = (sin(1.0 * angle + 30.0f * distance_from_center + 10.0 * value) + 1.0f) * 0.5f;

                        ptr->r = intensity;
                        ptr->g = intensity;
                        ptr->b = intensity;
                        ptr->a = 1.0f;

                        ++ptr;
                    }
                }
            };
            dynamic_texture->cast<curan::renderable::DynamicTexture>()->update_texture(updateBaseTexture);

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

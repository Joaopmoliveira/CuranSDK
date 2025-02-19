#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Box.h"
#include "rendering/DynamicHeight.h"
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

        std::atomic<bool> continue_updating = true;

        auto async_attacher = [&](){
            curan::renderable::DynamicHeight::Info infotexture;
            infotexture.height = 200;
            infotexture.width = 200;
            infotexture.depth = 200;
            infotexture.origin = {0.0,0.0,0.0};
            infotexture.spacing = {0.005,0.005,0.005};
            infotexture.builder = vsg::Builder::create();
            auto dynamic_texture = curan::renderable::DynamicHeight::make(infotexture);
            dynamic_texture->update_transform(vsg::translate(0.0,0.0,0.0));
            window << dynamic_texture;
            float value = 1.0;
            auto updateBaseTexture = [&value](vsg::floatArray2D& image)
            {
                using value_type = typename vsg::floatArray2D::value_type;
                for (int r = 0; r < image.height(); ++r)
                {
                    float r_ratio = static_cast<float>(r) / static_cast<float>(image.height() - 1);
                    value_type* ptr = &image.at(0, r);
                    for (int c = 0; c < image.width(); ++c)
                    {
                        float c_ratio = static_cast<float>(c) / static_cast<float>(image.width() - 1);

                        vsg::vec2 delta((r_ratio - 0.5f), (c_ratio - 0.5f));

                        float angle = atan2(delta.x, delta.y);

                        float distance_from_center = vsg::length(delta);

                        float intensity = 0.1*(sin(1.0 * angle + 30.0f * distance_from_center + 10.0f * value) + 1.0f) * 0.5f;
                        *ptr = intensity;
                        ++ptr;
                    }
                }
            };
            while(continue_updating){
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                dynamic_texture->cast<curan::renderable::DynamicHeight>()->update_texture(updateBaseTexture);
                value += 0.01f;
            }
            

        };
        std::thread local_thread_attacher{async_attacher};

        window.run();
        continue_updating = false;
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

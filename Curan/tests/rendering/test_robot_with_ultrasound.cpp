#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
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

        std::atomic<bool> continue_updating = true;

        auto async_attacher = [&window,&continue_updating](){
            std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
            curan::renderable::SequencialLinks::Info create_info;
            create_info.convetion = vsg::CoordinateConvention::Y_UP;
            create_info.json_path = robot_path;
            create_info.number_of_links = 8;
            vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks::make(create_info);
            window << robotRenderable;

            if(!continue_updating.load())
                return;

            curan::renderable::DynamicTexture::Info infotexture;
            infotexture.height = 100;
            infotexture.width = 100;
            infotexture.geomInfo.dx = vsg::vec3(0.2f,0.0f,0.0f);
            infotexture.geomInfo.dy = vsg::vec3(0.0f,.2f,0.0f);
            infotexture.geomInfo.dz = vsg::vec3(0.0f,0.0f,0.0f);
            infotexture.geomInfo.position = vsg::vec3(0.0f,0.1f,0.0f);
            infotexture.builder = vsg::Builder::create();
            auto dynamic_texture = curan::renderable::DynamicTexture::make(infotexture);
            dynamic_texture->update_transform(vsg::rotate<double>(vsg::radians(90.0),1.0,0.0,0.0)*vsg::translate<double>(0.0,0.126,0.0));
            robotRenderable->append(dynamic_texture);

            if(!continue_updating.load())
                return;

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

                        float intensity = (sin(1.0 * angle + 30.0f * distance_from_center + 10.0f * value) + 1.0f) * 0.5f;

                        ptr->r = intensity;
                        ptr->g = intensity;
                        ptr->b = intensity;
                        ptr->a = 1.0f;

                        ++ptr;
                    }
                }
            };
            dynamic_texture->cast<curan::renderable::DynamicTexture>()->update_texture(updateBaseTexture);

            double angle = 0.0;
            double time = 0.0;
            while(continue_updating.load()){
                auto robot = robotRenderable->cast<curan::renderable::SequencialLinks>();
                for(size_t index = 0; index < 7 ; ++index)
                    robot->set(index,angle);
                angle = std::sin(time)*1.5;
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
                time += 0.016;
            }
        };
        std::thread local_thread_attacher{async_attacher};

        window.run();
        continue_updating.store(false);
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

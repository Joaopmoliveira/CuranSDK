#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include <iostream>

void updateBaseTexture3D(vsg::floatArray3D& image, float value)
{
    for (size_t d = 0; d < image.depth(); ++d) {
        float d_ratio = static_cast<float>(d) / static_cast<float>(image.depth() - 1);
        for (size_t r = 0; r < image.height(); ++r)
        {
            float r_ratio = static_cast<float>(r) / static_cast<float>(image.height() - 1);
            for (size_t c = 0; c < image.width(); ++c)
            {
                float c_ratio = static_cast<float>(c) / static_cast<float>(image.width() - 1);

                vsg::vec3 delta((r_ratio - 0.5f), (c_ratio - 0.5f),(d_ratio-0.5));

                float distance_from_center = (d_ratio* d_ratio + r_ratio * r_ratio + c_ratio * c_ratio)/sqrt(3.0f);
                float angle = atan2(delta.x, delta.y);
                float intensity = (sin(1.0 * angle + 30.0f * distance_from_center + 10.0 * value) + 1.0f) * 0.5f;

                //float intensity = distance_from_center;
                image.set(c, r, d, intensity);
            }
        }
    }

    image.dirty();
}


int main(int argc, char** argv) {
try{
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

    std::atomic<bool> continue_moving = true;
    auto mover = [&continue_moving,&window](){
        constexpr size_t width = 100;
        constexpr size_t height = 100;
        constexpr size_t depth = 100;

        curan::renderable::Volume::Info volumeinfo;
        volumeinfo.width = width; 
        volumeinfo.height = height;
        volumeinfo.depth = depth;
        volumeinfo.spacing_x = 1.0;
        volumeinfo.spacing_y = 1.0;
        volumeinfo.spacing_z = 1.0;
        auto volume = curan::renderable::Volume::make(volumeinfo);
        window << volume;

        std::cout << "appended the volume on screen\n";

        float time = 0.0;
        auto casted_volume = volume->cast<curan::renderable::Volume>();
        while(continue_moving.load()){
            auto updater = [&](vsg::floatArray3D& image){
                updateBaseTexture3D(image, 1.0f+time);
            };
            casted_volume->update_volume(updater);
            //volume->update_transform(vsg::translate(std::cos(time)*0.2,std::sin(time)*0.2,0.2)*vsg::rotate(time,1.0,0.0,0.0));
            time += 0.016;
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    };
    std::thread mover_thread{mover};

    window.run();
    continue_moving.store(false);
    mover_thread.join();

    window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable >>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }
    });

} catch (const std::exception& e) {
     std::cerr << "Exception thrown : " << e.what() << std::endl;
    return 1;
}
// clean up done automatically thanks to ref_ptr<>
return 0;

}
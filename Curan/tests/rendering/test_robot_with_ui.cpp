#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"
#include <iostream>

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/Texture.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/implot.h>

struct ImGUIInterface : public vsg::Inherit<vsg::Command, ImGUIInterface> {

    using im_gui_callable = std::function<void(vsg::CommandBuffer& cb)>;

    struct Info {
        im_gui_callable callable;

        Info(im_gui_callable in_callable) : callable{in_callable}{

        }
    };

    im_gui_callable callable;
            
    ImGUIInterface(Info& info) : callable{info.callable}{
    }

    // we need to compile textures before we can use them for rendering
    void compile(vsg::Context& context) override
    {

    }

    void record(vsg::CommandBuffer& cb) const override
    {
        callable(cb);
    }

    static vsg::ref_ptr<ImGUIInterface> make(Info& info){
        return ImGUIInterface::create(info);
    }
};

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

        std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
        curan::renderable::SequencialLinks::Info create_info;
        create_info.convetion = vsg::CoordinateConvention::Y_UP;
        create_info.json_path = robot_path;
        create_info.number_of_links = 8;
        vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks::make(create_info);
        window << robotRenderable;

        std::atomic<bool> continue_updating = true;

        auto async_attacher = [&](){
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

        // Add the ImGui event handler first to handle events early
        viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());

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

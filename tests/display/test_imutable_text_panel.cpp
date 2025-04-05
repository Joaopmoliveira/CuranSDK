#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/ImutableTextPanel.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/Page.h"

#include <iostream>
#include "utils/TheadPool.h"

int main(){
    try
    {
        using namespace curan::ui;
        IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
        std::unique_ptr<Context> context = std::make_unique<Context>();

        DisplayParams param{std::move(context), 1200, 800};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        std::unique_ptr<ImutableTextPanel> layer = ImutableTextPanel::make("write for life");
        layer->set_background_color({1.f,1.0f,1.0f,1.0f}).set_text_color({.0f,.0f,.0f,1.0f});
        auto container = Container::make(Container::ContainerType::VARIABLE_CONTAINER,Container::Arrangement::UNDEFINED);
        container->set_variable_layout({SkRect::MakeXYWH(0.2,0,0.6,1)});
        ImutableTextPanel* layer_ptr = layer.get();
        *container << std::move(layer);

        curan::ui::Page page{std::move(container), SK_ColorBLACK};

        auto pool = curan::utilities::ThreadPool::create(1);
        pool->submit("text updater",[&](){
            for(size_t i = 0; i < 100; ++i){
                layer_ptr->text("new text: "+std::to_string(i));
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
        });
        

        ConfigDraw config{&page};

        while (!glfwWindowShouldClose(viewer->window))
        {
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
            SkCanvas *canvas = pointer_to_surface->getCanvas();

            if (viewer->was_updated())
            {
                page.update_page(viewer.get());
                viewer->update_processed();
            }
            page.draw(canvas);
            auto signals = viewer->process_pending_signals();
            if (!signals.empty())
                page.propagate_signal(signals.back(), &config);
            glfwPollEvents();

            bool val = viewer->swapBuffers();
            if (!val)
                std::cout << "failed to swap buffers\n";
            auto end = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        }
        return 0;
    }
    catch (...)
    {
        std::cout << "Failed";
        return 1;
    }
}
#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/MutatingTextPanel.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/Page.h"
#include <iostream>
#include <thread>

int main(){
    try
    {
        using namespace curan::ui;
        IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
        std::unique_ptr<Context> context = std::make_unique<Context>();

        DisplayParams param{std::move(context), 1200, 800};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        std::unique_ptr<MutatingTextPanel> layer = MutatingTextPanel::make(false,"write for life");
        layer->set_background_color({.0f, .0f, .0f, 1.0f}).set_text_color({1.f,1.f,1.f,1.f}).set_highlighted_color({.2f, .2f, .2f, 1.0f}).set_cursor_color({1.0,0.0,0.0,1.0});
        auto container = Container::make(Container::ContainerType::VARIABLE_CONTAINER, Container::Arrangement::UNDEFINED);
        container->set_variable_layout({SkRect::MakeLTRB(0.3,0.1,0.9,0.9)});
        container->set_color(SkColorSetARGB(255,255,255,255));
        auto ptr_layer = layer.get();
        
        layer->setFont(MutatingTextPanel::typeface::serif);
        *container << std::move(layer);

        curan::ui::Page page{std::move(container), SK_ColorBLACK};


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
    catch (std::runtime_error& e)
    {
        std::cout << "Failed:" << e.what() << std::endl;
        return 1;
    }
    catch(...){
        std::cout << "Failed:" << std::endl;
        return 2; 
    }
}
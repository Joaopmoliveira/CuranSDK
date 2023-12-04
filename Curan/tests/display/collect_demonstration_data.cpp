#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include <thread>

int main()
{
    try
    {
        using namespace curan::ui;
        IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
        std::unique_ptr<Context> context = std::make_unique<Context>();
        ;
        DisplayParams param{std::move(context), 1200, 800};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        while (!glfwWindowShouldClose(viewer->window))
        {
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
            SkCanvas *canvas = pointer_to_surface->getCanvas();
            canvas->drawColor(SK_ColorWHITE);
            SkPaint paint;
            paint.setAntiAlias(true);
            SkPath path;
            path.moveTo(124, 108);
            path.lineTo(172, 24);
            path.addCircle(50, 50, 30);
            path.moveTo(36, 148);
            path.quadTo(66, 188, 120, 136);
            canvas->drawPath(path, paint);
            auto end = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        }
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cout << "Exception thrown:" << e.what() << "\n";
    }
    catch (...)
    {
        std::cout << "Failed to create window for unknown reason\n";
        return 1;
    }
}
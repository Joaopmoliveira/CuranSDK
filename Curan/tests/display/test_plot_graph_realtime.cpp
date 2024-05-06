#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "utils/CircularBuffer.h"
#include <thread>
int main()
{
    try
    {
        using namespace curan::ui;
        IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
        std::unique_ptr<Context> context = std::make_unique<Context>();
        DisplayParams param{std::move(context), 1200, 800};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
        curan::utilities::CircularBuffer<SkPoint> buffer{10};

        float time = 0;
        uint8_t verbs[10];
        verbs[0] = SkPath::kMove_Verb;
        for(size_t i = 1 ; i < 10; ++i)
            verbs[i] = SkPath::kLine_Verb;
        for(auto& val : buffer){
            val.fX = time;
            val.fY = 100.0f+10.0f*std::sin(time*0.1f);
            time += 10.0f;
            std::printf("%.2f %.2f\n",val.fX,val.fY);
        }

        SkPath path = SkPath::Make(buffer.data(),10,verbs,10,nullptr,0,SkPathFillType::kEvenOdd,true);
        

        while (!glfwWindowShouldClose(viewer->window))
        {
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface *surface = viewer->getBackbufferSurface();
            SkCanvas *canvas = surface->getCanvas();
            SkPaint paint;
            paint.setAntiAlias(true);
            paint.setStyle(SkPaint::kStroke_Style);
            paint.setColor(SK_ColorWHITE);
            paint.setStrokeWidth(3);
            canvas->drawPath(path,paint);

            buffer.put({time,100.0f+10.0f*std::sin(time*0.1f)});
            time += 10.0f;
            if(time>200.0f)
                time = 0.0f;
            
            path = SkPath::Make(buffer.data(),10,verbs,10,nullptr,0,SkPathFillType::kEvenOdd,true);

            glfwPollEvents();
            auto signals = viewer->process_pending_signals();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
            auto end = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        }
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cout << "\nException thrown:\n" << e.what() << "\n";
    }
    catch (...)
    {
        std::cout << "Failed to create window for unknown reason\n";
        return 1;
    }
}
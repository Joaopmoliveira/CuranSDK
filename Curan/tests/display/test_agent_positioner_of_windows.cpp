#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include <thread>

std::unique_ptr<curan::ui::Window> make_window()
{
    curan::ui::IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
    std::unique_ptr<curan::ui::Context> context = std::make_unique<Context>();
    curan::ui::DisplayParams param{std::move(context), 1200, 800};
    return std::make_unique<curan::ui::Window>(std::move(param));
}

struct WindowInfoPlacement{
    std::unique_ptr<curan::ui::Window> window;
    std::array<int,2> previous_position;

};

struct Agent
{
    std::list<WindowInfoPlacement> windows;
    std::array<int,2> maximum_size;
    
    void loop_windows()
    {
        while (true)
        {
            bool did_change = false;
            windows.remove_if([](WindowInfoPlacement &placement)
                              { return glfwWindowShouldClose(placement.window->window); });
            if(windows.size()==0)
                break;
            auto start = std::chrono::high_resolution_clock::now();
            for (auto &window : windows)
            {
                int xpos, ypos;
                glfwGetWindowPos(window->window, &xpos, &ypos);
                if() //position changes then we need to update stuff

                SkSurface *pointer_to_surface = window->getBackbufferSurface();
                SkCanvas *canvas = pointer_to_surface->getCanvas();
                canvas->drawColor(SK_ColorWHITE);
                SkPoint point{100, 10};
                canvas->drawCircle(point, 5.0, paint_square);
                glfwPollEvents();
                auto signals = window->process_pending_signals();

                bool val = window->swapBuffers();
                if (!val)
                    std::cout << "failed to swap buffers\n";
            }
            auto end = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        }
    };

    void compute_positions{std::vector<std::array<double,2>>& positions}{
        size_t number_of_windows = positions.size();
        double window_total_size = 100;
        if(number_of_windows % 2 == 0) //even

        else //odd
    }
};

int main()
{
    std::unique_ptr<curan::ui::Window> viewer = make_window();
    std::unique_ptr<curan::ui::Window> viewer2 = make_window();
    Agent window_manager;
    return 0;
}
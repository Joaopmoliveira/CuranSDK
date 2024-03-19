#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/RuntimeEffect.h"
#include <iostream>
#include <thread>

static constexpr char SKSL[] =
"uniform float3 in_resolution;"
"uniform float  in_time;"
"float f(vec3 p) {"
"    p.z -= in_time * 10.;"
"    float a = p.z * .1;"
"    p.xy *= mat2(cos(a), sin(a), -sin(a), cos(a));"
"    return .1 - length(cos(p.xy) + sin(p.yz));"
"}"
"half4 main(vec2 fragcoord) { "
"    vec3 d = .5 - fragcoord.xy1 / in_resolution.y;"
"    vec3 p=vec3(0);"
"    for (int i = 0; i < 32; i++) {"
"      p += f(p) * d;"
"    }"
"    return ((sin(p) + vec3(2, 5, 12)) / length(p)).xyz1;"
"}";

int main()
{
    try
    {
        using namespace curan::ui;
        IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
        std::unique_ptr<Context> context = std::make_unique<Context>();
        DisplayParams param{std::move(context), 1200, 800};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
        RuntimeEffect effects{SKSL};

        while (!glfwWindowShouldClose(viewer->window))
        {
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface *surface = viewer->getBackbufferSurface();
            SkCanvas *canvas = surface->getCanvas();
            effects.draw(canvas,SkIRect::MakeWH(surface->width(),surface->height()));

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
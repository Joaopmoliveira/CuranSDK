#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Slider.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include <thread>

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		auto callback = [](Slider* slider, ConfigDraw* config) {
			std::cout << "received signal!\n";
		};

		auto slider = Slider::make({ 0.0f, 300.0f });
		slider->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY)
				.set_slider_color(SK_ColorLTGRAY).set_callback([](Slider* slider, ConfigDraw* config) {
			std::cout << "received signal!\n";
		}).set_size(SkRect::MakeWH(300,40));

		slider->compile();
		slider->set_position(SkRect::MakeXYWH(50,50,300,40));
		auto caldraw = slider->draw();
		auto calsignal = slider->call();

		ConfigDraw config_draw;

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);
			caldraw(canvas);
			glfwPollEvents();
			auto signals = viewer->process_pending_signals();
			if (!signals.empty())
				calsignal(signals.back(),&config_draw);
			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (std::exception & e) {
		std::cout << "Failed: " << e.what() << std::endl;
		return 1;
	}
}
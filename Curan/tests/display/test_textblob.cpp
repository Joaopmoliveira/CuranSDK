#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/TextBlob.h"
#include <iostream>

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		auto textblob = TextBlob::make("I have become death");
		textblob->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 90));
		SkRect rect = SkRect::MakeXYWH(50, 100, 300, 200);
		textblob->compile();
		textblob->set_position(rect);
		auto caldraw = textblob->draw();
		auto calsignal = textblob->call();

		ConfigDraw config_draw{};

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
	catch (...) {
		std::cout << "Failed";
		return 1;
	}
}
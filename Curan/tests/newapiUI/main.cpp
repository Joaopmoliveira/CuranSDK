#define STB_IMAGE_IMPLEMENTATION
#include "modifieduserinterface/widgets/ConfigDraw.h"
#include "modifieduserinterface/Window.h"
#include "modifieduserinterface/widgets/Button.h"
#include "modifieduserinterface/widgets/IconResources.h"
#include "modifieduserinterface/Window.h"

#include <iostream>

int main() {
	try {
		curan::ui::IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images" };
		std::unique_ptr<curan::ui::Context> context = std::make_unique<curan::ui::Context>();;
		curan::ui::DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<curan::ui::Window> viewer = std::make_unique<curan::ui::Window>(std::move(param));

		auto button = curan::ui::Button::make("hi mark!",&resources);
    	button->set_click_color(SK_ColorBLUE).set_current_state(curan::ui::Button::ButtonStates::PRESSED).set_hover_color(SK_ColorBLUE).set_waiting_color(SK_ColorBLUE);
		
		auto buttoncontainer = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::VERTICAL);
		*buttoncontainer << std::move(button);
		SkColor colbuton = { SK_ColorRED };

		//this should be an error
		auto val = button->get_click_color();
		
		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kFill_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(colbuton);

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);
			SkPoint point{ 100,10 };
			canvas->drawCircle(point,5.0, paint_square);
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
	catch (...) {
		std::cout << "Failed";
		return 1;
	}
}

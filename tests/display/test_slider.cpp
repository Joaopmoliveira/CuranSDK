#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Slider.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ "C:/dev/Curan/resources" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		SkColor colbuton = { SK_ColorWHITE };

		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kStrokeAndFill_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(20);
		paint_square.setColor(colbuton);
		paint_square.setStrokeJoin(SkPaint::kRound_Join);
		paint_square.setStrokeCap(SkPaint::kRound_Cap);

		auto callback = [](Slider* slider, ConfigDraw* config) {
			std::cout << "received signal!\n";
		};

		Slider::Info infor{};
		infor.click_color = SK_ColorLTGRAY;
		infor.hover_color = SK_ColorCYAN;
		infor.waiting_color = SK_ColorDKGRAY;
		infor.sliderColor = SK_ColorGRAY;
		infor.paintButton = paint_square;
		infor.size = SkRect::MakeWH(200, 40);
		infor.callback = callback;
		infor.limits = { 0.0f, 300.0f };
		std::shared_ptr<Slider> button = Slider::make(infor);
		SkRect rect = SkRect::MakeXYWH(50, 100, 300, 40);
		button->set_position(rect);
		auto caldraw = button->draw();
		auto calsignal = button->call();

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
	catch (...) {
		std::cout << "Failed";
		return 1;
	}
}
#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include <iostream>
#include <thread>

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		SkColor colbuton = { SK_ColorRED };

		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kFill_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(colbuton);

		auto typeface = curan::ui::defaultTypeface();

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);

    		SkFont font1(typeface, 64.0f, 1.0f, 0.0f);
    		SkFont font2(typeface, 64.0f, 1.5f, 0.0f);
    		font1.setEdging(SkFont::Edging::kAntiAlias);
    		font2.setEdging(SkFont::Edging::kAntiAlias);

    		sk_sp<SkTextBlob> blob1 = SkTextBlob::MakeFromString("Skia", font1);
    		sk_sp<SkTextBlob> blob2 = SkTextBlob::MakeFromString("Skia", font2);

    		SkPaint paint1, paint2, paint3;

    		paint1.setAntiAlias(true);
    		paint1.setColor(SkColorSetARGB(0xFF, 0x42, 0x85, 0xF4));

   			paint2.setAntiAlias(true);
    		paint2.setColor(SkColorSetARGB(0xFF, 0xDB, 0x44, 0x37));
    		paint2.setStyle(SkPaint::kStroke_Style);
    		paint2.setStrokeWidth(3.0f);

    		paint3.setAntiAlias(true);
    		paint3.setColor(SkColorSetARGB(0xFF, 0x0F, 0x9D, 0x58));

    		canvas->clear(SK_ColorWHITE);
    		canvas->drawTextBlob(blob1.get(), 20.0f, 64.0f, paint1);
    		canvas->drawTextBlob(blob1.get(), 20.0f, 144.0f, paint2);
   			canvas->drawTextBlob(blob2.get(), 20.0f, 224.0f, paint3);

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
#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/TextBlob.h"
#include <iostream>

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ "C:/dev/Curan/resources" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		SkColor colbuton = { SK_ColorWHITE };
		SkColor coltext = { SK_ColorBLACK };

		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kFill_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(colbuton);

		SkPaint paint_text;
		paint_text.setStyle(SkPaint::kFill_Style);
		paint_text.setAntiAlias(true);
		paint_text.setStrokeWidth(4);
		paint_text.setColor(coltext);

		const char* fontFamily = nullptr;
		SkFontStyle fontStyle;
		sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
		sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

		SkFont text_font = SkFont(typeface, 10, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);

		auto callback = []() {
			std::cout << "received signal!\n";
		};

		TextBlob::Info infor{};
		infor.button_text = "Touch!";
		infor.paint = paint_square;
		infor.paintText = paint_text;
		infor.size = SkRect::MakeWH(200, 90);
		infor.textFont = text_font;
		std::shared_ptr<TextBlob> button = TextBlob::make(infor);
		SkRect rect = SkRect::MakeXYWH(50, 100, 300, 200);
		button->set_position(rect);
		auto caldraw = button->draw();
		auto calsignal = button->call();

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);
			caldraw(canvas);
			glfwPollEvents();
			auto signals = viewer->process_pending_signals();
			if (!signals.empty())
				calsignal(signals.back());
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
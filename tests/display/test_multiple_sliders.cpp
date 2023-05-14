#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Slider.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>

#include <iostream>

std::shared_ptr<curan::ui::Overlay> create_option_page() {
	using namespace curan::ui;
	IconResources resources{ "C:/dev/Curan/resources" };

	SkColor colbuton = { SK_ColorBLACK };
	SkColor coltext = { SK_ColorWHITE };

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

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SK_ColorTRANSPARENT);

	const char* fontFamily = nullptr;
	SkFontStyle fontStyle;
	sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
	sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

	SkFont text_font = SkFont(typeface, 20, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	Container::InfoLinearContainer infocontainer;
	infocontainer.paint_layout = paint_square2;
	infocontainer.arrangement = curan::ui::Arrangement::HORIZONTAL;

	Slider::Info infor{};
	infor.click_color = SK_ColorLTGRAY;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorDKGRAY;
	infor.sliderColor = SK_ColorGRAY;
	infor.paintButton = paint_square;
	infor.size = SkRect::MakeWH(200, 40);
	infor.limits = { 0.0f, 300.0f };
	std::shared_ptr<Slider> button = Slider::make(infor);

	TextBlob::Info infotext;
	infotext.button_text = "Option 1";
	infotext.paint = paint_square;
	infotext.paintText = paint_text;
	infotext.size = SkRect::MakeWH(200, 40);
	infotext.textFont = text_font;
	std::shared_ptr<TextBlob> text = TextBlob::make(infotext);

	infocontainer.layouts = {text,button};
	std::shared_ptr<Container> container = Container::make(infocontainer);

	infotext.button_text = "Option 2";
	std::shared_ptr<TextBlob> text1 = TextBlob::make(infotext);
	std::shared_ptr<Slider> button1 = Slider::make(infor);
	infocontainer.layouts = { text1,button1 };
	std::shared_ptr<Container> container1 = Container::make(infocontainer);

	infotext.button_text = "Option 3";
	std::shared_ptr<TextBlob> text2 = TextBlob::make(infotext);
	std::shared_ptr<Slider> button2 = Slider::make(infor);
	infocontainer.layouts = { text2,button2 };
	std::shared_ptr<Container> container2 = Container::make(infocontainer);

	infotext.button_text = "Option 4";
	std::shared_ptr<TextBlob> text3 = TextBlob::make(infotext);
	std::shared_ptr<Slider> button3 = Slider::make(infor);
	infocontainer.layouts = { text3,button3 };
	std::shared_ptr<Container> container3 = Container::make(infocontainer);

	infocontainer.arrangement = curan::ui::Arrangement::VERTICAL;
	infocontainer.divisions = { 0.0 , 0.25 , 0.5 , 0.75 , 1.0 };
	infocontainer.layouts = { container,container1,container2,container3};
	std::shared_ptr<Container> containerotions = Container::make(infocontainer);

	Overlay::Info information;
	information.backgroundcolor = SK_ColorTRANSPARENT;
	information.contained = containerotions;
	return Overlay::make(information);
}


int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ "C:/dev/Curan/resources" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		SkColor colbuton = { SK_ColorBLACK };
		SkColor coltext = { SK_ColorWHITE };

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

		SkFont text_font = SkFont(typeface, 20, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);

		SkPaint paint_square2;
		paint_square2.setStyle(SkPaint::kFill_Style);
		paint_square2.setAntiAlias(true);
		paint_square2.setStrokeWidth(4);
		paint_square2.setColor(SK_ColorBLACK);

		Button::Info infor{ resources };
		infor.button_text = "Connect";
		infor.click_color = SK_ColorGRAY;
		infor.hover_color = SK_ColorDKGRAY;
		infor.waiting_color = SK_ColorBLACK;
		infor.icon_identifier = "";
		infor.paintButton = paint_square;
		infor.paintText = paint_text;
		infor.size = SkRect::MakeWH(100, 80);
		infor.textFont = text_font;
		std::shared_ptr<Button> button = Button::make(infor);
		infor.button_text = "Options";
		std::shared_ptr<Button> buttonoptions = Button::make(infor);

		Container::InfoLinearContainer info;
		info.paint_layout = paint_square2;
		info.arrangement = curan::ui::Arrangement::HORIZONTAL;
		info.divisions = { 0.0 , 0.5 , 1.0 };
		info.layouts = { button,buttonoptions };
		std::shared_ptr<Container> buttoncontainer = Container::make(info);

		auto rec = viewer->get_size();
		Page::Info information;
		information.backgroundcolor = SK_ColorBLACK;
		information.contained = buttoncontainer;
		std::shared_ptr<Page> page = Page::make(information);
		page->propagate_size_change(rec);

		auto button_callback = [&page]() {
			auto temp_optional_page = create_option_page();
			page->stack(temp_optional_page);
		};
		buttonoptions->set_callback(button_callback);

		int width = rec.width();
		int height = rec.height();

		ConfigDraw config_draw{ page.get() };

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			auto temp_height = pointer_to_surface->height();
			auto temp_width = pointer_to_surface->width();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			if (temp_height != height || temp_width != width) {
				rec = SkRect::MakeWH(temp_width, temp_height);
				page->propagate_size_change(rec);
			}
			page->draw(canvas);
			auto signals = viewer->process_pending_signals();

			if (!signals.empty())
				page->propagate_signal(signals.back(), &config_draw);

			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			//std::cout << "delay: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (...) {
		std::cout << "Failed";
		return 1;
	}
}
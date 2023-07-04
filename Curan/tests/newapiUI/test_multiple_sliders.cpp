#define STB_IMAGE_IMPLEMENTATION
#include "modifieduserinterface/widgets/ConfigDraw.h"
#include "modifieduserinterface/Window.h"
#include "modifieduserinterface/widgets/Button.h"
#include "modifieduserinterface/widgets/Container.h"
#include "modifieduserinterface/widgets/ConfigDraw.h"
#include "modifieduserinterface/widgets/IconResources.h"
#include "modifieduserinterface/widgets/Slider.h"
#include "modifieduserinterface/widgets/TextBlob.h"
#include "modifieduserinterface/widgets/Page.h"
#include "modifieduserinterface/widgets/Overlay.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>

#include <iostream>

std::shared_ptr<curan::ui::Overlay> create_option_page() {
	using namespace curan::ui;

	auto slider = Slider::make({ 0.0f, 300.0f });
	slider->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob = TextBlob::make("Option 1");
	textblob->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));

	auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container << std::move(slider) << std::move(textblob);
	container->set_divisions({ 0.0 , 0.5 , 1.0 });

	auto slider1 = Slider::make({ 0.0f, 300.0f });
	slider1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob1 = TextBlob::make("Option 2");
	textblob1->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));

	auto container1 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container1 << std::move(slider1) << std::move(textblob1);
	container1->set_divisions({ 0.0 , 0.5 , 1.0 });

	auto slider2 = Slider::make({ 0.0f, 300.0f });
	slider2->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob2 = TextBlob::make("Option 3");
	textblob2->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));

	auto container2 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container2 << std::move(slider2) << std::move(textblob2);
	container2->set_divisions({ 0.0 , 0.5 , 1.0 });

	auto slider3 = Slider::make({ 0.0f, 300.0f });
	slider3->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob3 = TextBlob::make("Option 4");
	textblob3->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));

	auto container3 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container3 << std::move(slider3) << std::move(textblob3);
	container3->set_divisions({ 0.0 , 0.5 , 1.0 });

	auto container4 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*container4 << std::move(container) << std::move(container1) << std::move(container2) << std::move(container3);
	container4->set_divisions({ 0.0 , 0.25 , 0.5 , 0.75 , 1.0 });

	Overlay::Info information;
	information.backgroundcolor = SK_ColorTRANSPARENT;
	information.contained = containerotions;
	return Overlay::make(information);
}


int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/image" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

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

		auto button_callback = [&page](Button* slider,ConfigDraw* config) {
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
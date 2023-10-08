#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Slider.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>

#include <iostream>
#include <thread>

std::unique_ptr<curan::ui::Overlay> create_option_page() {
	using namespace curan::ui;
	//---------------------- row 1 -------------------//
	auto slider = Slider::make({ 0.0f, 300.0f });
	slider->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob = TextBlob::make("Option 1");
	textblob->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));

	auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container << std::move(slider) << std::move(textblob);
	container->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- row 2 -------------------//
	auto slider1 = Slider::make({ 0.0f, 300.0f });
	slider1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob1 = TextBlob::make("Option 2");
	textblob1->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));

	auto container1 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container1 << std::move(slider1) << std::move(textblob1);
	container1->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- row 3 -------------------//
	auto slider2 = Slider::make({ 0.0f, 300.0f });
	slider2->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob2 = TextBlob::make("Option 3");
	textblob2->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));

	auto container2 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container2 << std::move(slider2) << std::move(textblob2);
	container2->set_divisions({ 0.0 , 0.5 , 1.0 });
	//---------------------- row 4 -------------------//
	auto slider3 = Slider::make({ 0.0f, 300.0f });
	slider3->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 40));
	auto textblob3 = TextBlob::make("Option 4");
	textblob3->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 40));

	auto container3 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*container3 << std::move(slider3) << std::move(textblob3);
	container3->set_divisions({ 0.0 , 0.5 , 1.0 });

	//---------------------- stack rows -------------------//
	auto container4 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*container4 << std::move(container) << std::move(container1) << std::move(container2) << std::move(container3);
	container4->set_divisions({ 0.0 , 0.25 , 0.5 , 0.75 , 1.0 });

	return Overlay::make(std::move(container4),SK_ColorTRANSPARENT);
}


int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		auto button = Button::make("Connect",resources);
		button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK);

		auto buttonoptions = Button::make("Options",resources);
		buttonoptions->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK);
		auto buttonoptions_pointer = buttonoptions.get();

		auto container2 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
		*container2 << std::move(button) << std::move(buttonoptions);
		container2->set_divisions({ 0.0 , 0.5 , 1.0 });

		auto rec = viewer->get_size();

		auto page = Page{std::move(container2),SK_ColorBLACK};
		page.propagate_size_change(rec);

		auto button_callback = [&page](Button* slider,Press press,ConfigDraw* config) {
			page.stack(std::move(create_option_page()));
		};
		buttonoptions_pointer->add_press_call(button_callback);

		auto width = rec.width();
		auto height = rec.height();

		ConfigDraw config_draw{ &page };

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			auto temp_height = pointer_to_surface->height();
			auto temp_width = pointer_to_surface->width();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			if (temp_height != height || temp_width != width) {
				rec = SkRect::MakeWH(temp_width, temp_height);
				page.propagate_size_change(rec);
			}
			page.draw(canvas);
			auto signals = viewer->process_pending_signals();

			if (!signals.empty())
				page.propagate_signal(signals.back(), &config_draw);

			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (std::exception& e) {
		std::cout << "Failed" << e.what() << std::endl;
		return 1;
	}
}
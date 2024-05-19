#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"

#include <iostream>
#include <thread>

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		auto button1 = Button::make("print name",resources);
		button1->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY);
		auto button2 = Button::make("print age",resources);
		button2->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY);

		auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
		*container << std::move(button1) << std::move(button2);
		container->set_divisions({ 0.0 , 0.5 , 1.0 });
		auto button3 = Button::make("print Eureka",resources);
		button3->set_click_color(SK_ColorRED).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY);

		auto container2 = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
		*container2 << std::move(container) << std::move(button3);
		container2->set_divisions({ 0.0 , 0.5 , 1.0 });

		Page page = Page{std::move(container2),SK_ColorBLACK};
		page.update_page(viewer.get());

		ConfigDraw config;

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			if (viewer->was_updated()) {
				page.update_page(viewer.get());
				viewer->update_processed();
			}
			page.draw(canvas);
			auto signals = viewer->process_pending_signals();
			if (!signals.empty())
				page.propagate_signal(signals.back(),&config);
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
		std::cout << "Failed: " << e.what()<<std::endl;
		return 1;
	}
}
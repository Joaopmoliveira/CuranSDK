#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/Loader.h"

#include "utils/Flag.h"
#include "utils/TheadPool.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"

#include "processmanagement/SingleChildParent.h"

#include <thread>
#include <csignal>
#include <chrono>
#include <atomic>
#include <cmath>
#include <system_error>

#include <variant>

#include <iostream>
#include <thread>

#include <stdio.h>
#include <filesystem>

#include <ostream>
#include <fstream>


int main() {
try {
	using namespace curan::communication;
	asio::io_context io_context;

	using namespace curan::ui;
	IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context),2200,1800 };
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

	auto button1 = Button::make("Parent!",resources);
	button1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorRED).set_size(SkRect::MakeWH(300, 300));

	auto parent = std::make_unique<curan::process::ProcessLaucher>(io_context, std::chrono::milliseconds(100),std::chrono::milliseconds(0), 10,[button_ptr = button1.get()]
		(bool val){
			button_ptr->set_waiting_color(SK_ColorRED);
		});

	button1->add_press_call([parent_ptr = parent.get()](Button* button, Press press,ConfigDraw* config) {
		if(!parent_ptr->handles)
			parent_ptr->async_lauch_process([button](bool sucess) { if(sucess) button->set_waiting_color(SK_ColorGREEN); }, CURAN_BINARY_LOCATION"/child_with_gui" CURAN_BINARY_SUFFIX);
		else 
			parent_ptr->async_terminate(std::chrono::milliseconds(300),[button](){button->set_waiting_color(SK_ColorRED);});
		
	});

	auto widgetcontainer =  Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(button1);

	widgetcontainer->set_color(SK_ColorBLACK);
	auto page = Page{std::move(widgetcontainer),SK_ColorBLACK};
	page.update_page(viewer.get());

	ConfigDraw config_draw{&page};

	viewer->set_minimum_size(page.minimum_size());	

	std::thread th{[&](){ 
		io_context.run();
		std::cout << "terminating IO context\n";
	}};

	while (!glfwWindowShouldClose(viewer->window) && !io_context.stopped()) {
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
			page.propagate_signal(signals.back(), &config_draw);
		page.propagate_heartbeat(&config_draw);
		glfwPollEvents();

		bool val = viewer->swapBuffers();
		if (!val)
			std::cout << "failed to swap buffers\n";
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	std::cout << "requested signal termination\n";
	io_context.post([&](){io_context.stop();});
	th.join();
	std::cout << "thread joined!\n";
}
catch (std::exception& e) {
	std::cout << "exception thrown : " << e.what() << std::endl;
	return 1;
}
catch (...) {
	std::cout << "exception thrown : " << std::endl;
	return 1;
}
return 0;
}
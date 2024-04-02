#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoProcHandler.h"
#include <thread>
#include <csignal>
#include <chrono>
#include <atomic>
#include <cmath>
#include <csignal>
#include <system_error>

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
#include <variant>

#include <iostream>
#include <thread>

#include <ostream>
#include <fstream>

#include "processmanagement/ChildProcess.h"

asio::io_context* ptr_ctx = nullptr;

void signal_handler(int signal){	
	if (ptr_ctx) ptr_ctx->stop();
}

int viewer_code(asio::io_context& io_context) {
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

	    auto button1 = Button::make("Child!",resources);
	    button1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(300, 300));

	    auto widgetcontainer =  Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	    *widgetcontainer << std::move(button1);

		widgetcontainer->set_color(SK_ColorBLACK);
		auto page = Page{std::move(widgetcontainer),SK_ColorBLACK};
		page.update_page(viewer.get());

		ConfigDraw config_draw{ &page};

		viewer->set_minimum_size(page.minimum_size());

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
		return 0;
	}
	catch (std::exception& e) {
		return 1;
	}
}

int main(int argv, char* argc[]) {
	try {
		std::unique_ptr<curan::process::ChildProcess> child;
		using namespace curan::communication;
		std::signal(SIGINT, signal_handler);
		asio::io_context io_context;
		ptr_ctx = &io_context;


		if(argv<0 || argv>2){
			std::cout << "the supplied arguments are too many, \nthe program either accepts the one or two arguments\n";
		} else if(argv==2) {
			std::cout << "the arguments are: \n";
			std::cout << "program path: " <<  argc[0];
			std::cout << "\nport filename" << argc[1];
			child = std::make_unique<curan::process::ChildProcess>(io_context, std::chrono::milliseconds(100), 10);
		} else {
			assert(argv==1);
			std::cout << "the arguments are: \n";
			std::cout << "program path:" <<  argc[0];
		}

		std::thread th{[&](){ 
			viewer_code(io_context);
		}
		};
		io_context.run();
		std::cout << "stopped running file program path:" <<  argc[0];
		std::raise(SIGINT);
		std::cout << "lauching signal!\n";
		th.join();
	}
	catch (std::exception& e) {
		return 1;
	}
	catch (...) {
		return 1;
	}
	return 0;
}
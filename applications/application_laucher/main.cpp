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
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>
#include <csignal>

#include <iostream>
#include <thread>
#include <boost/process.hpp>

asio::io_context* ptr_ctx = nullptr;

std::unique_ptr<boost::process::child> child_process;
boost::process::ipstream out;
std::atomic<bool> signal_untriggered = true;


constexpr auto waiting_color_active = SkColorSetARGB(120, 0, 255, 0);
constexpr auto waiting_color_inactive = SkColorSetARGB(120, 255, 0, 0);

void signal_handler(int signal)
{
	if (child_process) child_process->terminate();
	child_process = nullptr;
	signal_untriggered.store(false,std::memory_order_relaxed);
}

int main() {
	try {
		using namespace curan::ui;
		std::signal(SIGINT, signal_handler);
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context)};
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

	    auto button1 = Button::make("Temporal Calibration",resources);
	    button1->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(waiting_color_inactive)
                .set_size(SkRect::MakeWH(300, 300));
		button1->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!child_process){
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/TemporalCalibration" CURAN_BINARY_SUFFIX, boost::process::std_out > out);
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
			}
		});

	    auto button2 = Button::make("Spatial Calibration",resources);
	   	button2->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(waiting_color_inactive)
                .set_size(SkRect::MakeWH(300, 300));
		button2->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!child_process){
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/Ultrasoundcalibration" CURAN_BINARY_SUFFIX, boost::process::std_out > out);
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
			}
		});

	    auto button3 = Button::make("Volume ROI",resources);
	   	button3->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(waiting_color_inactive)
                .set_size(SkRect::MakeWH(300, 300));
		button3->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!child_process){
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/RealTimeRobotBoxSpecifier" CURAN_BINARY_SUFFIX, boost::process::std_out > out);
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
			}
		});

		auto button4 = Button::make("Reconstruction",resources);
	   	button4->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(waiting_color_inactive)
                .set_size(SkRect::MakeWH(300, 300));
		button4->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!child_process){
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/RealTimeRobotReconstructor" CURAN_BINARY_SUFFIX, boost::process::std_out > out);
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
			}
		});

		auto button5 = Button::make("Registration",resources);
	   	button5->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(waiting_color_inactive)
                .set_size(SkRect::MakeWH(300, 300));
		button5->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!child_process){
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/RoboticRegistration" CURAN_BINARY_SUFFIX, boost::process::std_out > out);
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
			}
		});

		auto button6 = Button::make("Neuro Navigation",resources);
	   	button6->set_click_color(SK_ColorDKGRAY)
                .set_hover_color(SK_ColorLTGRAY)
                .set_waiting_color(waiting_color_inactive)
                .set_size(SkRect::MakeWH(300, 300));
		button6->add_press_call([&](Button* inbut,Press pres, ConfigDraw* config){
			if(!child_process){
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/InteroperativeNavigation" CURAN_BINARY_SUFFIX, boost::process::std_out > out);
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
			}
		});

		auto image = resources.get_icon("human_robotics_logo.jpeg");
		std::unique_ptr<Container> widgetcontainer;
		if(image)
	    	widgetcontainer = Container::make(Container::ContainerType::VARIABLE_CONTAINER,Container::Arrangement::VERTICAL,*image);
        else 
			widgetcontainer = Container::make(Container::ContainerType::VARIABLE_CONTAINER,Container::Arrangement::VERTICAL);

	    *widgetcontainer << std::move(button1) 
                        << std::move(button2) 
                        << std::move(button3) 
                        << std::move(button4) 
                        << std::move(button5) 
                        << std::move(button6);

		widgetcontainer->set_variable_layout({SkRect::MakeXYWH(0.0,0.0,0.3333,0.5),SkRect::MakeXYWH(0.3332,0.0,0.3333,0.5),SkRect::MakeXYWH(0.6665,0.0,0.3333,0.5),
											  SkRect::MakeXYWH(0.0,0.5,0.3333,0.5),SkRect::MakeXYWH(0.3332,0.5,0.3333,0.5),SkRect::MakeXYWH(0.6665,0.5,0.3333,0.5)});

		widgetcontainer->set_color(SK_ColorBLACK);
		auto page = Page{std::move(widgetcontainer),SK_ColorBLACK};
		page.update_page(viewer.get());

		ConfigDraw config_draw{ &page};
		config_draw.stack_page->stack(Loader::make("human_robotics_logo.jpeg",resources));

		viewer->set_minimum_size(page.minimum_size());

		while (!glfwWindowShouldClose(viewer->window) && signal_untriggered.load(std::memory_order_relaxed)) {
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
		std::cout << "Failed: " << e.what() << std::endl;
		return 1;
	}
}
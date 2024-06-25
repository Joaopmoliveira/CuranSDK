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
#include "utils/TheadPool.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>
#include <csignal>

#include <iostream>
#include <thread>
#include <boost/process.hpp>

boost::asio::io_context asio_ctx;

std::unique_ptr<boost::process::child> child_process;
boost::process::ipstream child_out;
std::stringstream child_stream;

#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
std::unique_ptr<boost::process::child> plus_process;
boost::process::ipstream plus_out;
std::stringstream plus_stream;
std::atomic<bool> plus_active;
#endif

std::atomic<bool> signal_untriggered = true;

constexpr auto waiting_color_active = SkColorSetARGB(70, 0, 255, 0);
constexpr auto waiting_color_inactive = SkColorSetARGB(70, 255, 0, 0);

void signal_handler(int signal)
{
	if (child_process && child_process->running())
		child_process->terminate();
	child_process = nullptr;
	signal_untriggered.store(false, std::memory_order_relaxed);
}

int main()
{
	try
	{
		std::error_code ec;
		using namespace curan::ui;
		auto pool = curan::utilities::ThreadPool::create(2);
		std::signal(SIGINT, signal_handler);
		IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();
		;
		DisplayParams param{std::move(context)};
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		Button *path_planning_button = nullptr;
		Button *temporal_calibration_button = nullptr;
		Button *ultrasound_calibration_button = nullptr;
		Button *real_time_reconstructor_button = nullptr;

		auto button1 = Button::make("Path Planning", resources);
		button1->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 300));
		button1->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config)
								{
			if(!child_process){
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/VolumetricPathPlanning" CURAN_BINARY_SUFFIX, 
                                boost::process::std_out > child_out,
                                ec,
                                asio_ctx,
        			boost::process::on_exit([&](int exit, const std::error_code& ec_in){
						if(exit)
            				std::cout << "VolumetricPathPlanning failure" << ec_in.message() << std::endl;
						inbut->set_waiting_color(waiting_color_inactive);
						child_process->terminate();
						child_process = nullptr;
    				}));
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
			} });

		auto button2 = Button::make("Temporal Calibration", resources);
		button2->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 300));
		button2->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config)
								{
			if(!child_process){
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
				plus_process = std::make_unique<boost::process::child>(CURAN_PLUS_EXECUTABLE_PATH,
																	   "--config-file=" CURAN_COPIED_RESOURCE_PATH"/plus_config/wire_reconstructions/wire_reconstruction_recording/wire_reconstruction_recording/RecordingTest_config.xml",
																	   "--verbose=1", 
																	   boost::process::std_out > plus_out,
																		asio_ctx,
																		[&](int exit, const std::error_code& ec_in){
				if(exit)
					std::cout << "Plus failure" << ec_in.message() << std::endl;
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
				plus_process->terminate();
				plus_process = nullptr;

																		});
#endif
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/TemporalCalibration" CURAN_BINARY_SUFFIX, 
                                boost::process::std_out > child_out,
                                ec,
                                asio_ctx,
        			boost::process::on_exit([&](int exit, const std::error_code& ec_in){
				if(exit)
					std::cout << "TemporalCalibration failure" << ec_in.message() << std::endl;
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
#ifdef CURAN_PLUS_EXECUTABLE_PATH 
				plus_process->terminate();
				plus_process = nullptr;
#endif
    				}));
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
				plus_process->terminate();
				plus_process = nullptr;
#endif
			} });

		auto button3 = Button::make("Spatial Calibration", resources);
		button3->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 300));
		button3->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config)
								{
			if(!child_process){
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
				plus_process = std::make_unique<boost::process::child>(CURAN_PLUS_EXECUTABLE_PATH,
																	   "--config-file=" CURAN_COPIED_RESOURCE_PATH"/plus_config/wire_reconstructions/wire_reconstruction_recording/wire_reconstruction_recording/RecordingTest_config.xml",
																	   "--verbose=1", 
																	   boost::process::std_out > plus_out,
																		asio_ctx,
																		[&](int exit, const std::error_code& ec_in){
				if(exit)
					std::cout << "Plus failure" << ec_in.message() << std::endl;
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
				plus_process->terminate();
				plus_process = nullptr;
																		});
#endif
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/Ultrasoundcalibration" CURAN_BINARY_SUFFIX, 
                                boost::process::std_out > child_out,
                                ec,
                                asio_ctx,
        			boost::process::on_exit([&](int exit, const std::error_code& ec_in){
				if(exit)
					std::cout << "Ultrasoundcalibration failure" << ec_in.message() << std::endl;
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
#ifdef CURAN_PLUS_EXECUTABLE_PATH 
				plus_process->terminate();
				plus_process = nullptr;
#endif
    				}));
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
				plus_process->terminate();
				plus_process = nullptr;
#endif
			} });

		auto button4 = Button::make("Reconstruction", resources);
		button4->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 300));
		button4->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config)
								{
			if(!child_process){
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
				plus_process = std::make_unique<boost::process::child>(CURAN_PLUS_EXECUTABLE_PATH,
																	   "--config-file=" CURAN_COPIED_RESOURCE_PATH"/plus_config/wire_reconstructions/wire_reconstruction_recording/wire_reconstruction_recording/RecordingTest_config.xml",
																	   "--verbose=1", 
																	   boost::process::std_out > plus_out,
																		asio_ctx,
																		[&](int exit, const std::error_code& ec_in){
				if(exit)
					std::cout << "Plus failure" << ec_in.message() << std::endl;
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
				plus_process->terminate();
				plus_process = nullptr;
																		});
#endif
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/RealTimeReconstructor" CURAN_BINARY_SUFFIX, 
                                boost::process::std_out > child_out,
                                ec,
                                asio_ctx,
        			boost::process::on_exit([&](int exit, const std::error_code& ec_in){
				if(exit)
					std::cout << "RealTimeReconstructor failure" << ec_in.message() << std::endl;
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
#ifdef CURAN_PLUS_EXECUTABLE_PATH 
				plus_process->terminate();
				plus_process = nullptr;
#endif
    				}));
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
				plus_process->terminate();
				plus_process = nullptr;
#endif
			} });

		auto button5 = Button::make("Neuro Navigation", resources);
		button5->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 300));
		button5->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config)
								{
			if(!child_process){
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
				plus_process = std::make_unique<boost::process::child>(CURAN_PLUS_EXECUTABLE_PATH,
																	   "--config-file=" CURAN_COPIED_RESOURCE_PATH"/plus_config/wire_reconstructions/wire_reconstruction_recording/wire_reconstruction_recording/RecordingTest_config.xml",
																	   "--verbose=1", 
																	   boost::process::std_out > plus_out,
																		asio_ctx,
																		[&](int exit, const std::error_code& ec_in){
				if(exit)
					std::cout << "Plus failure" << ec_in.message() << std::endl;
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
				plus_process->terminate();
				plus_process = nullptr;
																		});
#endif
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/InteroperativeNavigation" CURAN_BINARY_SUFFIX, 
                                boost::process::std_out > child_out,
                                ec,
                                asio_ctx,
        			boost::process::on_exit([&](int exit, const std::error_code& ec_in){
				if(exit)
					std::cout << "InteroperativeNavigation failure" << ec_in.message() << std::endl;
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
#ifdef CURAN_PLUS_EXECUTABLE_PATH 
				plus_process->terminate();
				plus_process = nullptr;
#endif
    				}));
				inbut->set_waiting_color(waiting_color_active);
			} else {
				inbut->set_waiting_color(waiting_color_inactive);
				child_process->terminate();
				child_process = nullptr;
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
				plus_process->terminate();
				plus_process = nullptr;
#endif
			} });

		auto image = resources.get_icon("human_robotics_logo.jpeg");
		std::unique_ptr<Container> widgetcontainer;
		if (image)
			widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL, *image);
		else
			widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);

		*widgetcontainer << std::move(button1)
						 << std::move(button2)
						 << std::move(button3)
						 << std::move(button4)
						 << std::move(button5);

		widgetcontainer->set_color(SK_ColorBLACK);
		auto page = Page{std::move(widgetcontainer), SK_ColorBLACK};
		page.update_page(viewer.get());

		pool->submit(curan::utilities::Job{"process incoming messages from output stream", [&]()
										   {
											   asio_ctx.run();
										   }});

		ConfigDraw config_draw{&page};
		config_draw.stack_page->stack(Loader::make("human_robotics_logo.jpeg", resources));

		viewer->set_minimum_size(page.minimum_size());

		while (!glfwWindowShouldClose(viewer->window) && signal_untriggered.load(std::memory_order_relaxed))
		{
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface *pointer_to_surface = viewer->getBackbufferSurface();

			SkCanvas *canvas = pointer_to_surface->getCanvas();
			if (viewer->was_updated())
			{
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
		plus_out.close();
		asio_ctx.stop();
		return 0;
	}
	catch (std::exception &e)
	{
		std::cout << "Failed: " << e.what() << std::endl;
		return 1;
	}
}
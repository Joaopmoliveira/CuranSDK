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
#include "userinterface/widgets/TaskManager.h"
#include "utils/TheadPool.h"
#include "utils/Logger.h"
#include "utils/FileStructures.h"
#include "utils/Overloading.h"
#include <variant>
#include <csignal>

#include <iostream>
#include <boost/process.hpp>
#include <boost/asio/read_until.hpp>

constexpr auto waiting_color_active = SkColorSetARGB(70, 0, 255, 0);
constexpr auto waiting_color_inactive = SkColorSetARGB(70, 255, 0, 0);

class Application;

std::atomic<curan::ui::TaskManager*> manager = nullptr;

class PendinAsyncData : public std::enable_shared_from_this<PendinAsyncData>
{
	static size_t identifier;
	volatile bool in_use = false;
	boost::asio::io_context &asio_ctx;

	std::string executable_name = ">>"; 

	std::unique_ptr<boost::process::child> child_process;
	std::error_code child_ec;
	boost::process::async_pipe child_out;
	boost::asio::streambuf child_buf;

#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
	std::unique_ptr<boost::process::child> plus_process;
	boost::asio::streambuf plus_buf;
	std::error_code plus_ec;
	boost::process::async_pipe plus_out;
#endif

	Application *parent = nullptr;

#ifdef CURAN_PLUS_EXECUTABLE_PATH
	void post_async_plus_read()
	{
		auto plus_asyn_read_callback = [async_data = getptr()](const boost::system::error_code &ec, std::size_t size)
		{
			static std::string line;
			static std::istream istr(&async_data->plus_buf);
			if (!ec){
				std::getline(istr, line);
				if(manager) 
					manager.load()->set_mainbody("plus >> " + line);
				else 
					std::cout << "plus >> " << line << std::endl;
			}
			if (!async_data->in_use){
				if(manager) 
					manager.load()->set_mainbody("plus >> stopping..." + line);
				else 
					std::cout << "plus >> stopping..." << line << std::endl;
				return;
			}
			if (!ec)
				async_data->post_async_plus_read();
		};
		boost::asio::async_read_until(plus_out,plus_buf,'\n',plus_asyn_read_callback);
	}
#endif
	void post_async_read()
	{
		auto child_async_read_callback = [async_data = getptr()](const boost::system::error_code &ec, std::size_t size)
		{
			static std::string line;
			static std::istream istr(&async_data->child_buf);
			if (!ec){
				std::getline(istr, line);
				if(manager)  manager.load()->set_mainbody(async_data->executable_name + line);
				else std::cout << async_data->executable_name << line << std::endl;
			}
			if (!async_data->in_use){
				if(manager) manager.load()->set_mainbody(async_data->executable_name + " stopping... " + line);
				else std::cout << async_data->executable_name << " stopping... " << line << std::endl;
				return;
			}
			if (!ec)
				async_data->post_async_read();
		};
		boost::asio::async_read_until(child_out,child_buf,'\n',child_async_read_callback);
	}

	void launch_child(const std::string &executable)
	{
		child_process = std::make_unique<boost::process::child>(std::string{CURAN_BINARY_LOCATION "/"} + executable + std::string{CURAN_BINARY_SUFFIX},
																boost::process::std_out > child_out,
																child_ec,
																asio_ctx,
																boost::process::on_exit([async_data = getptr(), in_executable = executable_name](int exit, const std::error_code &ec_in)
																						{
						if(exit){
							if(manager) manager.load()->set_mainbody(in_executable+"failure\n" + ec_in.message());
							else std::cout << in_executable << " failure\n" << ec_in.message() << std::endl;
						}
						async_data->terminate_all(); }));
		if(manager) manager.load()->set_appendix("running").set_mainbody("lauching app: "+executable);
	}

#ifdef CURAN_PLUS_EXECUTABLE_PATH
	void launch_plus()
	{
		plus_process = std::make_unique<boost::process::child>(CURAN_PLUS_EXECUTABLE_PATH,
															   std::string{"--config-file="} + std::string{CURAN_COPIED_RESOURCE_PATH "/plus_config/plus_spacial_calib_robot_xml/robot_image.xml"},
															   "--verbose=1",
															   boost::process::std_out > plus_out,
															   plus_ec,
															   asio_ctx,
															   boost::process::on_exit([async_data = getptr()](int exit, const std::error_code &ec_in)
																					   {
																						   if (exit){
																							if(manager) manager.load()->set_mainbody("Plus failure" + ec_in.message());
																							else std::cout << "Plus failure" << ec_in.message() << std::endl;
																						   }
																						   async_data->terminate_all(); }));

		 if(manager) manager.load()->set_appendix("running").set_mainbody("lauching plus server");
	}																		
#endif

	
	bool launch_all(const std::string &executable, bool all = true)
	{
		if (in_use)
			return false;
		in_use = true;
		launch_child(executable);
#ifdef CURAN_PLUS_EXECUTABLE_PATH
		if (all)
			launch_plus();
#endif
		return true;
	}
	struct Private
	{
		explicit Private() = default;
	};

public:
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
	PendinAsyncData(Private, boost::asio::io_context &in_asio_ctx, const std::string &executable, Application *in_parent, bool all = true) : executable_name{executable} , child_out{in_asio_ctx}, plus_out{in_asio_ctx}, asio_ctx{in_asio_ctx}, parent{in_parent}
	{
		executable_name = executable + " >> ";
		++identifier;
		if (!parent)
			throw std::runtime_error("parent must be specified");
	}
#else
	PendinAsyncData(Private, boost::asio::io_context &in_asio_ctx, const std::string &executable, Application *in_parent, bool all = true) : executable_name{executable} , asio_ctx{in_asio_ctx}, child_out{in_asio_ctx}, parent{in_parent}
	{
		executable_name = executable + " >> ";
		++identifier;
		if (!parent)
			throw std::runtime_error("parent must be specified");
	}
#endif

	~PendinAsyncData()
	{
		terminate_all();
	}

	std::shared_ptr<PendinAsyncData> getptr()
	{
		return shared_from_this();
	}

	static std::shared_ptr<PendinAsyncData> make(boost::asio::io_context &asio_ctx, const std::string &executable, Application *in_parent, bool all = true)
	{
		auto shared_async_resource = std::make_shared<PendinAsyncData>(Private(), asio_ctx, executable, in_parent, all);
		shared_async_resource->launch_all(executable, all);
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
		if(all)
			shared_async_resource->post_async_plus_read();
#endif
		shared_async_resource->post_async_read();
		return shared_async_resource;
	}
	void terminate_all();
};

size_t PendinAsyncData::identifier = 0;

std::unique_ptr<curan::ui::Overlay> warning_overlay(const std::string &warning,curan::ui::IconResources& resources)
{
    using namespace curan::ui;
    auto warn = Button::make(" ", "warning.png", resources);
    warn->set_click_color(SK_AlphaTRANSPARENT)
        .set_hover_color(SK_AlphaTRANSPARENT)
        .set_waiting_color(SK_AlphaTRANSPARENT)
        .set_size(SkRect::MakeWH(400, 200));

    auto button = Button::make(warning, resources);
    button->set_click_color(SK_AlphaTRANSPARENT)
        .set_hover_color(SK_AlphaTRANSPARENT)
        .set_waiting_color(SK_AlphaTRANSPARENT)
        .set_size(SkRect::MakeWH(200, 50));

    auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
    *viwers_container << std::move(warn) << std::move(button);
    viwers_container->set_color(SK_ColorTRANSPARENT).set_divisions({0.0, .8, 1.0});

    return Overlay::make(std::move(viwers_container), SkColorSetARGB(10, 125, 125, 125), true);
}

class Application
{
	curan::ui::Button *ptr_button1 = nullptr;
	curan::ui::Button *ptr_button2 = nullptr;
	curan::ui::Button *ptr_button3 = nullptr;
	curan::ui::Button *ptr_button4 = nullptr;
	curan::ui::Button *ptr_button5 = nullptr;
	curan::ui::Button *ptr_button6 = nullptr;
	curan::ui::Button *ptr_button7 = nullptr;

	std::shared_ptr<PendinAsyncData> pending_task = nullptr;

	boost::asio::io_context asio_ctx;
	std::shared_ptr<curan::utilities::ThreadPool> pool;

	curan::ui::IconResources &resources;

	std::unique_ptr<curan::ui::Page> page = nullptr;

public:
	Application(curan::ui::IconResources &in_resources) : resources{in_resources}
	{
		page = std::make_unique<curan::ui::Page>(create_main_widget_container(), SK_ColorBLACK);
		pool = curan::utilities::ThreadPool::create(1);
		pool->submit(curan::utilities::Job{"running asio context", [&]()
										   {
											   try
											   {
												   auto work = boost::asio::make_work_guard(asio_ctx);
												   asio_ctx.run();
											   }
											   catch (...)
											   {
												   std::cout << "exception thrown inside asio context...\n";
											   }
										   }});
	}

	~Application()
	{
		terminate_all();
		asio_ctx.stop();
	}

	std::unique_ptr<curan::ui::Container> create_main_widget_container()
	{
		auto button1 = curan::ui::Button::make("Path Planning","medicalviewer.png",resources);
		ptr_button1 = button1.get();
		button1->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 150));
		button1->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config)
								{
			if(!launch_all("SurgicalPlanning",false)){
				terminate_all();
				inbut->set_waiting_color(waiting_color_inactive);
			}
			else{
				inbut->set_waiting_color(waiting_color_active);
			} });

		auto button2 = curan::ui::Button::make("Temporal Calibration","ultrasound_validation.png", resources);
		ptr_button2 = button2.get();
		button2->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 150));
		button2->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config)
								{
			// 1 . check_if_temporalcalibration_arguments_are_valid();
			if(!launch_all("TemporalCalibration")){
				terminate_all();
				inbut->set_waiting_color(waiting_color_inactive);
			}
			else{
				inbut->set_waiting_color(waiting_color_active);
			} });

		auto button3 = curan::ui::Button::make("Spatial Calibration","ultrasound_validation.png", resources);
		ptr_button3 = button3.get();
		button3->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 150));
		button3->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config)
								{
			// 1 . check_if_ultrasoundcalibration_arguments_are_valid();
			if(!launch_all("Ultrasoundcalibration")){
				terminate_all();
				inbut->set_waiting_color(waiting_color_inactive);
			}
			else{
				inbut->set_waiting_color(waiting_color_active);
			} });

		auto button4 = curan::ui::Button::make("Volume Scanning","volumereconstruction.png", resources);
		ptr_button4 = button4.get();
		button4->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 150));
		button4->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config)
								{
			try{
				curan::utilities::UltrasoundCalibrationData calibration{CURAN_COPIED_RESOURCE_PATH "/spatial_calibration.json"};
			} catch(...){
				config->stack_page->stack(warning_overlay("Ultrasound calibration not available",resources));
				return;
			}

			try{
				curan::utilities::TrajectorySpecificationData trajectory_data{CURAN_COPIED_RESOURCE_PATH "/trajectory_specification.json"};
			} catch(...){
				config->stack_page->stack(warning_overlay("Pre-operative image not available",resources));
				return;
			}

			if(!launch_all("RealTimeReconstructor")){
				terminate_all();
				inbut->set_waiting_color(waiting_color_inactive);
			}
			else{
				inbut->set_waiting_color(waiting_color_active);
			} });

		auto button6 = curan::ui::Button::make("Surface Scanning","surfaceconstruction.png", resources);
		ptr_button6 = button6.get();
		button6->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 150));
		button6->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config)
									{
			// 1 . check_if_realtimereconstructor_arguments_are_valid();
			try{
				curan::utilities::UltrasoundCalibrationData calibration{CURAN_COPIED_RESOURCE_PATH "/spatial_calibration.json"};
			} catch(...){
				config->stack_page->stack(warning_overlay("Ultrasound calibration not available",resources));
				return;
			}


			try{
				curan::utilities::TrajectorySpecificationData trajectory_data{CURAN_COPIED_RESOURCE_PATH "/trajectory_specification.json"};
			} catch(...){
				config->stack_page->stack(warning_overlay("Pre-operative image not available",resources));
				return;
			}

			if(!launch_all("SurfaceExtraction")){
				terminate_all();
				inbut->set_waiting_color(waiting_color_inactive);
			}
			else{
				inbut->set_waiting_color(waiting_color_active);
			} });

		auto button5 = curan::ui::Button::make("Neuro Navigation","biopsyviewer.png", resources);
		ptr_button5 = button5.get();
		button5->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_font_size(20)
			.set_size(SkRect::MakeWH(300, 150));
		button5->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config)
								{
			try{
				curan::utilities::UltrasoundCalibrationData calibration{CURAN_COPIED_RESOURCE_PATH "/spatial_calibration.json"};
			} catch(...){
				config->stack_page->stack(warning_overlay("Ultrasound calibration not available",resources));
				return;
			}

			try{
				curan::utilities::TrajectorySpecificationData trajectory_data{CURAN_COPIED_RESOURCE_PATH "/trajectory_specification.json"};
			} catch(...){
				config->stack_page->stack(warning_overlay("Pre-operative image not available",resources));
				return;
			}

			try{
				curan::utilities::RegistrationData trajectory_data{CURAN_COPIED_RESOURCE_PATH "/registration_specification.json"};
			} catch(...){
				config->stack_page->stack(warning_overlay("Intra-operative Navigation not available",resources));
				return;
			}

			if(!launch_all("InteroperativeNavigation")){
				terminate_all();
				inbut->set_waiting_color(waiting_color_inactive);
			}
			else{
				inbut->set_waiting_color(waiting_color_active);
			} });

		auto registrationcontainer = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::VERTICAL);
		*registrationcontainer << std::move(button4) << std::move(button6);
		auto widgetcontainer = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::HORIZONTAL);
		*widgetcontainer << std::move(button1)
						 << std::move(button2)
						 << std::move(button3)
						 << std::move(registrationcontainer)
						 << std::move(button5);

		auto lablogo = resources.get_icon("hrtransparent.png");
		auto tecnicologo = resources.get_icon("IST_A_RGB_POS.png");
		if(lablogo && tecnicologo){
			auto pagelayout = curan::ui::Container::make(curan::ui::Container::ContainerType::VARIABLE_CONTAINER, curan::ui::Container::Arrangement::UNDEFINED);
			auto logos = curan::ui::Container::make(curan::ui::Container::ContainerType::VARIABLE_CONTAINER, curan::ui::Container::Arrangement::UNDEFINED);
			auto imagelablogo = curan::ui::ImageDisplay::make();
			imagelablogo->print_only_image(true)
					      .update_image(*lablogo);
			auto imagetecnicologo = curan::ui::ImageDisplay::make();
			imagetecnicologo->print_only_image(true)
						  .update_image(*tecnicologo);
			*pagelayout << std::move(imagelablogo) << std::move(imagetecnicologo) << std::move(widgetcontainer);
			pagelayout->set_variable_layout({SkRect::MakeXYWH(0,0,0.1,0.1),SkRect::MakeXYWH(0.9,0,0.1,0.1),SkRect::MakeXYWH(0,0.1,1.0,0.9)});
			pagelayout->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});

			auto tasktracker = curan::ui::TaskManager::make("Curan");
			manager = tasktracker.get();
			tasktracker->set_height(40).set_appendix("waiting");		
			auto container_with_task = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::VERTICAL);
			*container_with_task << std::move(pagelayout) << std::move(tasktracker);
			container_with_task->set_divisions({0.0, 0.98, 1.0});
			return std::move(container_with_task);
		} else {
			auto pagelayout = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::HORIZONTAL);
			*pagelayout << std::move(widgetcontainer);
			pagelayout->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(255, 255, 240)});
			auto tasktracker = curan::ui::TaskManager::make("Curan");
			manager = tasktracker.get();
			tasktracker->set_height(40).set_appendix("waiting");		
			auto container_with_task = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::VERTICAL);
			*container_with_task << std::move(pagelayout) << std::move(tasktracker);
			container_with_task->set_divisions({0.0, 0.98, 1.0});
			return std::move(container_with_task);
		}
	}

	bool launch_all(const std::string &executable, bool all = true)
	{
		if (pending_task)
			return false;
		pending_task = PendinAsyncData::make(asio_ctx, executable,this,all);
		return true;
	}

	void terminate_all()
	{
		if(manager) 
			manager.load()->set_mainbody("terminate all called!").set_appendix("waiting");
		else 
			std::cout << "terminate all called!" << std::endl;
		
		if (pending_task)
			pending_task->terminate_all();
		pending_task = nullptr;
		termination_widget_logic();
	}

	curan::ui::Page *get_page()
	{
		return page.get();
	}

	void warn_terminate_all()
	{
		pending_task = nullptr;
		termination_widget_logic();
	}

	void termination_widget_logic()
	{
		if (ptr_button1)
			ptr_button1->set_waiting_color(waiting_color_inactive);
		if (ptr_button2)
			ptr_button2->set_waiting_color(waiting_color_inactive);
		if (ptr_button3)
			ptr_button3->set_waiting_color(waiting_color_inactive);
		if (ptr_button4)
			ptr_button4->set_waiting_color(waiting_color_inactive);
		if (ptr_button5)
			ptr_button5->set_waiting_color(waiting_color_inactive);
		if (ptr_button6)
			ptr_button6->set_waiting_color(waiting_color_inactive);
		if (ptr_button7)
			ptr_button7->set_waiting_color(waiting_color_inactive);
	}
};

std::atomic<bool> signal_untriggered = true;

void PendinAsyncData::terminate_all()
{
	if (!in_use)
		return;
	in_use = false;
	if (child_process)
		child_process->terminate();
	child_process = nullptr;
	child_out.async_close();
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
	if (plus_process)
		plus_process->terminate();
	plus_process = nullptr;
	plus_out.async_close();
#endif
	parent->warn_terminate_all();
}

void signal_handler(int signal)
{
	signal_untriggered.store(false, std::memory_order_relaxed);
}

int main()
{
	try
	{
		std::signal(SIGINT, signal_handler);
		curan::ui::IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
		std::unique_ptr<curan::ui::Context> context = std::make_unique<curan::ui::Context>();
		curan::ui::DisplayParams param{std::move(context)};
		param.windowName = "Curan:1.0.0";
		std::unique_ptr<curan::ui::Window> viewer = std::make_unique<curan::ui::Window>(std::move(param));
		Application app{resources};
		app.get_page()->update_page(viewer.get());

		curan::ui::ConfigDraw config_draw{app.get_page()};
		viewer->set_minimum_size(app.get_page()->minimum_size());

		while (!glfwWindowShouldClose(viewer->window) && signal_untriggered.load(std::memory_order_relaxed))
		{
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface *pointer_to_surface = viewer->getBackbufferSurface();

			SkCanvas *canvas = pointer_to_surface->getCanvas();
			if (viewer->was_updated())
			{
				app.get_page()->update_page(viewer.get());
				viewer->update_processed();
			}
			app.get_page()->draw(canvas);
			auto signals = viewer->process_pending_signals();

			if (!signals.empty())
				app.get_page()->propagate_signal(signals.back(), &config_draw);
			app.get_page()->propagate_heartbeat(&config_draw);
			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (std::exception &e)
	{
		std::cout << "Failed: " << e.what() << std::endl;
		return 1;
	}
	catch (...)
	{
		std::cout << "exception thrown\n";
		return 1;
	}
}
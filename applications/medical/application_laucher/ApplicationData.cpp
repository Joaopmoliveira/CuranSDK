#include "ApplicationData.h"

static std::atomic<curan::ui::TaskManager*> manager = nullptr;

#ifdef CURAN_PLUS_EXECUTABLE_PATH
	void PendinAsyncData::post_async_plus_read()
	{
		auto plus_asyn_read_callback = [async_data = getptr()](const std::error_code &ec, std::size_t size){
			std::string line;
			if(!(async_data->plus_out.is_open()))
				return;
			std::istream istr(&async_data->plus_buf);
			if (!ec){
				std::getline(istr, line);
                std::cout << "plus >> " << line << std::endl;
				if(manager) 
                    manager.load()->set_mainbody("plus >> " + line);
			} else {
                std::cout << "plus >> stopping... " << line << std::endl;
                if(manager) manager.load()->set_mainbody("plus >> stopping... " + line);
            }
			if (!ec)
				async_data->post_async_plus_read();
		};
		boost::asio::async_read_until(plus_out,plus_buf,'\n',plus_asyn_read_callback);
	}
#endif

	void PendinAsyncData::post_async_read()
	{
		auto child_async_read_callback = [async_data = getptr()](const std::error_code &ec, std::size_t size)
		{
			std::string line;
			if(!(async_data->child_out.is_open()))
				return;
			std::istream istr(&async_data->child_buf);
			if (!ec){
				std::getline(istr, line);
                std::cout << async_data->executable_name << line << std::endl;
				if(manager)  manager.load()->set_mainbody(async_data->executable_name + line);
			} else {
                std::cout << async_data->executable_name << " stopping... " << line << std::endl;   
                if(manager) manager.load()->set_mainbody(async_data->executable_name + " stopping... " + line);
            }
			if (!ec)
				async_data->post_async_read();
		};
		boost::asio::async_read_until(child_out,child_buf,'\n',child_async_read_callback);
	}

	void PendinAsyncData::async_launch_child(const std::string &executable)
	{
        auto termination_logic = [async_data = getptr(), in_executable = executable_name](int exit, const std::error_code &ec_in){
            if(!ec_in){
                if(manager) {manager.load()->set_mainbody(in_executable+"failure\n" + ec_in.message()); std::cout << in_executable << "termination callback - failure\n" << ec_in.message() << std::endl;}
                else std::cout << in_executable << "termination callback - failure\n" << ec_in.message() << std::endl;
            }
            std::cout << "termination callback - error code" << exit << std::endl;
            async_data->parent->register_termination();
            async_data->parent->terminate_all();
        };
        boost::process::child val{std::string{CURAN_BINARY_LOCATION "/"} + executable + std::string{CURAN_BINARY_SUFFIX},
                                    boost::process::std_out > child_out,
                                    boost::process::std_in.null(),
                                    boost::process::std_err.null(),
                                    asio_ctx,
                                    grou,
                                    boost::process::on_exit(termination_logic)};      

		child_process = std::move(val);
		parent->register_launch();
		if(manager) manager.load()->set_appendix("running").set_mainbody("lauching app: "+executable);
	}

#ifdef CURAN_PLUS_EXECUTABLE_PATH
	void PendinAsyncData::async_launch_plus()
	{
        auto termination_logic = [async_data = getptr()](int exit, const std::error_code &ec_in){
            if (!ec_in){
                if(manager) {manager.load()->set_mainbody("Plus failure" + ec_in.message()); std::cout << "termination callback - plus failure" << ec_in.message() << std::endl;}
                else std::cout << "termination callback - plus failure" << ec_in.message() << std::endl;
            }
            std::cout << "termination callback - error code" << exit << std::endl;
            async_data->parent->register_termination();
            async_data->parent->terminate_all();
        };
        boost::process::child val{CURAN_PLUS_EXECUTABLE_PATH,
                                  std::string{"--config-file="} + std::string{CURAN_COPIED_RESOURCE_PATH "/plus_config/plus_spacial_calib_robot_xml/robot_image.xml"},
                                  "--verbose=1",
                                  boost::process::std_out > plus_out,
                                  boost::process::std_in.null(),
                                  boost::process::std_err.null(),
                                  asio_ctx,
                                  plus_grou,
                                  boost::process::on_exit(termination_logic)};    
        plus_process = std::move(val);  
        parent->register_launch();
		if(manager) manager.load()->set_appendix("running").set_mainbody("lauching plus server");
	}																		
#endif

	
	bool PendinAsyncData::async_launch_all(const std::string &executable, bool all)
	{
        if(!parent->terminated()){
            std::cout << "parent not terminanted\n" << std::endl;
        }
        std::cout << "lauching (" << executable << ")" << std::endl;
		async_launch_child(executable);

        std::cout << "register pipe submission" << std::endl;
        post_async_read();
        std::cout << "register pipe aceptance" << std::endl;

#ifdef CURAN_PLUS_EXECUTABLE_PATH
		if (all){
            plus_lauched = true;
            std::cout << "lauching (plus)" << std::endl;
            async_launch_plus();
            std::cout << "register plus pipe submission" << std::endl;
            post_async_plus_read();
            std::cout << "register plus pipe aceptance" << std::endl;
        }
#endif
		return true;
	}

#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
    PendinAsyncData::PendinAsyncData(Private, boost::asio::io_context &in_asio_ctx, const std::string &executable, Application *in_parent, bool all) 
                                                : executable_name{executable} ,asio_ctx{in_asio_ctx} , child_out{in_asio_ctx}, plus_out{in_asio_ctx}, parent{in_parent} , child_process{}
	{
		executable_name = executable + " >> ";
		++identifier;
		if (!parent)
			throw std::runtime_error("parent must be specified");
	}
#else
    PendinAsyncData::PendinAsyncData(Private, boost::asio::io_context &in_asio_ctx, const std::string &executable, Application *in_parent, bool all) 
                                                : executable_name{executable} , asio_ctx{in_asio_ctx}, child_out{in_asio_ctx}, parent{in_parent},child_process{}
	{
		executable_name = executable + " >> ";
		++identifier;
		if (!parent)
			throw std::runtime_error("parent must be specified");
	}
#endif

    PendinAsyncData::~PendinAsyncData()
	{
		async_terminate_all();
	}

	std::shared_ptr<PendinAsyncData> PendinAsyncData::make(boost::asio::io_context &asio_ctx, const std::string &executable, Application *in_parent, bool all)
	{
		auto shared_async_resource = std::make_shared<PendinAsyncData>(Private(), asio_ctx, executable, in_parent, all);
		shared_async_resource->async_launch_all(executable, all);
		return shared_async_resource;
	}

    void PendinAsyncData::async_terminate_all()
    {
        if(terminated_called)
            return ;
        if(child_out.is_open()){
            std::cout << "requested pipe termination" << std::endl;
            child_out.async_close();
        }
        grou.terminate();
            
    #ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
        if(plus_lauched){
            if(plus_out.is_open()){
                std::cout << "requested plus pipe termination" << std::endl;
                plus_out.async_close();
            }
            plus_grou.terminate();
        }
    #endif
        terminated_called = true;
        parent->warn_terminate_all();
    }

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



	Application::Application(curan::ui::IconResources &in_resources) : resources{in_resources},asio_ctx{},work{boost::asio::make_work_guard(asio_ctx)}
	{
		page = std::make_unique<curan::ui::Page>(create_main_widget_container(), SK_ColorBLACK);
		pool = curan::utilities::ThreadPool::create(1);
		pool->submit("running asio context", [&](){
		    try{
				asio_ctx.run();
                std::raise(SIGINT);
			} catch (...) {
                std::raise(SIGINT);
				std::cout << "exception thrown inside asio context...\n";
			}
		});
	}

	Application::~Application()
	{
		terminate_all();
		reset_work();
	}

	void Application::reset_work(){
		work.reset();
	}

	bool Application::terminated(){
		return pending_applications==0;
	}

	std::unique_ptr<curan::ui::Container> Application::create_main_widget_container()
	{
		auto button1 = curan::ui::Button::make("Path Planning","medicalviewer.png",resources);
		ptr_button1 = button1.get();
		button1->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 150));
		button1->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config){
            launch_all("SurgicalPlanning",false);
            inbut->set_waiting_color(waiting_color_active);
		});

		auto button2 = curan::ui::Button::make("Temporal Calibration","ultrasound_validation.png", resources);
		ptr_button2 = button2.get();
		button2->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 150));
		button2->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config){
			// 1 . check_if_temporalcalibration_arguments_are_valid();
            inbut->set_waiting_color(waiting_color_active);
            launch_all("TemporalCalibration");
		});

		auto button3 = curan::ui::Button::make("Spatial Calibration","ultrasound_validation.png", resources);
		ptr_button3 = button3.get();
		button3->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 150));
		button3->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config){
			// 1 . check_if_ultrasoundcalibration_arguments_are_valid();
            inbut->set_waiting_color(waiting_color_active);
            launch_all("Ultrasoundcalibration");
		});

		auto button4 = curan::ui::Button::make("Volume Scanning","volumereconstruction.png", resources);
		ptr_button4 = button4.get();
		button4->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 150));
		button4->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config){
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
            inbut->set_waiting_color(waiting_color_active);
            launch_all("RealTimeReconstructor");
		});

		auto button5 = curan::ui::Button::make("Surface Scanning","surfaceconstruction.png", resources);
		ptr_button5 = button5.get();
		button5->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_size(SkRect::MakeWH(300, 150));
		button5->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config){
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
            inbut->set_waiting_color(waiting_color_active);
            launch_all("SurfaceExtraction");
		});

		auto button6 = curan::ui::Button::make("Neuro Navigation","biopsyviewer.png", resources);
		ptr_button6 = button6.get();
		button6->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_font_size(20)
			.set_size(SkRect::MakeWH(300, 150));
		button6->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config)
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
            launch_all("InteroperativeNavigation",false);
		});

		auto button7 = curan::ui::Button::make("3D Neuro Navigation","biopsyviewer.png", resources);
		ptr_button7 = button7.get();
		button7->set_click_color(SK_ColorDKGRAY)
			.set_hover_color(SK_ColorLTGRAY)
			.set_waiting_color(waiting_color_inactive)
			.set_font_size(20)
			.set_size(SkRect::MakeWH(300, 150));
		button7->add_press_call([&](curan::ui::Button *inbut, curan::ui::Press pres, curan::ui::ConfigDraw *config)
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
				config->stack_page->stack(warning_overlay("3D Intra-operative Navigation not available",resources));
				return;
			}
            launch_all("RealTimeNavigation3D",false);
		});

		auto registrationcontainer = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::VERTICAL);
		*registrationcontainer << std::move(button4) << std::move(button5);
		auto navigationcontainer = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::VERTICAL);
		*navigationcontainer << std::move(button6) << std::move(button7);
		auto widgetcontainer = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER, curan::ui::Container::Arrangement::HORIZONTAL);
		*widgetcontainer << std::move(button1)
						 << std::move(button2)
						 << std::move(button3)
						 << std::move(registrationcontainer)
						 << std::move(navigationcontainer);

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

	bool Application::launch_all(const std::string &executable, bool all)
	{
		if (pending_task)
			return false;
		pending_task = PendinAsyncData::make(asio_ctx, executable,this,all);
        std::cout << "lauching terminated" << std::endl;
		return true;
	}

	void Application::terminate_all()
	{
		if (pending_task){
			if(manager) {
				std::cout << "terminate all called!" << std::endl;
				manager.load()->set_mainbody("terminate all called!").set_appendix("waiting");
			}
			else 
				std::cout << "terminate all called!" << std::endl;
			pending_task->async_terminate_all();
			pending_task = nullptr;
			termination_widget_logic();
		}
	}

	void Application::warn_terminate_all()
	{
		pending_task = nullptr;
		termination_widget_logic();
	}

	void Application::termination_widget_logic()
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


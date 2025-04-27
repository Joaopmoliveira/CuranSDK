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
#include "utils/Overloading.h"
#include <variant>
#include <csignal>

#include <iostream>
#include <boost/process.hpp>
#include <boost/asio/read_until.hpp>

constexpr auto waiting_color_active = SkColorSetARGB(70, 0, 255, 0);
constexpr auto waiting_color_inactive = SkColorSetARGB(70, 255, 0, 0);

std::atomic<curan::ui::TaskManager*> manager = nullptr;

class Application
{
	curan::ui::Button *ptr_button1 = nullptr;
	curan::ui::Button *ptr_button2 = nullptr;
	curan::ui::Button *ptr_button3 = nullptr;
	curan::ui::Button *ptr_button4 = nullptr;
	curan::ui::Button *ptr_button5 = nullptr;
	curan::ui::Button *ptr_button6 = nullptr;
	curan::ui::Button *ptr_button7 = nullptr;

	std::shared_ptr<curan::utilities::ThreadPool> pool;

	curan::ui::IconResources &resources;
	std::unique_ptr<curan::ui::Page> page = nullptr;

public:
	Application(curan::ui::IconResources &in_resources) : resources{in_resources}
	{
		page = std::make_unique<curan::ui::Page>(create_main_widget_container(), SK_ColorBLACK);
	}

	~Application()
	{}

    void folder_dropped(){
        // read folder

        // check if there are dicom files present in folder

        // group series

        // find mha files

        // add items to page 

        // once item is selected then
        
    }

    

	std::unique_ptr<curan::ui::Container> create_main_widget_container()
	{
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

	curan::ui::Page *get_page()
	{
		return page.get();
	}

	void warn_terminate_all()
	{
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
#include "ApplicationData.h"

std::atomic<bool> signal_untriggered = true;

void signal_handler(int signal){
	signal_untriggered.store(false, std::memory_order_relaxed);
}

int main()
{
	try
	{
		std::signal(SIGINT, signal_handler);
		curan::ui::IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
		std::unique_ptr<curan::ui::Context> context = std::make_unique<curan::ui::Context>();
		curan::ui::DisplayParams param{std::move(context),2000,1000};
		param.windowName = "Curan:1.0.0";
		std::unique_ptr<curan::ui::Window> viewer = std::make_unique<curan::ui::Window>(std::move(param));
		Application app{resources};
		curan::ui::ConfigDraw config_draw{app.get_page()};
		viewer->set_minimum_size(app.get_page()->minimum_size());
		app.get_page()->update_page(viewer.get());
		bool request_made = false;
		while ((!glfwWindowShouldClose(viewer->window) && signal_untriggered.load(std::memory_order_relaxed)) || !app.terminated())
		{
			if((glfwWindowShouldClose(viewer->window)|| !signal_untriggered.load(std::memory_order_relaxed)) && !request_made){
				app.reset_work();
				app.terminate_all();
				config_draw.stack_page->stack(warning_overlay("terminating application",resources));
				request_made = true;
			}
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
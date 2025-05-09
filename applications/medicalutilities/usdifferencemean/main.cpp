#define _USE_MATH_DEFINES
#include <cmath>
#include "ceres/ceres.h"
#include "optimization/WireCalibration.h"
#include <nlohmann/json.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "CalibratePages.h"

#include <random>

int main(int argc, char* argv[]) {
	using namespace curan::ui;
	auto shared_pool = curan::utilities::ThreadPool::create(4);

	ConfigurationData data;
	data.shared_pool = shared_pool;
	std::cout << "the received port is: " << data.port << "\n";
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context)};
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
	IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

	std::shared_ptr<ProcessingMessage> processing;
	auto page = create_main_page(data,processing,resources);

	page.update_page(viewer.get());

	ConfigDraw config{&page};
	processing->config_draw = &config;
	
	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
		if (viewer->was_updated()) {
	    	page.update_page(viewer.get());
			viewer->update_processed();
		}
		page.draw(canvas);
        for(auto&& signal : viewer->process_pending_signals())
            page.propagate_signal(signal, &config);
		glfwPollEvents();

		bool val = viewer->swapBuffers();
		if (!val)
			std::cout << "failed to swap buffers\n";
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	processing->attempt_stop();
	std::cout << "trying to stop communication\n" << std::endl;
	return 0;
}
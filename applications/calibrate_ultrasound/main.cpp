#define STB_IMAGE_IMPLEMENTATION

#include "CalibratePages.h"

int main(int argc, char* argv[]) {
	using namespace curan::ui;
	curan::utils::initialize_thread_pool(10);

	ConfigurationData data;
	std::cout << "the received port is: " << data.port << "\n";
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context),2200,1800 };
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

	std::shared_ptr<ProcessingMessage> processing;
	auto page = create_main_page(data,processing);

	auto rec = viewer->get_size();
	page->propagate_size_change(rec);

	int width = rec.width();
	int height = rec.height();

	ConfigDraw config{page.get()};

	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		auto temp_height = pointer_to_surface->height();
		auto temp_width = pointer_to_surface->width();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
		if (temp_height != height || temp_width != width) {
			rec = SkRect::MakeWH(temp_width, temp_height);
			page->propagate_size_change(rec);
		}
		page->draw(canvas);
		auto signals = viewer->process_pending_signals();
		if (!signals.empty())
			page->propagate_signal(signals.back(), &config);
		glfwPollEvents();

		bool val = viewer->swapBuffers();
		if (!val)
			std::cout << "failed to swap buffers\n";
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	processing->attempt_stop();
	std::cout << "trying to stop everything" << std::endl;

	int tasks_n = 0;
	int tasks_queue = 0;
	curan::utils::pool->get_number_tasks(tasks_n, tasks_queue);
	std::cout << "Number of tasks executing: " << tasks_n << " number of tasks in queue" << tasks_queue << "\n";
	curan::utils::terminate_thread_pool();
	std::cout << "Number of frame recordings: " << processing->list_of_recorded_points.size() << "\n";
	std::cout << "Received spacing: \n";
	
	int counter_f = 1;
	for (const auto& f : processing->list_of_recorded_points) {
		int counter_p = 0;
		std::cout << "slice : ";
		for (const auto& p : f) {
			std::cout << "	point(" << counter_p << ") -> (" << p.x << ", " << p.y << ")\n";
			++counter_p;
		}
		++counter_f;
	}

	std::cout << "\n";
	return 0;
}
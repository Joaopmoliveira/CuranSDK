#include "userinterface/Window.h"

int main() {
	using namespace curan::ui;
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context),1200,800 };
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
		canvas->drawColor(SK_ColorWHITE);
		glfwPollEvents();
		viewer->process_pending_signals();
		bool val = viewer->swapBuffers();
		if (!val)
			throw std::runtime_error("failed to swap buffers");
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	return 0;
}
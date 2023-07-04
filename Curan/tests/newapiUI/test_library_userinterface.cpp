#include "modifieduserinterface/Window.h"

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

		SkPaint paint;
		paint.setStyle(SkPaint::kFill_Style);
		paint.setAntiAlias(true);
		paint.setStrokeWidth(4);
		paint.setColor(0xff4285F4);

		SkRect rect = SkRect::MakeXYWH(10, 10, 100, 160);
		canvas->drawRect(rect, paint);

		SkRRect oval;
		oval.setOval(rect);
		oval.offset(40, 80);
		paint.setColor(0xffDB4437);
		canvas->drawRRect(oval, paint);

		paint.setColor(0xff0F9D58);
		canvas->drawCircle(180, 50, 25, paint);

		rect.offset(80, 50);
		paint.setColor(0xffF4B400);
		paint.setStyle(SkPaint::kStroke_Style);
		canvas->drawRoundRect(rect, 10, 10, paint);
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
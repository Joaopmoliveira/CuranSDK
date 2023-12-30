#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ItemExplorer.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include <thread>

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        std::array<unsigned char,100*100> image_buffer;
        constexpr float maximum_size = 2*50.0*50.0;
        for(size_t row = 0; row < 100; ++row)
            for(size_t col = 0; col < 100; ++col)
                image_buffer[row+col*100] = static_cast<unsigned char>((((row-50.0)*(row-50.0)+(col-50.0)*(col-50.0))/maximum_size)*255.0);


        SkImageInfo information = SkImageInfo::Make(100, 100, kGray_8_SkColorType, kOpaque_SkAlphaType);
	    auto pixmap = SkPixmap(information, pixels, 100 * sizeof(unsigned char));
	    auto image_to_display = SkSurfaces::WrapPixels(pixmap)->makeImageSnapshot();

		auto item_explorer = ItemExplorer::make("Touch!",resources);
        {
            Item item;
            item.identifier = 1;
            item.image = image_to_display;
            item.text = "failure";
            item_explorer->add(std::move(item));
        }

        {
            Item item;
            item.identifier = 2;
            item.image = image_to_display;
            item.text = "sucess";
            item_explorer->add(std::move(item));
        }

        {
            Item item;
            item.identifier = 3;
            item.image = image_to_display;
            item.text = "big";
            item_explorer->add(std::move(item));
        }

		SkRect rect = SkRect::MakeXYWH(0, 0, 1000, 700);
		item_explorer->set_position(rect);
		item_explorer->compile();
		item_explorer->add_press_call(callback);

		auto caldraw = item_explorer->draw();
		auto calsignal = item_explorer->call();

		ConfigDraw config_draw;

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);
			caldraw(canvas);
			glfwPollEvents();
			auto signals = viewer->process_pending_signals();
			if (!signals.empty())
				calsignal(signals.back(),&config_draw);
			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch(const std::exception& e){
		std::cout << "Exception thrown:" << e.what() << "\n";
	}
	catch (...) {
		std::cout << "Failed to create window for unknown reason\n";
		return 1;
	}
}
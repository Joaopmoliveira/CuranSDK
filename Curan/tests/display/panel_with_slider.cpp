#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"

#include <unordered_map>
#include <optional>
#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include "userinterface/widgets/ImageWrapper.h"
#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Overloading.h"

class SlidingPanel : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<SlidingPanel>
{
private:

	size_t counter = 0;

	SkColor colbuton = {SK_ColorRED};
	SkPaint paint_square;
	SkPaint bluring_paint;
	SkPaint paint_stroke;
	SkPaint background_paint;
	SkPaint paint_points;

	std::unordered_map<size_t,curan::ui::Stroke> strokes;
	curan::ui::PointCollection current_stroke;

	SkRect background_rect;
	SkMatrix homogenenous_transformation;
	SkMatrix inverse_homogenenous_transformation;

	curan::ui::IconResources& system_icons;
	SkFont text_font;
	std::optional<curan::ui::ImageWrapper> background;

	std::array<float, 3> color_phase_offset;

	bool is_pressed = false;
	bool is_highlighting = false;
	curan::ui::ZoomIn zoom_in;

	SkSamplingOptions options;
	sk_sp<SkImageFilter> imgfilter;


	SlidingPanel(curan::ui::IconResources& other,std::optional<curan::ui::ImageWrapper> image_wrapper){

    }

	void insert_in_map(const curan::ui::PointCollection& future_stroke){

    }


public:

	static std::unique_ptr<SlidingPanel> make(curan::ui::IconResources& other,size_t lower_limit, size_t upper_limit){

    }

	~SlidingPanel()
	{}

	void compile() override{

    }

	void framebuffer_resize(const SkRect& new_page_size) override{

    }

	curan::ui::drawablefunction draw() override{

    }

	curan::ui::callablefunction call() override{

    }
	
};

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1200 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		std::unique_ptr<SlidingPanel> image_display = SlidingPanel::make(resources,get_image());
		Panel* panel_pointer = image_display.get();

		auto button = Button::make("Connect",resources);
		button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));

		auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
		*container << std::move(button) << std::move(image_display);
		container->set_divisions({ 0.0 , 0.1 , 1.0 });

    	curan::ui::Page page{std::move(container),SK_ColorBLACK};

		ConfigDraw config_draw;

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			if (viewer->was_updated()) {
		    	page.update_page(viewer.get());
				viewer->update_processed();
			}
        	page.draw(canvas);
        	auto signals = viewer->process_pending_signals();
        	if (!signals.empty())
            	page.propagate_signal(signals.back(),&config_draw);
        	glfwPollEvents();
    
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
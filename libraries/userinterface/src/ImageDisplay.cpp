#include "userinterface/widgets/ImageDisplay.h"
#include "utils/TheadPool.h"
#include <iostream>
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/ComputeImageBounds.h"

namespace curan {
namespace ui {

ImageDisplay::ImageDisplay()  {

}

std::unique_ptr<ImageDisplay> ImageDisplay::make() {
	std::unique_ptr<ImageDisplay> image_display = std::unique_ptr<ImageDisplay>( new ImageDisplay());
	return image_display;
}

ImageDisplay& ImageDisplay::update_image(ImageWrapper wrapped_image) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	images_to_render = wrapped_image;
	return *(this);
}

drawablefunction ImageDisplay::draw() {
	if(!compiled)
		throw std::runtime_error("cannot query positions while container not compiled");
	auto lamb = [this](SkCanvas* canvas) {
		auto widget_rect = get_position();
		SkRect current_selected_image_rectangle = widget_rect;

		auto image = get_image_wrapper();
		if(image)
			override_image_wrapper(*image);

		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kStroke_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(SK_ColorGREEN);

		if (old_image) {
			auto val = *old_image;
			auto image_display_surface = (*old_image).image;
			current_selected_image_rectangle = compute_bounded_rectangle(widget_rect,image_display_surface->width(),image_display_surface->height());
			if(!f_print_only_image)
				canvas->drawRect(current_selected_image_rectangle,paint_square);
			SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0f / 3.0f, 1.0f / 3.0f });
			std::lock_guard<std::mutex> g{ get_mutex() };
			if(paint_compliant_filtered_image)
				canvas->drawImageRect(image_display_surface, current_selected_image_rectangle, opt,&(*paint_compliant_filtered_image));
			else
				canvas->drawImageRect(image_display_surface, current_selected_image_rectangle, opt);
		}
		auto custom_drawing = get_custom_drawingcall();
		if (custom_drawing) (*custom_drawing)(canvas, current_selected_image_rectangle,widget_rect);
	};
	return lamb;
}

callablefunction ImageDisplay::call() {
	if(!compiled)
		throw std::runtime_error("cannot query positions while container not compiled");
	auto lamb = [this](Signal canvas, ConfigDraw* config) {

		return false;
	};
	return lamb;
}

std::optional<ImageWrapper> ImageDisplay::get_image_wrapper() {
	std::lock_guard<std::mutex> g(get_mutex());
	auto copy = images_to_render;
	images_to_render = std::nullopt;
	return copy;
}

void ImageDisplay::override_image_wrapper(ImageWrapper wrapper) {
	std::lock_guard<std::mutex> g(get_mutex());
	old_image = wrapper;
}

ImageDisplay& ImageDisplay::update_custom_drawingcall(custom_step call) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	custom_drawing_call = call;
	return *(this);
}

ImageDisplay& ImageDisplay::clear_custom_drawingcall() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	custom_drawing_call = std::nullopt;
	return *(this);
}

std::optional<custom_step> ImageDisplay::get_custom_drawingcall() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	return custom_drawing_call;
}

ImageDisplay& ImageDisplay::update_batch(custom_step call, ImageWrapper provider) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	images_to_render = provider;
	custom_drawing_call = call;
	return *(this);
}

void ImageDisplay::compile(){
	compiled = true;
}

ImageDisplay& ImageDisplay::set_color_filter(sk_sp<SkColorFilter> filter){
	SkPaint paint;
	paint.setColorFilter(filter);
	std::lock_guard<std::mutex> g(get_mutex());
	paint_compliant_filtered_image = paint;
	return *(this);
}

}
}
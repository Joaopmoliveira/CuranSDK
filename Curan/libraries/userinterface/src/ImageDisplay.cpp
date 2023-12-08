#include "userinterface/widgets/ImageDisplay.h"
#include "utils/TheadPool.h"
#include <iostream>
#include "userinterface/widgets/ConfigDraw.h"

namespace curan {
namespace ui {

ImageWrapper::ImageWrapper(std::shared_ptr<utilities::MemoryBuffer> in_buffer,size_t width,size_t height,SkColorType color, SkAlphaType opaqueness):
 buffer{in_buffer}{
		image_info = SkImageInfo::Make(width, height,color,opaqueness);
	    size_t row_size = width * sizeof(char);
	    auto wraped_skia_pixmap = SkPixmap{image_info,in_buffer->begin()->data(),row_size};
		image = SkSurfaces::WrapPixels(wraped_skia_pixmap)->makeImageSnapshot();
}

ImageDisplay::ImageDisplay()  {

}

std::unique_ptr<ImageDisplay> ImageDisplay::make() {
	std::unique_ptr<ImageDisplay> image_display = std::unique_ptr<ImageDisplay>( new ImageDisplay());
	return image_display;
}

void ImageDisplay::update_image(ImageWrapper wrapped_image) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	images_to_render = wrapped_image;
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
		canvas->drawRect(widget_rect, paint_square);
		if (image) {
			auto val = *image;
			auto image_display_surface = (*image).image;
			float image_width = image_display_surface->width();
			float image_height = image_display_surface->height();
			float current_selected_width = widget_rect.width();
			float current_selected_height = widget_rect.height();
			float scale_factor = std::min(current_selected_width * 0.9f / image_width, current_selected_height * 0.95f / image_height);
			float init_x = (current_selected_width - image_width * scale_factor) / 2.0f + widget_rect.x();
			float init_y = (current_selected_height - image_height * scale_factor) / 2.0f + widget_rect.y();

			current_selected_image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);

			SkRect testing = widget_rect;
			float init_x_new = 0;
			float init_y_new = 0;
			if (current_selected_width * 0.9f / image_width < current_selected_height * 0.95f / image_height) { // the width is the largest dimension which must be scalled 
				init_x_new = (current_selected_width - image_width * scale_factor) / 2.0f + widget_rect.x();
				init_y_new = (current_selected_height - image_height * scale_factor) / 2.0f + widget_rect.y();
				float local_scalling = current_selected_image_rectangle.height()/image_height;
				testing = SkRect::MakeXYWH(init_x_new, init_y_new, local_scalling * image_width, scale_factor * image_height);
			}
			else { //the height is the largest dimension which must be scalled
				init_x_new = (current_selected_width - image_width * scale_factor) / 2.0f + widget_rect.x();
				init_y_new = (current_selected_height - image_height * scale_factor) / 2.0f + widget_rect.y();


				testing = SkRect::MakeXYWH(init_x_new, init_y_new, scale_factor * image_width, 0.5f*scale_factor * image_height);
			}

			SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0f / 3.0f, 1.0f / 3.0f });
			canvas->drawImageRect(image_display_surface, current_selected_image_rectangle, opt);
			canvas->drawRect(current_selected_image_rectangle, paint_square);
		}

		auto custom_drawing = get_custom_drawingcall();

		if (custom_drawing) {
			auto special = *custom_drawing;
			special(canvas, current_selected_image_rectangle);
		}
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

void ImageDisplay::framebuffer_resize() {

}

std::optional<ImageWrapper> ImageDisplay::get_image_wrapper() {
	std::lock_guard<std::mutex> g(get_mutex());
	return images_to_render;
}

ImageDisplay& ImageDisplay::override_image_wrapper(ImageWrapper wrapper) {
	std::lock_guard<std::mutex> g(get_mutex());
	old_image = wrapper;
	return *(this);
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

}
}
#include "userinterface/widgets/ImageDisplay.h"
#include "utils/TheadPool.h"
#include <iostream>
#include "userinterface/widgets/ConfigDraw.h"

namespace curan {
namespace ui {

ImageDisplay::ImageDisplay(Info& info) : Drawable{SkRect::MakeWH(info.width,info.height)} {
	SkImageInfo image_info = SkImageInfo::Make(info.width,info.height,kRGB_888x_SkColorType, kPremul_SkAlphaType);
	SkSurfaceProps props{ SkSurfaceProps::Flags::kDynamicMSAA_Flag,kRGB_H_SkPixelGeometry};
}

std::shared_ptr<ImageDisplay> ImageDisplay::make(Info& info) {
	return std::make_shared<ImageDisplay>(info);
}

void ImageDisplay::update_image(image_provider provider) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	SkPixmap pixelmap;
	provider(pixelmap);
	auto image = SkImage::MakeFromRaster(pixelmap, nullptr, nullptr);
	auto lam = [image, provider, pixelmap]() {
		return image;
	};
	images_to_render = lam;
}

drawablefunction ImageDisplay::draw() {
	auto lamb = [this](SkCanvas* canvas) {
		auto widget_rect = get_position();
		SkRect current_selected_image_rectangle = widget_rect;

		auto image = get_image_wrapper();
		override_image_wrapper(image);
		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kStroke_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(SK_ColorGREEN);
		canvas->drawRect(widget_rect, paint_square);
		if (image) {
			auto val = *image;
			auto image_display_surface = val();
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


				testing = SkRect::MakeXYWH(init_x_new, init_y_new, scale_factor * image_width, 0.5*scale_factor * image_height);
			}

			SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0 / 3, 1.0 / 3 });
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
	auto lamb = [this](Signal canvas, ConfigDraw* config) {

		return false;
	};
	return lamb;
}

void ImageDisplay::framebuffer_resize() {

}

std::optional<skia_image_producer> ImageDisplay::get_image_wrapper() {
	std::lock_guard<std::mutex> g(get_mutex());
	return images_to_render;
}

void ImageDisplay::override_image_wrapper(std::optional<skia_image_producer> wrapper) {
	std::lock_guard<std::mutex> g(get_mutex());
	old_image = wrapper;
}

void ImageDisplay::update_custom_drawingcall(custom_step call) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	custom_drawing_call = call;
}

void ImageDisplay::clear_custom_drawingcall() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	custom_drawing_call = std::nullopt;
}

std::optional<custom_step> ImageDisplay::get_custom_drawingcall() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	return custom_drawing_call;
}

void ImageDisplay::update_batch(custom_step call, image_provider provider) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	SkPixmap pixelmap;
	provider(pixelmap);
	auto image = SkImage::MakeFromRaster(pixelmap, nullptr, nullptr);
	auto lam = [image, provider, pixelmap]() {
		return image;
	};
	images_to_render = lam;
	custom_drawing_call = call;
}

}
}
#include "userinterface/widgets/ImageDisplay.h"
#include "utils/TheadPool.h"
#include <iostream>

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
	SkPixmap pixelmap;
	provider(pixelmap);
	auto image = SkImage::MakeFromRaster(pixelmap, nullptr, nullptr);
	auto lam = [image, provider, pixelmap]() {
		return image;
	};
	std::lock_guard<std::mutex> g{ get_mutex() };
	images_to_render.push_back(lam);
}

drawablefunction ImageDisplay::draw() {
	auto lamb = [this](SkCanvas* canvas) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		auto widget_rect = get_position();
		if (images_to_render.size() != 0) {
			auto val = images_to_render.front();
			auto image_display_surface = val();

			float image_width = image_display_surface->width();
			float image_height = image_display_surface->height();
			float current_selected_width = widget_rect.width();
			float current_selected_height = widget_rect.height();
			float scale_factor = std::min(current_selected_width * 0.9f / image_width, current_selected_height * 0.95f / image_height);
			float init_x = (current_selected_width - image_width * scale_factor) / 2.0f + widget_rect.x();
			float init_y = (current_selected_height - image_height * scale_factor) / 2.0f + widget_rect.y();

			SkRect current_selected_image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);
			SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0 / 3, 1.0 / 3 });

			canvas->drawImageRect(image_display_surface, current_selected_image_rectangle, opt);

			if (custom_drawing_call) {
				auto special = *custom_drawing_call;
				special(canvas, init_x, init_y);
			}
		}
		if (images_to_render.size() > 1) {
			images_to_render.pop_front();
		}
	};
	return lamb;
}

callablefunction ImageDisplay::call() {
	auto lamb = [this](Signal canvas) {

		return false;
	};
	return lamb;
}

void ImageDisplay::framebuffer_resize() {

}

}
}
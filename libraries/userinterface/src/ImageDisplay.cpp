#include "userinterface/widgets/ImageDisplay.h"
#include "utils/TheadPool.h"
#include <iostream>

namespace curan {
namespace ui {

ImageDisplay::ImageDisplay(Info& info) : Drawable{SkRect::MakeWH(info.width,info.height)} {
	SkImageInfo image_info = SkImageInfo::Make(info.width,info.height,kRGB_888x_SkColorType, kPremul_SkAlphaType);
	SkSurfaceProps props{ SkSurfaceProps::Flags::kDynamicMSAA_Flag,kRGB_H_SkPixelGeometry};
	image_display_surface = SkSurface::MakeRaster(image_info, &props);
	SkCanvas* canvas = image_display_surface->getCanvas();
	canvas->drawColor(SK_ColorWHITE);
}

std::shared_ptr<ImageDisplay> ImageDisplay::make(Info& info) {
	return std::make_shared<ImageDisplay>(info);
}

void ImageDisplay::update_image(image_provider provider) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	auto start = std::chrono::high_resolution_clock::now();
	SkPixmap pixelmap;
	provider(pixelmap);
	auto canvas = image_display_surface->getCanvas();
	auto image = SkImage::MakeFromRaster(pixelmap, nullptr, nullptr);

	SkBitmap loc{};
	loc.installPixels(pixelmap);

	SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0 / 3, 1.0 / 3 });
	float image_width = pixelmap.width();
	float image_height = pixelmap.height();
	float current_selected_width = image_display_surface->width();
	float current_selected_height = image_display_surface->height();

	float scale_factor = (std::min)(current_selected_width * 0.9f / image_width, current_selected_height * 0.95f / image_height);

	float init_x = (current_selected_width - image_width * scale_factor) / 2.0f;
	float init_y = (current_selected_height - image_height * scale_factor) / 2.0f;
	SkRect current_selected_image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);
	canvas->drawImageRect(image, current_selected_image_rectangle, opt);
	auto end = std::chrono::high_resolution_clock::now();
	std::cout << "took internal from suply image: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "\n";
}

drawablefunction ImageDisplay::draw() {
	auto lamb = [this](SkCanvas* canvas) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		auto start = std::chrono::high_resolution_clock::now();
		auto widget_rect = get_position();
		float image_width = image_display_surface->width();
		float image_height = image_display_surface->height();
		float current_selected_width = widget_rect.width();
		float current_selected_height = widget_rect.height();
		float scale_factor = std::min(current_selected_width * 0.9f / image_width, current_selected_height * 0.95f / image_height);
		float init_x = (current_selected_width - image_width * scale_factor) / 2.0f + widget_rect.x();
		float init_y = (current_selected_height - image_height * scale_factor) / 2.0f + widget_rect.y();
		SkRect current_selected_image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);
		SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0 / 3, 1.0 / 3 });
		canvas->drawImageRect(image_display_surface->makeImageSnapshot(), current_selected_image_rectangle, opt);
		auto end = std::chrono::high_resolution_clock::now();
		std::cout << "took internal: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "\n";
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
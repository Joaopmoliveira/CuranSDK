#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include <iostream>
#include <thread>

std::vector<unsigned char> create_image(size_t width, size_t height){
	std::vector<unsigned char> image_data;
	image_data.resize(width*height);
	for(size_t row = 0; row < height; ++row)
		for(size_t col = 0; col < width; ++col)
			image_data[row*width+col] = (int)255.0*((row*row+col*col)/(double)(width*width+height*height));
	return image_data;
};


uint_least8_t dicom_compliant_conversion[256] = {0,29,32,35,37,38,40,41,42,43,44,45,46,47,47,48,49,50,51,51,52,53,54,54,55,56,57,57,58,59,60,60,61,62,62,63,63,64,65,65,66,67,67,68,69,69,70,71,71,72,73,73,74,75,75,76,77,77,78,79,79,80,80,81,82,82,83,84,84,85,86,86,87,88,89,89,90,91,91,92,93,93,94,95,95,96,97,97,98,99,99,100,101,101,102,103,104,104,105,106,107,107,108,109,110,110,111,112,112,113,114,114,115,116,117,117,118,119,120,120,121,122,123,123,124,125,126,127,127,128,129,129,130,131,132,133,133,134,135,136,137,138,138,139,140,141,142,143,143,144,145,146,147,147,148,149,150,151,152,153,154,154,155,156,157,158,159,160,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,175,176,177,178,179,180,181,182,183,184,185,186,188,188,189,191,191,192,193,194,195,196,197,199,199,201,202,202,204,205,206,207,208,209,210,212,212,214,215,216,217,218,220,220,222,223,224,225,226,227,228,230,231,232,233,235,236,237,238,240,241,242,243,244,246,247,248,250,251,253,254,255};

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		auto imagebuff = create_image(500, 500);

		auto image_info = SkImageInfo::Make(500, 500,SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
		size_t pixel_size = 1 ;
	    size_t row_size = 500 * pixel_size;
	    auto wraped_skia_pixmap = SkPixmap{image_info,imagebuff.data(),row_size};
		auto image = SkSurfaces::WrapPixels(wraped_skia_pixmap)->makeImageSnapshot();	

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);

			SkPaint paint;
    		SkScalar colorMatrix1[20] = {
			0, 1, 0, 0, 0,
        	0, 0, 1, 0, 0,
        	1, 0, 0, 0, 0,
        	0, 0, 0, 1, 0};
    		paint.setColorFilter(SkColorFilters::Matrix(colorMatrix1));

    		canvas->drawImage(image, 0, 0, SkSamplingOptions(),&paint);	

    		SkScalar grayscale[20] = {
        	0.21f, 0.72f, 0.07f, 0.0f, 0.0f,
        	0.21f, 0.72f, 0.07f, 0.0f, 0.0f,
        	0.21f, 0.72f, 0.07f, 0.0f, 0.0f,
        	0.0f,  0.0f,  0.0f,  1.0f, 0.0f};
    		paint.setColorFilter(SkColorFilters::Matrix(grayscale));
    		canvas->drawImage(image, 512, 0, SkSamplingOptions(),&paint);	

    		paint.setColorFilter(SkColorFilters::Table(dicom_compliant_conversion));
    		canvas->drawImage(image, 1024, 0, SkSamplingOptions(),&paint);	


			glfwPollEvents();
			auto signals = viewer->process_pending_signals();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (...) {
		std::cout << "Failed";
		return 1;
	}
}
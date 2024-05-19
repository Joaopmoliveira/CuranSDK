#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/ImageWrapper.h"
#include <iostream>
#include <thread>
#include "itkImage.h"
#include "itkImageFileReader.h"


constexpr unsigned int Dimension = 3;
using PixelType = itk::RGBAPixel<unsigned char>;
using ImageType = itk::Image<PixelType, Dimension>;
using ReaderType = itk::ImageFileReader<ImageType>;


void load_image(ImageType::Pointer& pointer_to_block_of_memory){
	auto reader = ReaderType::New();
	
	std::string dirName_input{"C:/Users/SURGROB7/Desktop/Manuel_Carvalho/targets_1.png"};

	reader->SetFileName(dirName_input);

	try
		{
			reader->Update();
		}
		catch (const itk::ExceptionObject &ex)
		{
			std::cout << ex << std::endl;
			//return;
		}
	

	pointer_to_block_of_memory = reader->GetOutput();
}

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		SkColor colbuton = { SK_ColorRED };

 		ImageType::Pointer pointer_to_block_of_memory;

		load_image(pointer_to_block_of_memory);

    	ImageType::SizeType size_itk = pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();
    	auto buff = curan::utilities::CaptureBuffer::make_shared(pointer_to_block_of_memory->GetBufferPointer(), pointer_to_block_of_memory->GetPixelContainer()->Size() * sizeof(PixelType), pointer_to_block_of_memory);
    	curan::ui::ImageWrapper wrapper{buff, size_itk[0], size_itk[1], kRGBA_8888_SkColorType,kUnpremul_SkAlphaType};

		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kFill_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(colbuton);

		bool transl = true;

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);
			SkPoint point{ 400,400 };

			canvas->drawImage(wrapper.image,0,0);

			canvas->drawCircle(point,20.0, paint_square);

			/* if(transl){
				canvas->rotate(20);
				canvas->translate(128, 0);
				transl = false;
			} */


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
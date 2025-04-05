#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Slider.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/SliderPanel.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>

#include <iostream>
#include <thread>

#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkResampleImageFilter.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkImageFileWriter.h"

#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"

using DicomPixelType = unsigned short;
using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        
        using ImageReaderType = itk::ImageFileReader<itk::Image<double,3>>;

        std::printf("\nReading input volume...\n");
        auto fixedImageReader = ImageReaderType::New();
        fixedImageReader->SetFileName(CURAN_COPIED_RESOURCE_PATH"/precious_phantom/precious_phantom.mha");
    
        // Rescale and cast the volume to use with the correct MaskPixelType (0-255)
        auto rescale = itk::RescaleIntensityImageFilter<itk::Image<double,3>, itk::Image<double,3>>::New();
        rescale->SetInput(fixedImageReader->GetOutput());
        rescale->SetOutputMinimum(0);
        rescale->SetOutputMaximum(255.0);
    
        auto castfilter = itk::CastImageFilter<itk::Image<double,3>, ImageType>::New();
        castfilter->SetInput(rescale->GetOutput());
        castfilter->Update();
        
        curan::ui::VolumetricMask vol{castfilter->GetOutput()};

        std::unique_ptr<SlidingPanel> image_display_x = SlidingPanel::make(resources, &vol, Direction::X);
        std::unique_ptr<SlidingPanel> image_display_y = SlidingPanel::make(resources, &vol, Direction::Y);
        std::unique_ptr<SlidingPanel> image_display_z = SlidingPanel::make(resources, &vol, Direction::Z);

		auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
		*container << std::move(image_display_x) << std::move(image_display_y) << std::move(image_display_z);

		auto page = Page{std::move(container),SK_ColorBLACK};
		page.update_page(viewer.get());

		ConfigDraw config_draw{ &page };

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
            for(const auto& sig : signals)
                page.propagate_signal(sig, &config_draw);				

			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (std::exception& e) {
		std::cout << "Failed" << e.what() << std::endl;
		return 1;
	}
}
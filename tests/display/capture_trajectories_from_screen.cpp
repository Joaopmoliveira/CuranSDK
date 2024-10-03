#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Panel.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/Page.h"

#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"

#include "itkImage.h"
#include "itkImageFileReader.h"

using DicomPixelType = unsigned short;
using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension>;

std::optional<curan::ui::ImageWrapper> get_image(){
    using ImageReaderType = itk::ImageFileReader<DICOMImageType>;
    auto ImageReader = ImageReaderType::New();

    std::string dirName{CURAN_COPIED_RESOURCE_PATH"/images/grid.jpg"};
    ImageReader->SetFileName(dirName);

	using RescaleType = itk::RescaleIntensityImageFilter<DICOMImageType, DICOMImageType>;
  	auto rescale = RescaleType::New();
  	rescale->SetInput(ImageReader->GetOutput());
  	rescale->SetOutputMinimum(0);
  	rescale->SetOutputMaximum(itk::NumericTraits<PixelType>::max());

  	using FilterType = itk::CastImageFilter<DICOMImageType,ImageType>;
  	auto filter = FilterType::New();
  	filter->SetInput(rescale->GetOutput());

    try{
          filter->Update();
    }  catch (const itk::ExceptionObject& ex)
    {
        std::cout << ex << std::endl;
        return std::nullopt;
    }
  
    ImageType::Pointer pointer_to_block_of_memory = filter->GetOutput();
    ImageType::SizeType size_itk =  pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();
    auto buff = curan::utilities::CaptureBuffer::make_shared(pointer_to_block_of_memory->GetBufferPointer(),pointer_to_block_of_memory->GetPixelContainer()->Size()*sizeof(PixelType),pointer_to_block_of_memory);
    return curan::ui::ImageWrapper{buff,size_itk[0],size_itk[1]};
}

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1200 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		std::unique_ptr<Panel> image_display = Panel::make(resources,get_image());

		auto button = Button::make("Connect",resources);
		button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));

		auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
		*container << std::move(button) << std::move(image_display);
		container->set_divisions({ 0.0 , 0.1 , 1.0 });

    	curan::ui::Page page{std::move(container),SK_ColorBLACK};

		ConfigDraw config{&page};

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
            	page.propagate_signal(signals.back(),&config);
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

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "itkImage.h"
#include "itkImageFileReader.h"

using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

void function(curan::ui::ImageDisplay* image_display){
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    using ImageReaderType = itk::ImageFileReader<ImageType>;
    auto ImageReader = ImageReaderType::New();

    std::string dirName{CURAN_COPIED_RESOURCE_PATH"/dicom_sample/mri_brain/233.dcm"};
    ImageReader->SetFileName(dirName);
    try{
          ImageReader->Update();

    }     
    catch (const itk::ExceptionObject& ex)
    {
        std::cout << ex << std::endl;
        return ;
    }
  
    ImageType::Pointer pointer_to_block_of_memory = ImageReader->GetOutput();
    ImageType::SizeType size_itk =  pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();
    auto buff = curan::utilities::CaptureBuffer::make_shared(pointer_to_block_of_memory->GetBufferPointer(),pointer_to_block_of_memory->GetPixelContainer()->Size()*sizeof(PixelType),pointer_to_block_of_memory);
    curan::ui::ImageWrapper wrapper{buff,size_itk[0],size_itk[1]};
    image_display->update_image(wrapper);
}

int main() {
try {
    using namespace curan::ui;
    std::unique_ptr<Context> context = std::make_unique<Context>();;
    DisplayParams param{ std::move(context),600,600 };
    std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

    std::unique_ptr<ImageDisplay> image_display = ImageDisplay::make();
    //image_display->set_size(SkRect::MakeWH(500,500));
    ImageDisplay* pointer_to = image_display.get();
    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
    *container << std::move(image_display);
    curan::ui::Page page{std::move(container),SK_ColorBLACK};

    auto call = [pointer_to](){
        function(pointer_to);
    };

    std::thread image_generator(call);

    page.update_page(viewer.get());

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
    image_generator.join();
    return 0;   
    }
catch (std::exception & e ) {
    std::cout << "Failed: " << e.what() << std::endl;
    return 1;
}
}
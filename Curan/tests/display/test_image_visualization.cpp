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

    std::string dirName{CURAN_COPIED_RESOURCE_PATH"/itk_data_manel/BrainProtonDensitySlice.png"};
    ImageReader->SetFileName(dirName);
    ImageReader->Update();

    
    ImageType::Pointer pointer_to_block_of_memory = ImageReader->GetOutput();
    auto lam = [pointer_to_block_of_memory](SkPixmap& requested) {
        ImageType::RegionType region = pointer_to_block_of_memory->GetLargestPossibleRegion();
        ImageType::SizeType size_itk = region.GetSize();
	    auto inf = SkImageInfo::Make(size_itk.GetSize()[0], size_itk.GetSize()[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
	    size_t row_size = size_itk.GetSize()[0] * sizeof(char);
	    SkPixmap map{inf,pointer_to_block_of_memory->GetBufferPointer(),row_size};
	    requested = map;
	    return;
    };
    image_display->update_image(lam);
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

    auto rec = viewer->get_size();
    page.propagate_size_change(rec);
    int width = rec.width();
    int height = rec.height();

    ConfigDraw config{&page};

    while (!glfwWindowShouldClose(viewer->window)) {
        auto start = std::chrono::high_resolution_clock::now();
        SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
        auto temp_height = pointer_to_surface->height();
        auto temp_width = pointer_to_surface->width();
        SkCanvas* canvas = pointer_to_surface->getCanvas();
        if (temp_height != height || temp_width != width) {
            rec = SkRect::MakeWH(temp_width, temp_height);
            page.propagate_size_change(rec);
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
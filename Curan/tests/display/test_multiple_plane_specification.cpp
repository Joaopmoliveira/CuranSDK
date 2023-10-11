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

void callback_x_dir(curan::ui::ImageDisplay* image_display,ImageType::Pointer image_data, const MemoryOfDisplay& memory){
    assert(image_display!=nullptr);

    ImageType::RegionType region = pointer_to_block_of_memory->GetLargestPossibleRegion();
    ImageType::SizeType size_itk = region.GetSize();

    auto lam = [pointer_to_block_of_memory](SkPixmap& requested) {
	    auto inf = SkImageInfo::Make(size_itk.GetSize()[0], size_itk.GetSize()[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
	    size_t row_size = size_itk.GetSize()[0] * sizeof(char);
        SkPixmap map{inf,pointer_to_block_of_memory->GetBufferPointer(),row_size};
        requested = map;
	    return;
    };
}

void callback_y_dir(curan::ui::ImageDisplay* image_display,ImageType::Pointer image_data, const MemoryOfDisplay& memory){
    assert(image_display!=nullptr);

    ImageType::RegionType region = pointer_to_block_of_memory->GetLargestPossibleRegion();
    ImageType::SizeType size_itk = region.GetSize();

    auto lam = [pointer_to_block_of_memory](SkPixmap& requested) {
	    auto inf = SkImageInfo::Make(size_itk.GetSize()[0], size_itk.GetSize()[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
	    size_t row_size = size_itk.GetSize()[0] * sizeof(char);
        SkPixmap map{inf,pointer_to_block_of_memory->GetBufferPointer(),row_size};
        requested = map;
	    return;
    };
}

void callback_z_dir(curan::ui::ImageDisplay* image_display,ImageType::Pointer image_data, const MemoryOfDisplay& memory){
    assert(image_display!=nullptr);

    ImageType::RegionType region = pointer_to_block_of_memory->GetLargestPossibleRegion();
    ImageType::SizeType size_itk = region.GetSize();

    auto lam = [pointer_to_block_of_memory](SkPixmap& requested) {
	    auto inf = SkImageInfo::Make(size_itk.GetSize()[0], size_itk.GetSize()[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
	    size_t row_size = size_itk.GetSize()[0] * sizeof(char);
        SkPixmap map{inf,pointer_to_block_of_memory->GetBufferPointer(),row_size};
        requested = map;
	    return;
    };
}

struct MemoryOfDisplay{
    size_t current_index;
    size_t max_index = 1;
    size_t min_index = 0;

    MemoryOfDisplay(size_t max,size_t min) max_index{max},min_index{min},current_index{min}:{}

    void operator++()(){
        current_index = (current_index+1>max_index) ? min_index : current_index+1;
    }
};

int main() {
    using namespace curan::ui;
    using ImageReaderType = itk::ImageFileReader<ImageType>;
    auto ImageReader = ImageReaderType::New();
    std::string dirName{CURAN_COPIED_RESOURCE_PATH "/precious_phantom/precious_phantom.mha"};
    ImageReader->SetFileName(dirName);
    ImageReader->Update();

    std::unique_ptr<Context> context = std::make_unique<Context>();;
    DisplayParams param{ std::move(context),600,600 };
    std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

    MemoryOfDisplay memory_x{};
    std::unique_ptr<ImageDisplay> image_display_x = ImageDisplay::make();
    ImageDisplay* pointer_to_x = image_display_x.get();

    MemoryOfDisplay memory_y{};
    std::unique_ptr<ImageDisplay> image_display_y = ImageDisplay::make();
    ImageDisplay* pointer_to_y = image_display_y.get();

    MemoryOfDisplay memory_z{};
    std::unique_ptr<ImageDisplay> image_display_z = ImageDisplay::make();
    ImageDisplay* pointer_to_z = image_display_z.get();

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
    *container << std::move(image_display_x) << std::move(image_display_y) << std::move(image_display_z) ;
    curan::ui::Page page{std::move(container),SK_ColorBLACK};

    page.update_page(viewer.get());

    ConfigDraw config{&page};

    while (!glfwWindowShouldClose(viewer->window)) {
        auto start = std::chrono::high_resolution_clock::now();
        SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
        auto temp_height = pointer_to_surface->height();
        auto temp_width = pointer_to_surface->width();
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
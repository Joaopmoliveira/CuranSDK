#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "itkImage.h"
#include "itkImageFileReader.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
    return static_cast<typename std::underlying_type<E>::type>(e);
}

enum class TravelerDirection{
    X = 0,
    Y,
    Z
};

class BiDimensionalTraveler{
    ImageType::Pointer displayed_image;
    std::array<size_t,3> size_index;
    std::array<size_t,3> current_index;
    std::array<Eigen::Rotation2Dd,3> rotation_of_each_direction;
    std::array<Eigen::Matrix<double,2,1>,3> translation_of_each_direction;

    template<TravelerDirection direction>
    void increment(){
        current_index[to_underlying(direction)] = current_index[to_underlying(direction)]+1 % size_index[to_underlying(direction)];
    };

    template<TravelerDirection direction>
    void decrement(){
        current_index[to_underlying(direction)] = current_index[to_underlying(direction)]-1 % size_index[to_underlying(direction)];
    }; 

    template<TravelerDirection direction>
    void rotate(double angle){
        rotation_of_each_direction[to_underlying(direction)] *= Eigen::Rotation2Dd{angle};
    };

    template<TravelerDirection direction>
    void translate(double x,double y){
            Eigen::Matrix<double,2,1> local;
            local[0] = x; local[1] = y;
            translation_of_each_direction[to_underlying(direction)] += local;
    };

    template<TravelerDirection direction>
    Eigen::Matrix<double,3,1> get_position(double x, double y){
        // first we need to forward pass the current transformation of the image on screen into pixel coordiantes

        // once we gave the pixel coordinates we need to query the homogeneous transform of the 
    }

};

void callback_x_dir(curan::ui::ImageDisplay* image_display,ImageType::Pointer image_data, BiDimensionalTraveler& memory){
    assert(image_display!=nullptr);

    ImageType::RegionType region = image_data->GetLargestPossibleRegion();
    ImageType::SizeType size_itk = region.GetSize();

    auto lam = [image_data,size_itk](SkPixmap& requested) {
	    auto inf = SkImageInfo::Make(size_itk.GetSize()[0], size_itk.GetSize()[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
	    size_t row_size = size_itk.GetSize()[0] * sizeof(char);
        SkPixmap map{inf,image_data->GetBufferPointer(),row_size};
        requested = map;
	    return;
    };
}

void callback_y_dir(curan::ui::ImageDisplay* image_display,ImageType::Pointer image_data, BiDimensionalTraveler& memory){
    assert(image_display!=nullptr);

    ImageType::RegionType region = image_data->GetLargestPossibleRegion();
    ImageType::SizeType size_itk = region.GetSize();

    auto lam = [image_data,size_itk](SkPixmap& requested) {
	    auto inf = SkImageInfo::Make(size_itk.GetSize()[0], size_itk.GetSize()[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
	    size_t row_size = size_itk.GetSize()[0] * sizeof(char);
        SkPixmap map{inf,image_data->GetBufferPointer(),row_size};
        requested = map;
	    return;
    };
}

void callback_z_dir(curan::ui::ImageDisplay* image_display,ImageType::Pointer image_data, BiDimensionalTraveler& memory){
    assert(image_display!=nullptr);

    ImageType::RegionType region = image_data->GetLargestPossibleRegion();
    ImageType::SizeType size_itk = region.GetSize();

    auto lam = [image_data,size_itk](SkPixmap& requested) {
	    auto inf = SkImageInfo::Make(size_itk.GetSize()[0], size_itk.GetSize()[1], SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
	    size_t row_size = size_itk.GetSize()[0] * sizeof(char);
        SkPixmap map{inf,image_data->GetBufferPointer(),row_size};
        requested = map;
	    return;
    };
}


curan::ui::Page create_page_with_widgets(){
    std::unique_ptr<curan::ui::ImageDisplay> image_display_x = curan::ui::ImageDisplay::make();

    std::unique_ptr<curan::ui::ImageDisplay> image_display_y = curan::ui::ImageDisplay::make();

    std::unique_ptr<curan::ui::ImageDisplay> image_display_z = curan::ui::ImageDisplay::make();

    auto containerbuttons = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::HORIZONTAL);

    auto container_image_display = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::HORIZONTAL);
    *container_image_display << std::move(image_display_x) << std::move(image_display_y) << std::move(image_display_z) ;

    auto container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::VERTICAL);
    *container << std::move(containerbuttons) << std::move(container_image_display);

    curan::ui::Page page{std::move(container),SK_ColorBLACK};
    return std::move(page);
}


int main() {
    BiDimensionalTraveler traveler;

    using namespace curan::ui;
    using ImageReaderType = itk::ImageFileReader<ImageType>;
    auto ImageReader = ImageReaderType::New();
    std::string dirName{CURAN_COPIED_RESOURCE_PATH "/precious_phantom/precious_phantom.mha"};
    ImageReader->SetFileName(dirName);
    ImageReader->Update();

    std::unique_ptr<curan::ui::Context> context = std::make_unique<curan::ui::Context>();;
    curan::ui::DisplayParams param{ std::move(context), 600,600 };
    std::unique_ptr<curan::ui::Window> viewer = std::make_unique<curan::ui::Window>(std::move(param));

    auto page = create_page_with_widgets();
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
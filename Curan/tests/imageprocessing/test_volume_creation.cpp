#include "imageprocessing/VolumeReconstructor.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"

constexpr long width = 50;
constexpr long height = 50;
constexpr double offset = 1;
float spacing[3] = {0.05 , 0.05 , 1};
float final_spacing [3] = {1 ,1, 1};
using imageType = curan::image::VolumeReconstructor::output_type;

void updateBaseTexture3D(vsg::floatArray3D& image, imageType::Pointer image_to_render)
{
    using OutputPixelType = float;
    using InputImageType = itk::Image<unsigned char, 3>;
    using OutputImageType = itk::Image<OutputPixelType, 3>;
    using FilterType = itk::CastImageFilter<InputImageType, OutputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<OutputImageType, OutputImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(filter->GetOutput());
    rescale->SetOutputMinimum(0.0);
    rescale->SetOutputMaximum(1.0);

    try{
        rescale->Update();
    } catch (const itk::ExceptionObject& e) {
        std::cerr << "Error: " << e << std::endl;
        throw std::runtime_error("error");
    }

    OutputImageType::Pointer out = rescale->GetOutput();

    using IteratorType = itk::ImageRegionIteratorWithIndex<OutputImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
        imageType::IndexType idx = outputIt.GetIndex();
        //std::cout << "value: " << outputIt.Get() << std::endl;
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
        //std::printf("%f\n",outputIt.Get());
    }
}

void create_array_of_linear_images_in_x_direction(std::vector<imageType::Pointer>& desired_images){
    // the volume shoud be a box with spacing 1.0 mm in all directions with a size of 2 mm in each side 
    // to fill the volume we create an image with two images 

    itk::Matrix<double> image_orientation;
	itk::Point<double> image_origin;

	image_orientation[0][0] = 1.0;
	image_orientation[1][0] = 0.0;
	image_orientation[2][0] = 0.0;

	image_orientation[0][1] = 0.0;
	image_orientation[1][1] = 1.0;
	image_orientation[2][1] = 0.0;

	image_orientation[0][2] = 0.0;
	image_orientation[1][2] = 0.0;
	image_orientation[2][2] = 1.0;

    std::cout << "orientation: \n" << image_orientation << std::endl;

	image_origin[0] = 0.0;
	image_origin[1] = 0.0;
	image_origin[2] = 0.0;

    curan::image::char_pixel_type pixel[width*height];
    for(size_t y = 0; y < height ; ++y)
        for(size_t x = 0; x < width ; ++x)
            pixel[x+y*height] = (int) std::sqrt(y*y+x*x);

    for(int z = 0; z < width ; ++z){
        imageType::Pointer image = imageType::New();
        imageType::IndexType start;
        start[0] = 0; // first index on X
        start[1] = 0; // first index on Y
        start[2] = 0; // first index on Z

        imageType::SizeType size;
        size[0] = width; // size along X
        size[1] = height; // size along Y
        size[2] = 1; // size along Z

        imageType::RegionType region_1;
        region_1.SetSize(size);
        region_1.SetIndex(start);

        image->SetRegions(region_1);
        image->SetDirection(image_orientation);
        image->SetOrigin(image_origin);
        image->SetSpacing(spacing);
        image->Allocate();

        image_origin[0] = 0.0;
	    image_origin[1] = 0.0;
	    image_origin[2] = image_origin[2] + offset;

        auto pointer = image->GetBufferPointer();
        std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);
        desired_images.push_back(image);
    }
}

void create_array_of_linear_images_in_y_direction(std::vector<imageType::Pointer>& desired_images){


}

void create_array_of_linear_images_in_z_direction(std::vector<imageType::Pointer>& desired_images){


}

void create_array_of_rotational_images_around_the_x_axis(std::vector<imageType::Pointer>& desired_images){


}

void create_array_of_rotational_images_around_the_y_axis(std::vector<imageType::Pointer>& desired_images){


}

void create_array_of_rotational_images_around_the_z_axis(std::vector<imageType::Pointer>& desired_images){


}
//void volume_creation(curan::renderable::Window& window){
void volume_creation(curan::renderable::Window& window){
try{
    std::vector<imageType::Pointer> images;
    create_array_of_linear_images_in_x_direction(images);

	curan::image::VolumeReconstructor reconstructor;

    std::array<double,2> clip_origin = {0.0,0.0};
    std::array<double,2> clip_size = {width,height};

	reconstructor.set_output_spacing(spacing);
	reconstructor.set_fill_strategy(curan::image::VolumeReconstructor::FillingStrategy::GAUSSIAN);
	reconstructor.set_clipping_bounds(clip_origin, clip_size);
	reconstructor.add_frames(images);

	std::cout << "Started volumetric reconstruction: \n";
	reconstructor.update();
	std::cout << "Finished volumetric reconstruction: \n";

	std::cout << "Started volumetric filling: \n";
	curan::image::VolumeReconstructor::KernelDescriptor descript;
	descript.fillType = curan::image::VolumeReconstructor::FillingStrategy::DISTANCE_WEIGHT_INVERSE;
    descript.size = 5;
	descript.stdev = 1;
	descript.minRatio = 0.1;
	reconstructor.add_kernel_descritor(descript);
	reconstructor.fill_holes();
	std::cout << "Finished volumetric filling: \n";

	curan::image::VolumeReconstructor::output_type::Pointer buffer;
	reconstructor.get_output_pointer(buffer);

    auto size = buffer->GetLargestPossibleRegion().GetSize();
    std::printf("size : ");
    std::cout << size << std::endl;
    
    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = size.GetSize()[0]; 
    volumeinfo.height = size.GetSize()[1];
    volumeinfo.depth = size.GetSize()[2];
    volumeinfo.spacing_x = final_spacing[0];
    volumeinfo.spacing_y = final_spacing[1];
    volumeinfo.spacing_z = final_spacing[2];
    
    auto volume = curan::renderable::Volume::make(volumeinfo);
    window << volume;

    auto casted_volume = volume->cast<curan::renderable::Volume>();
    auto updater = [buffer](vsg::floatArray3D& image){
        updateBaseTexture3D(image, buffer);
    };
    casted_volume->update_volume(updater);
}
catch(std::exception & e){
    std::cout << "failed : " << e.what() << std::endl;
}
}

int main(){
try{
    //volume_creation();    
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{1000, 800};
    info.window_size = size;
    curan::renderable::Window window{info};

    std::thread volume_reconstruction{[&](){ volume_creation(window); }};
    window.run();
    volume_reconstruction.join();

    window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable >>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }
    });
} catch(std::exception & e){
    std::cout << "failed: " << e.what() << std::endl;
}
return 0;
}
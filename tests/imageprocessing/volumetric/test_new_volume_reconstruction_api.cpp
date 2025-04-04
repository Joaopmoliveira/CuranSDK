#include "imageprocessing/StaticReconstructor.h"
#include <optional>
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"

constexpr long width = 426;
constexpr long height = 386;
float spacing[3] = {0.0001852 , 0.0001852 , 0.0001852};
float final_spacing [3] = {0.001852 ,0.001852, 0.001852};

void create_array_of_linear_images_in_x_direction(std::vector<curan::image::StaticReconstructor::output_type::Pointer>& desired_images){
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

	image_origin[0] = 0.0;
	image_origin[1] = 0.0;
	image_origin[2] = 0.0;

    curan::image::char_pixel_type pixel[width*height];
    for(size_t y = 0; y < height ; ++y){
        for(size_t x = 0; x < width ; ++x){
			auto val = 255.0*(std::sqrt(y*y+x*x)/std::sqrt((height-1)*(height-1)+(width-1)*(width-1)));
			pixel[x+y*height] = (int) val;
		}
	}

    for(int z = 0; z < width ; ++z){
        curan::image::StaticReconstructor::output_type::Pointer image = curan::image::StaticReconstructor::output_type::New();
        curan::image::StaticReconstructor::output_type::IndexType start;
        start[0] = 0; // first index on X
        start[1] = 0; // first index on Y
        start[2] = 0; // first index on Z

        curan::image::StaticReconstructor::output_type::SizeType size;
        size[0] = width; // size along X
        size[1] = height; // size along Y
        size[2] = 1; // size along Z

        curan::image::StaticReconstructor::output_type::RegionType region_1;
        region_1.SetSize(size);
        region_1.SetIndex(start);

        image->SetRegions(region_1);
        image->SetDirection(image_orientation);
        image->SetOrigin(image_origin);
        image->SetSpacing(spacing);
        image->Allocate();

        image_origin[0] = 0.0;
	    image_origin[1] = 0.0;
	    image_origin[2] = std::sin((1.0/(width-1))*z);

        auto pointer = image->GetBufferPointer();
        std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);
        desired_images.push_back(image);
    }
}

void updateBaseTexture3D(vsg::vec4Array2D& image, curan::image::StaticReconstructor::output_type::Pointer image_to_render)
{
    using OutputPixelType = float;
    using OutputImageType = itk::Image<OutputPixelType, 3>;
    using FilterType = itk::CastImageFilter<curan::image::StaticReconstructor::output_type, OutputImageType>;
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
        OutputImageType::IndexType idx = outputIt.GetIndex();
        image.set(idx[0],idx[1],vsg::vec4(outputIt.Get(),outputIt.Get(),outputIt.Get(),1.0));
    }
}

void updateBaseTexture3D(vsg::floatArray3D& image, curan::image::StaticReconstructor::output_type::Pointer image_to_render)
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
        curan::image::StaticReconstructor::output_type::IndexType idx = outputIt.GetIndex();
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
}

void update_volume(curan::renderable::Window& window,std::atomic<bool>& continue_updating){
	std::vector<curan::image::StaticReconstructor::output_type::Pointer> image_array;
	create_array_of_linear_images_in_x_direction(image_array);

	std::array<double,3> vol_origin = {0.0,0.0,0.0};
	std::array<double,3> vol_spacing = {final_spacing[0],final_spacing[1],final_spacing[2]};
	std::array<double,3> vol_size = {0.0606,0.0731,0.1349};
	std::array<std::array<double,3>,3> vol_direction;
	vol_direction[0] = {1.0,0.0,0.0};
	vol_direction[1] = {0.0,1.0,0.0};
	vol_direction[2] = {0.0,0.0,1.0};
	curan::image::StaticReconstructor::Info recon_info{vol_spacing,vol_origin,vol_size,vol_direction};
	curan::image::StaticReconstructor reconstructor{recon_info};
	reconstructor.set_compound(curan::image::reconstruction::Compounding::LATEST_COMPOUNDING_MODE)
        .set_interpolation(curan::image::reconstruction::Interpolation::NEAREST_NEIGHBOR_INTERPOLATION);

	auto buffer = reconstructor.get_output_pointer();

	auto sizeimage = buffer->GetLargestPossibleRegion().GetSize();
    std::printf("size : ");
    std::cout << sizeimage << std::endl;
    
    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = sizeimage.GetSize()[0]; 
    volumeinfo.height = sizeimage.GetSize()[1];
    volumeinfo.depth = sizeimage.GetSize()[2];
    volumeinfo.spacing_x = final_spacing[0]*1e3;
    volumeinfo.spacing_y = final_spacing[1]*1e3;
    volumeinfo.spacing_z = final_spacing[2]*1e3;
    auto volume = curan::renderable::Volume::make(volumeinfo);
	auto casted_volume = volume->cast<curan::renderable::Volume>();
    auto updater = [buffer](vsg::floatArray3D& image){
        updateBaseTexture3D(image, buffer);
    };
    casted_volume->update_volume(updater);
    window << volume;

    curan::renderable::DynamicTexture::Info infotexture;
    infotexture.height = height;
    infotexture.width = width;
    infotexture.builder = vsg::Builder::create();
    infotexture.spacing = {final_spacing[0],final_spacing[1],final_spacing[2]};
    infotexture.origin = {0.0,0.0,0.0};
    auto texture = curan::renderable::DynamicTexture::make(infotexture);
    window << texture;

	for(auto img : image_array){
        texture->cast<curan::renderable::DynamicTexture>()->update_texture([img](vsg::vec4Array2D& image){ updateBaseTexture3D(image,img);});
        auto localorigin = img->GetOrigin();
        texture->update_transform(vsg::translate(localorigin[0],localorigin[1],localorigin[2]));
		reconstructor.add_frame(img);
		reconstructor.update();
		if(!continue_updating)
			return;
		buffer = reconstructor.get_output_pointer();
		auto updater = [buffer](vsg::floatArray3D& image){ updateBaseTexture3D(image, buffer); };
    	casted_volume->update_volume(updater);
	}
}

int main(){
	curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{1200, 1000};
    info.window_size = size;
    curan::renderable::Window window{info};
	std::atomic<bool> continue_updating = true;
	auto callable = [&continue_updating,&window](){
		update_volume(window,continue_updating);
	};
	std::thread volume_updater{callable};
    window.run();
	continue_updating = false;
	volume_updater.join();
    window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable >>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }
    });
	return 0;
};
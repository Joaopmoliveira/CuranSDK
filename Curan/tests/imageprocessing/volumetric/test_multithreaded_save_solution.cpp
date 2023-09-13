#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include <optional>
#include "itkCastImageFilter.h"
#include "itkImageFileWriter.h"
#include "itkImage.h"

constexpr long width = 500;
constexpr long height = 500;
float spacing[3] = {0.002 , 0.002 , 0.002};
float final_spacing [3] = {0.002,0.002, 0.002};

using imagetype = itk::Image<unsigned char,3>;

void create_array_of_linear_images_in_x_direction(std::vector<imagetype::Pointer>& desired_images){
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
        imagetype::Pointer image = imagetype::New();
        imagetype::IndexType start;
        start[0] = 0; // first index on X
        start[1] = 0; // first index on Y
        start[2] = 0; // first index on Z

        imagetype::SizeType size;
        size[0] = width; // size along X
        size[1] = height; // size along Y
        size[2] = 1; // size along Z

        imagetype::RegionType region_1;
        region_1.SetSize(size);
        region_1.SetIndex(start);

        image->SetRegions(region_1);
        image->SetDirection(image_orientation);
        image->SetOrigin(image_origin);
        image->SetSpacing(spacing);

        


        image->Allocate();

        image_origin[0] = 0.0;
	    image_origin[1] = 0.0;
	    image_origin[2] = 1.0/(width-1)*z;

        auto pointer = image->GetBufferPointer();
        std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);
        desired_images.push_back(image);
    }
}

void volume_creation(curan::renderable::Window& window,std::atomic<bool>& stopping_condition){
    try{
	    std::vector<imagetype::Pointer> image_array;
	    create_array_of_linear_images_in_x_direction(image_array);

	    std::array<double,3> vol_origin = {0.0,0.0,0.0};
	    std::array<double,3> vol_spacing = {final_spacing[0],final_spacing[1],final_spacing[2]};
	    std::array<double,3> vol_size = {1.0,1.0,1.0};
	    std::array<std::array<double,3>,3> vol_direction;
	    vol_direction[0] = {1.0,0.0,0.0};
	    vol_direction[1] = {0.0,1.0,0.0};
	    vol_direction[2] = {0.0,0.0,1.0};
	    curan::image::IntegratedReconstructor::Info recon_info{vol_spacing,vol_origin,vol_size,vol_direction};

        /* int clip_origin_x = (int)std::floor(0*width/2.0);
        int clip_origin_y = (int)std::floor(0*height/2.0); */

        int clip_origin_x = 5;
        int clip_origin_y = 5;

        curan::image::Clipping desired_clip;
        desired_clip.clipRectangleOrigin[0] = 0;
        desired_clip.clipRectangleOrigin[1] = 0;
        desired_clip.clipRectangleSize[0] = 498;
        desired_clip.clipRectangleSize[1] = 499;

/*         desired_clip.clipRectangleOrigin[0] = 0;
        desired_clip.clipRectangleOrigin[1] = 0;
        desired_clip.clipRectangleSize[0] = width;
        desired_clip.clipRectangleSize[1] = height-1; */


        auto integrated_volume =  curan::image::IntegratedReconstructor::make(recon_info);

        integrated_volume->cast<curan::image::IntegratedReconstructor>()->set_clipping(desired_clip);

        integrated_volume->cast<curan::image::IntegratedReconstructor>()->set_compound(curan::image::reconstruction::Compounding::LATEST_COMPOUNDING_MODE)
            .set_interpolation(curan::image::reconstruction::Interpolation::NEAREST_NEIGHBOR_INTERPOLATION);
        window << integrated_volume;
        
        auto reconstruction_thread_pool = curan::utilities::ThreadPool::create(10);
        std::printf("started volumetric reconstruction\n");
        size_t counter = 0;
	    for(auto img : image_array){
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		    integrated_volume->cast<curan::image::IntegratedReconstructor>()->add_frame(img);
            if(stopping_condition)
                return;
		    integrated_volume->cast<curan::image::IntegratedReconstructor>()->multithreaded_update(reconstruction_thread_pool);
            std::chrono::steady_clock::time_point elapsed_for_reconstruction = std::chrono::steady_clock::now();
            auto val_elapsed_for_reconstruction = (int)std::chrono::duration_cast<std::chrono::microseconds>(elapsed_for_reconstruction - begin).count();
            //std::printf("added image (volume reconstruction %d)\n",val_elapsed_for_reconstruction);
            ++counter;
            if(stopping_condition)
                return;

	    }
        integrated_volume->cast<curan::image::IntegratedReconstructor>()->textureData->data();
        itk::Image<unsigned char, 3>::Pointer my_itk_image = itk::Image<unsigned char, 3>::New();
        itk::Image<unsigned char, 3>::RegionType output_region;
        itk::Image<unsigned char, 3>::IndexType output_start;
	    output_start[0] = 0;
        output_start[1] = 0;
        output_start[2] = 0;
        output_region.SetIndex(output_start);
        my_itk_image->SetRegions(output_region);

        itk::Image<unsigned char, 3>::PointType output_origin;
        output_origin[0] = vol_origin[0];
        output_origin[1] = vol_origin[1];;	
        output_origin[2] = vol_origin[2];;
        my_itk_image->SetOrigin(output_origin);
        itk::Image<unsigned char, 3>::SizeType output_size;
        output_size[0] = integrated_volume->cast<curan::image::IntegratedReconstructor>()->textureData->width();
        output_size[1] = integrated_volume->cast<curan::image::IntegratedReconstructor>()->textureData->height();;	
        output_size[2] = integrated_volume->cast<curan::image::IntegratedReconstructor>()->textureData->depth();;
        output_region.SetSize(output_size);
        
        itk::Image<unsigned char, 3>::SpacingType output_spacing;
        output_spacing[0] = vol_spacing[0];
        output_spacing[1] = vol_spacing[1];
        output_spacing[2] = vol_spacing[2];
	    my_itk_image->SetSpacing(output_spacing);

        itk::Image<unsigned char, 3>::DirectionType output_directorion;
        output_directorion[0][0] = 1.0;
        output_directorion[1][0] = 0.0;
        output_directorion[2][0] = 0.0;

        output_directorion[0][1] = 0.0;
        output_directorion[1][1] = 1.0;
        output_directorion[2][1] = 0.0;

        output_directorion[0][2] = 0.0;
        output_directorion[1][2] = 0.0;
        output_directorion[2][2] = 1.0;
    	my_itk_image->SetDirection(output_directorion);
	    my_itk_image->Allocate(true);

        

        for(size_t z = 0; z < integrated_volume->cast<curan::image::IntegratedReconstructor>()->textureData->depth(); ++z)
            for(size_t y = 0; y < integrated_volume->cast<curan::image::IntegratedReconstructor>()->textureData->height(); ++y)
                for(size_t x = 0; x < integrated_volume->cast<curan::image::IntegratedReconstructor>()->textureData->width(); ++x){
                    //my_itk_image(row, col) = integrated_volume->cast<curan::image::IntegratedReconstructor>()->textureData->data()(row, col);
                }
        
        return ;
    }catch(std::exception& e){
        std::cout << "exception was throuwn with error message :" << e.what() << std::endl;
        return ;
    }
};


int main(){
try{   
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

    std::atomic<bool> stopping_condition = false;
    std::thread volume_reconstruction{[&](){ volume_creation(window,stopping_condition); }};
    window.run();
    stopping_condition = true;
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
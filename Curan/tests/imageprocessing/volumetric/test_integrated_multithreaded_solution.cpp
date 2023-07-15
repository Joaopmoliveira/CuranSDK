#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include <optional>

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
        auto integrated_volume =  curan::image::IntegratedReconstructor::make(recon_info);
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
            std::printf("added image (volume reconstruction %d)\n",val_elapsed_for_reconstruction);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            ++counter;
            if(stopping_condition)
                return;

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
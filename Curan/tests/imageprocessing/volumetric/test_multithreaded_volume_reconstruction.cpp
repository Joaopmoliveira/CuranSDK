#include "imageprocessing/StaticReconstructor.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include <optional>
#include "imageprocessing/SplicingTools.h"

constexpr long width = 500;
constexpr long height = 500;
float spacing[3] = {0.002 , 0.002 , 0.002};
float final_spacing [3] = {0.002 , 0.002 , 0.002};

void updateBaseTexture3DMultiThreaded(vsg::floatArray3D& image, curan::image::StaticReconstructor::output_type::Pointer image_to_render,std::shared_ptr<curan::utilities::ThreadPool> shared_pool)
{
    auto size =  image_to_render->GetLargestPossibleRegion().GetSize();
    int fullExt[6] = {0,size[0]-1, 0,size[1]-1, 0 ,size[2]-1 };
    if(fullExt[1]<0)
        fullExt[1] = 0;
    if(fullExt[3]<0)
        fullExt[3] = 0;
    if(fullExt[5]<0)
        fullExt[5] = 0;       
    std::vector<std::array<int,6>> splitting;
    splitting.resize(shared_pool->size());
    auto val = curan::image::splice_input_extent(splitting,fullExt);

    std::condition_variable cv;
	std::mutex local_mut;
	std::unique_lock<std::mutex> unique_{local_mut};
	int executed = 0;
	size_t index = 0;
	for(const auto& split : splitting){
		curan::utilities::Job job;
		job.description = "partial volume rendering copy";
		job.function_to_execute = [&](){
			try{
                auto buffer = image_to_render->GetBufferPointer()+split[0]+split[2]*size[0]+split[4]*size[0]*size[1];
                for(size_t zind = split[4]; zind<=split[5] ; ++zind){
                    for(size_t yind = split[2]; yind<=split[3] ; ++yind ){
                        for(size_t xind = split[0]; xind <= split[1]; ++xind){
                            image.set(xind, yind, zind, *buffer/255.0f);
                            ++buffer;
                        }
                    }
                }
			    {    
					std::lock_guard<std::mutex> g{local_mut};
					++executed;
				}
				cv.notify_one();
			} catch(std::exception & e){
				std::cout << "exception was thrown with error message: " << e.what() << std::endl;
			}
		};
		++index;
		shared_pool->submit(std::move(job));
	}
	//this blocks until all threads have processed their corresponding block that they need to process
	cv.wait(unique_,[&](){ return executed==splitting.size();});
}



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
	    image_origin[2] = 1.0/(width-1)*z;

        auto pointer = image->GetBufferPointer();
        std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);
        desired_images.push_back(image);
    }
}

void volume_creation(curan::renderable::Window& window,std::atomic<bool>& stopping_condition){
    try{
	    std::vector<curan::image::StaticReconstructor::output_type::Pointer> image_array;
	    create_array_of_linear_images_in_x_direction(image_array);

	    std::array<double,3> vol_origin = {0.0,0.0,0.0};
	    std::array<double,3> vol_spacing = {final_spacing[0],final_spacing[1],final_spacing[2]};
	    std::array<double,3> vol_size = {1.0,1.0,1.0};
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
        window << volume;

        auto reconstruction_thread_pool = curan::utilities::ThreadPool::create(10);
        std::printf("started volumetric reconstruction\n");
        size_t counter = 0;
	    for(auto img : image_array){
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		    reconstructor.add_frame(img);
		    reconstructor.multithreaded_update(reconstruction_thread_pool);
            std::chrono::steady_clock::time_point elapsed_for_reconstruction = std::chrono::steady_clock::now();
            auto val_elapsed_for_reconstruction = (int)std::chrono::duration_cast<std::chrono::microseconds>(elapsed_for_reconstruction - begin).count();
            begin = std::chrono::steady_clock::now();
            if(stopping_condition)
                break;
            auto casted_volume = volume->cast<curan::renderable::Volume>();
            auto updater = [buffer,reconstruction_thread_pool](vsg::floatArray3D& image){
                updateBaseTexture3DMultiThreaded(image, buffer,reconstruction_thread_pool);
            };
            casted_volume->update_volume(updater);
            std::chrono::steady_clock::time_point elapsed_for_rendering = std::chrono::steady_clock::now();
            auto val_elapsed_for_rendering = (int)std::chrono::duration_cast<std::chrono::microseconds>(elapsed_for_rendering - begin).count();
            std::printf("added image (volume reconstruction %d) (volume rendering %d)\n",val_elapsed_for_reconstruction,val_elapsed_for_rendering);
            ++counter;
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
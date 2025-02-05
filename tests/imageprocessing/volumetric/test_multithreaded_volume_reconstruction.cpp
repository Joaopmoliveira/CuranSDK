#include "imageprocessing/StaticReconstructor.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include <optional>
#include "imageprocessing/SplicingTools.h"
#include <nlohmann/json.hpp> // Include the JSON library

constexpr long width = 100;
constexpr long height = 100;
float spacing[3] = {0.01 , 0.01 , 0.01};
float final_spacing [3] = {0.01 , 0.01 , 0.01};


void updateBaseTexture3DMultiThreaded(vsg::floatArray3D& image, curan::image::StaticReconstructor::output_type::Pointer image_to_render,std::shared_ptr<curan::utilities::ThreadPool> shared_pool)
{
    auto size =  image_to_render->GetLargestPossibleRegion().GetSize();
    int fullExt[6] = {0,(int)size[0]-1, 0,(int)size[1]-1, 0 ,(int)size[2]-1 };
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
        auto lamb = [&](){
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
		curan::utilities::Job job{"partial volume rendering copy",lamb};
		++index;
		shared_pool->submit(std::move(job));
	}
	//this blocks until all threads have processed their corresponding block that they need to process
	cv.wait(unique_,[&](){ return executed==splitting.size();});
}



void create_array_of_linear_images_in_x_direction(std::vector<curan::image::StaticReconstructor::output_type::Pointer>& desired_images){
//Sphere
//  itk::Matrix<double> image_orientation;
// 	itk::Point<double> image_origin;

// 	image_orientation[0][0] = 1.0;
// 	image_orientation[1][0] = 0.0;
// 	image_orientation[2][0] = 0.0;

// 	image_orientation[0][1] = 0.0;
// 	image_orientation[1][1] = 1.0;
// 	image_orientation[2][1] = 0.0;

// 	image_orientation[0][2] = 0.0;
// 	image_orientation[1][2] = 0.0;
// 	image_orientation[2][2] = 1.0;

// 	image_origin[0] = 0.0;
// 	image_origin[1] = 0.0;
// 	image_origin[2] = 0.0;
    
//     const double sphere_diameter_max = width;
//     const double z_midpoint = width/2;
    
    
//     curan::image::char_pixel_type pixel[width*height];
//     for(int z = 0; z < width ; ++z){
//         double sphere_diameter = 0.0;
//         if (z < z_midpoint){
//             sphere_diameter = sphere_diameter_max -(z/z_midpoint);
//         } else {
//             sphere_diameter = sphere_diameter_max +(z/z_midpoint);
//         }
//         double sphere_radius = sphere_diameter/2.0;

//     for(size_t y = 0; y < height ; ++y){
//         for(size_t x = 0; x < width ; ++x){
// 			auto val = std::sqrt((y-height/2.0) * (y-height/2.0)+
//                                 (x-width/2.0) * (x-width/2.0)+
//                                 (z-z_midpoint) * (z-z_midpoint));
//             double intensity = 100.0;
//             if (val > sphere_radius){
//                 intensity = 0.0;
//             }else {
            
//                 intensity = 100 - val / sphere_radius;
//             }
// 			pixel[x+y*height] = intensity;
// 		}
// 	}
//         curan::image::StaticReconstructor::output_type::Pointer image = curan::image::StaticReconstructor::output_type::New();
//         curan::image::StaticReconstructor::output_type::IndexType start;
//         start[0] = 0; // first index on X
//         start[1] = 0; // first index on Y
//         start[2] = 0; // first index on Z

//         curan::image::StaticReconstructor::output_type::SizeType size;
//         size[0] = width; // size along X
//         size[1] = height; // size along Y
//         size[2] = 1; // size along Z

//         curan::image::StaticReconstructor::output_type::RegionType region_1;
//         region_1.SetSize(size);
//         region_1.SetIndex(start);

//         image->SetRegions(region_1);
//         image->SetDirection(image_orientation);
//         image->SetOrigin(image_origin);
//         image->SetSpacing(spacing);
//         image->Allocate();

//         image_origin[0] = 0.0;
// 	    image_origin[1] = 0.0;
// 	    image_origin[2] = 1.0/(width-1)*z;

//         auto pointer = image->GetBufferPointer();
//         std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);
//         desired_images.push_back(image);
//     }
// }

//Cone
//     itk::Matrix<double> image_orientation;
//     itk::Point<double> image_origin;

//     image_orientation[0][0] = 1.0;
//     image_orientation[1][0] = 0.0;
//     image_orientation[2][0] = 0.0;

//     image_orientation[0][1] = 0.0;
//     image_orientation[1][1] = 1.0;
//     image_orientation[2][1] = 0.0;

//     image_orientation[0][2] = 0.0;
//     image_orientation[1][2] = 0.0;
//     image_orientation[2][2] = 1.0;

//     image_origin[0] = 0.0;
//     image_origin[1] = 0.0;
//     image_origin[2] = 0.0;

//     constexpr double cone_height = width; // Height of the cone

//     curan::image::char_pixel_type pixel[width * height];

//     for(int z = 0; z < width ; ++z){
//         double cone_radius = static_cast<double>(width-z) * (static_cast<double>(width) / (2.0 * cone_height));
//         for(size_t y = 0; y < height ; ++y){
//             for(size_t x = 0; x < width ; ++x){
//                 double distance_from_center = std::sqrt((y - height / 2.0) * (y - height / 2.0) +
//                                                         (x - width / 2.0) * (x - width / 2.0));

//                 double intensity = 1.0; // Default intensity inside the sphere

//                 if (distance_from_center > cone_radius) {
//                     intensity = 0.0; // Intensity outside the sphere will be 0
//                 } else {
//                     // Intensity within the sphere decreases with distance from the center
//                     intensity = 100.0 - distance_from_center / cone_radius;
//                 }

//                 pixel[x + y * height] = static_cast<curan::image::char_pixel_type>(intensity);
//             }
//         }
//     curan::image::StaticReconstructor::output_type::Pointer image = curan::image::StaticReconstructor::output_type::New();
//         curan::image::StaticReconstructor::output_type::IndexType start;
//         start[0] = 0; // first index on X
//         start[1] = 0; // first index on Y
//         start[2] = 0; // first index on Z

//         curan::image::StaticReconstructor::output_type::SizeType size;
//         size[0] = width; // size along X
//         size[1] = height; // size along Y
//         size[2] = 1; // size along Z

//         curan::image::StaticReconstructor::output_type::RegionType region_1;
//         region_1.SetSize(size);
//         region_1.SetIndex(start);

//         image->SetRegions(region_1);
//         image->SetDirection(image_orientation);
//         image->SetOrigin(image_origin);
//         image->SetSpacing(spacing);
//         image->Allocate();

//         image_origin[0] = 0.0;
// 	    image_origin[1] = 0.0;
// 	    image_origin[2] = 1.0/(width-1)*z;

//         auto pointer = image->GetBufferPointer();
//         std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);
//         desired_images.push_back(image);
//     }
// }

//Cylinder
// itk::Matrix<double> image_orientation;
//     itk::Point<double> image_origin;

//     image_orientation[0][0] = 1.0;
//     image_orientation[1][0] = 0.0;
//     image_orientation[2][0] = 0.0;

//     image_orientation[0][1] = 0.0;
//     image_orientation[1][1] = 1.0;
//     image_orientation[2][1] = 0.0;

//     image_orientation[0][2] = 0.0;
//     image_orientation[1][2] = 0.0;
//     image_orientation[2][2] = 1.0;

//     image_origin[0] = 0.0;
//     image_origin[1] = 0.0;
//     image_origin[2] = 0.0;

//     const double cylinder_diameter = width;  // Set the diameter of the cylinder
//     const double z_midpoint = width / 2;
//     curan::image::char_pixel_type pixel[width*height];
//     for (int z = 0; z < width; ++z) {
//         for (size_t y = 0; y < height; ++y) {
//             for (size_t x = 0; x < width; ++x) {
//                 auto val = std::sqrt((y - height / 2.0) * (y - height / 2.0) +
//                                      (x - width / 2.0) * (x - width / 2.0));
//                 double intensity = 100.0;
//                 if (val > cylinder_diameter / 2.0) {
//                     intensity = 0.0;
//                 } else {
//                     intensity = 100 - val / (cylinder_diameter / 2.0);
//                 }
//                 pixel[x + y * height] = intensity;
//             }
//         }
//         curan::image::StaticReconstructor::output_type::Pointer image = curan::image::StaticReconstructor::output_type::New();
//         curan::image::StaticReconstructor::output_type::IndexType start;
//         start[0] = 0; // first index on X
//         start[1] = 0; // first index on Y
//         start[2] = 0; // first index on Z

//         curan::image::StaticReconstructor::output_type::SizeType size;
//         size[0] = width; // size along X
//         size[1] = height; // size along Y
//         size[2] = 1; // size along Z

//         curan::image::StaticReconstructor::output_type::RegionType region_1;
//         region_1.SetSize(size);
//         region_1.SetIndex(start);

//         image->SetRegions(region_1);
//         image->SetDirection(image_orientation);
//         image->SetOrigin(image_origin);
//         image->SetSpacing(spacing);
//         image->Allocate();

//         image_origin[0] = 0.0;
//         image_origin[1] = 0.0;
//         image_origin[2] = 1.0 / (width - 1) * z;

//         auto pointer = image->GetBufferPointer();
//         std::memcpy(pointer, pixel, sizeof(curan::image::char_pixel_type) * width * height);
//         desired_images.push_back(image);
//     }
// }

// //Double cone
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

    constexpr double sphere_radius_max = width/2.0; // Maximum sphere radius
    constexpr double z_midpoint = width / 2.0; // Midpoint of z-axis

    curan::image::char_pixel_type pixel[width * height];

    for(int z = 0; z < width ; ++z){
        double sphere_radius = 0.0;
        if (z < z_midpoint) {
            // Linearly increase the sphere radius up to the midpoint of z-axis
            sphere_radius = sphere_radius_max * (z / z_midpoint);
        } else {
            // Linearly decrease the sphere radius after the midpoint of z-axis
            sphere_radius = sphere_radius_max * ((width - 1 - z) / (width - 1 - z_midpoint));
        }

        for(size_t y = 0; y < height ; ++y){
            for(size_t x = 0; x < width ; ++x){
                double distance_from_center = std::sqrt((y - height / 2.0) * (y - height / 2.0) +
                                                        (x - width / 2.0) * (x - width / 2.0));

                double intensity = 1.0; // Default intensity inside the sphere

                if (distance_from_center > sphere_radius) {
                    intensity = 0.0; // Intensity outside the sphere will be 0
                } else {
                    // Intensity within the sphere decreases with distance from the center
                    intensity = 1.0 - distance_from_center / sphere_radius;
                }

                pixel[x + y * height] = static_cast<curan::image::char_pixel_type>(intensity * 255.0);
            }
        }
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
        
        auto reconstruction_start_time = std::chrono::steady_clock::now();
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
        std::cout << "Started volumetric filling: \n";
        reconstructor.set_fillstrategy(curan::image::reconstruction::FillingStrategy::GAUSSIAN);
        curan::image::reconstruction::KernelDescriptor descript;
        descript.fillType = curan::image::reconstruction::FillingStrategy::GAUSSIAN;
        descript.size = 10;
	    descript.stdev = 1;
	    descript.minRatio = 0.1;
	    reconstructor.add_kernel_descritor(descript);
        reconstructor.fill_holes();
        
        
        auto reconstruction_end_time = std::chrono::steady_clock::now();

        // Calculate and print the reconstruction time
        auto reconstruction_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        reconstruction_end_time - reconstruction_start_time);

        std::cout << "Reconstruction time: " << reconstruction_duration.count() << " milliseconds" << std::endl;

        buffer = reconstructor.get_output_pointer();
        std::stringstream data_values;
        nlohmann::json volume_file;
        volume_file["width"] = volumeinfo.width;
        volume_file["heigth"] = volumeinfo.height;
        volume_file["depth"] = volumeinfo.depth;
        using IteratorType = itk::ImageRegionIteratorWithIndex<curan::image::StaticReconstructor::output_type>;
        counter = 0;
        IteratorType outputIt(buffer, buffer->GetRequestedRegion());
        for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt,++counter){
            data_values << (int)outputIt.Get() << " ";
            //std::printf("size of string stream : %d\n",counter);
            }
        volume_file["data"] = data_values.str();
        std::ofstream output_file{"test_run_1.json"};
        output_file << volume_file;
        std::printf("Just printed the file\n");
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
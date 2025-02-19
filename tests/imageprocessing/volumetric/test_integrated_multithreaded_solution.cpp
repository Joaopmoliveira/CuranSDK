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

/* Anterior
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
 */

void create_array_of_linear_images_in_x_direction(std::vector<imagetype::Pointer>& desired_images){
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
//             sphere_diameter = sphere_diameter_max * (z/z_midpoint);
//         } else {
//             sphere_diameter = sphere_diameter_max * ((width-1-z)/(width-1-z_midpoint));
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

//Cylinder
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

    const double cylinder_diameter = width;  // Set the diameter of the cylinder
    const double z_midpoint = width / 2;
    curan::image::char_pixel_type pixel[width*height];
    for (int z = 0; z < width; ++z) {
        for (size_t y = 0; y < height; ++y) {
            for (size_t x = 0; x < width; ++x) {
                auto val = std::sqrt((y - height / 2.0) * (y - height / 2.0) +
                                     (x - width / 2.0) * (x - width / 2.0));
                double intensity = 100.0;
                if (val > cylinder_diameter / 2.0) {
                    intensity = 0.0;
                } else {
                    intensity = 100 - val / (cylinder_diameter / 2.0);
                }
                pixel[x + y * height] = intensity;
            }
        }

// //Double cone
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

//     constexpr double sphere_radius_max = width/2.0; // Maximum sphere radius
//     constexpr double z_midpoint = width / 2.0; // Midpoint of z-axis

//     curan::image::char_pixel_type pixel[width * height];

//     for(int z = 0; z < width ; ++z){
//         double sphere_radius = 0.0;
//         if (z < z_midpoint) {
//             // Linearly increase the sphere radius up to the midpoint of z-axis
//             sphere_radius = sphere_radius_max * (z / z_midpoint);
//         } else {
//             // Linearly decrease the sphere radius after the midpoint of z-axis
//             sphere_radius = sphere_radius_max * ((width - 1 - z) / (width - 1 - z_midpoint));
//         }

//         for(size_t y = 0; y < height ; ++y){
//             for(size_t x = 0; x < width ; ++x){
//                 double distance_from_center = std::sqrt((y - height / 2.0) * (y - height / 2.0) +
//                                                         (x - width / 2.0) * (x - width / 2.0));

//                 double intensity = 1.0; // Default intensity inside the sphere

//                 if (distance_from_center > sphere_radius) {
//                     intensity = 0.0; // Intensity outside the sphere will be 0
//                 } else {
//                     // Intensity within the sphere decreases with distance from the center
//                     intensity = 1.0 - distance_from_center / sphere_radius;
//                 }

//                 pixel[x + y * height] = static_cast<curan::image::char_pixel_type>(intensity * 255.0);
//             }
//         }
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
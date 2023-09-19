#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include <itkImage.h>
#include "itkImportImageFilter.h"
#include <optional>
#include <nlohmann/json.hpp> // Include the JSON library
#include <random> // Include the random header for generating random numbers

constexpr long width = 100;
constexpr long height = 100;
float spacing[3] = {0.01 , 0.01 , 0.01}; //0.002
float final_spacing [3] = {0.01, 0.01 , 0.01}; //0.002

// Define a function to generate random numbers within a specified range
double generateRandomNumber(double min, double max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}


using imagetype = itk::Image<unsigned char,3>;

void create_array_of_images(std::vector<imagetype::Pointer>& desired_images){
    itk::Matrix<double> image_orientation;
	itk::Point<double> image_origin;

	image_origin[0] = 0.0;
	image_origin[1] = 0.0;
	image_origin[2] = 0.0;

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
        
	    image_orientation(0,0) = 1.0;
	    image_orientation(1,0) = 0.0;
	    image_orientation(2,0) = 0.0;

	    image_orientation(0,1) = 0.0;
	    image_orientation(1,1) = cos(1.57079632679*z/(width-1));
	    image_orientation(2,1) = sin(1.57079632679*z/(width-1));

	    image_orientation(0,2) = 0.0;
	    image_orientation(1,2) = -sin(1.57079632679*z/(width-1));
	    image_orientation(2,2) = cos(1.57079632679*z/(width-1));

/*         image_orientation(0,1) = 0.0;
	    image_orientation(1,1) = 1;
	    image_orientation(2,1) = 0;

	    image_orientation(0,2) = 0.0;
	    image_orientation(1,2) = 0;
	    image_orientation(2,2) = 1;
 */
        image->SetRegions(region_1);
        image->SetDirection(image_orientation);
        image->SetOrigin(image_origin);
        image->SetSpacing(spacing);
        image->Allocate();

        image_origin[0] = 0.0;
	    image_origin[1] = 0.0;
	    image_origin[2] = 1.0/(width-1)*z;


      using IteratorType = itk::ImageRegionIteratorWithIndex<imagetype>;
        IteratorType outputIt(image, image->GetRequestedRegion());
        // //Sphere
        // for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
        //     imagetype::IndexType idx = outputIt.GetIndex();
        //     imagetype::PointType world_pos;
        //     image->TransformIndexToPhysicalPoint(idx, world_pos);

        //     // Calculate distance from center
        //     imagetype::PointType center;
        //     center[0] = width * final_spacing[0] / 2.0;
        //     center[1] = height * final_spacing[1] / 2.0;
        //     center[2] = width * final_spacing[2] / 2.0;
        //     double distance = std::sqrt(
        //         (world_pos[0] - center[0]) * (world_pos[0] - center[0]) +
        //         (world_pos[1] - center[1]) * (world_pos[1] - center[1]) +
        //         (world_pos[2] - center[2]) * (world_pos[2] - center[2]));

        //     // Use the distance to determine voxel intensity
        //     double max_radius = width * final_spacing[0] / 4.0;
        //     unsigned char val;
        //     if (distance > max_radius){
        //         val = 0.0;
        //     }else {
        //         val = 255.0 * (1.0 - distance / max_radius);
        //         }
        //     outputIt.Set(val);


        // //Cone
        // double cone_height = static_cast<double>(height); 
       
        // for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
        //     imagetype::IndexType idx = outputIt.GetIndex();
        //     double y = static_cast<double>(idx[1]);
        //     double x = static_cast<double>(idx[0]);
            
        
        //     // Calculate distance from the center and adjust intensity based on cone
        //     double distance_from_center = std::sqrt((y - height / 2.0) * (y - height / 2.0) +
        //                                             (z - width / 2.0) * (z - width / 2.0));
        //     double cone_radius = (width - 1-x) * ((width-1) / (4.0 * cone_height));
        //     unsigned char val;
        //     if (distance_from_center <= cone_radius && 20<x) {
        //         // Intensity increases as you move towards the apex of the cone
        //         val = 255.0 * (1.0 - distance_from_center / cone_radius);
        //     } else {
        //         val = 0.0;
        //     }
        //     outputIt.Set(val);

        //Cylinder
        for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
            imagetype::IndexType idx = outputIt.GetIndex();
            imagetype::PointType world_pos;
            image->TransformIndexToPhysicalPoint(idx, world_pos);

        //     // Calculate distance from center
            imagetype::PointType center;
            center[0] = width * final_spacing[0] / 2.0;
            center[1] = height * final_spacing[1] / 2.0;
            center[2] = width * final_spacing[2] / 2.0;
            double distance = std::sqrt(
            (world_pos[1] - center[1]) * (world_pos[1] - center[1]) +
            (world_pos[2] - center[2]) * (world_pos[2] - center[2]));

        //     // Use the distance to determine voxel intensity
            double max_radius = width * final_spacing[0] / 4.0;
            unsigned char val;
            if (distance > max_radius ){
                val = 0.0;
            }else {
        val =  (1.0 - distance / max_radius)*255;
        }
           outputIt.Set(val);

        //Cone
/*         double cone_height = static_cast<double>(height); 
       
        for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
            imagetype::IndexType idx = outputIt.GetIndex();
            double y = static_cast<double>(idx[1]);
            double x = static_cast<double>(idx[0]);
            

            // Calculate distance from the center and adjust intensity based on cone
            double distance_from_center = std::sqrt((y - height / 2.0) * (y - height / 2.0) +
                                                    (z - width / 2.0) * (z - width / 2.0));
            double cone_radius;                                       
            if (x < width/2) {
            // Linearly increase the sphere radius up to the midpoint of z-axis
            cone_radius = x;
            } else {
            // Linearly decrease the sphere radius after the midpoint of z-axis
            cone_radius = (width - x) ;
             }
            
            unsigned char val;
            if (distance_from_center <= cone_radius) {
                // Intensity increases as you move towards the apex of the cone
                val = 255.0 * (1.0 - distance_from_center / cone_radius);
            } else {
                val = 0.0;
            }
            outputIt.Set(val);*/

        }
        
        desired_images.push_back(image);
    }
}

void volume_creation(curan::renderable::Window& window,std::atomic<bool>& stopping_condition){
    try{
	    std::vector<imagetype::Pointer> image_array;
	    create_array_of_images(image_array);

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
            .set_interpolation(curan::image::reconstruction::Interpolation::LINEAR_INTERPOLATION);
        window << integrated_volume;
        auto reconstruction_start_time = std::chrono::steady_clock::now();
        auto reconstruction_thread_pool = curan::utilities::ThreadPool::create(10);
        std::printf("started volumetric reconstruction\n");
        size_t counter = 0;
        while(!stopping_condition){
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

            auto reconstruction_end_time = std::chrono::steady_clock::now();
            std::cout << "Started volumetric filling: \n";
 
            itk::Size<3U>  output_size;
            output_size = integrated_volume->cast<curan::image::IntegratedReconstructor>()->get_output_size();
            size_t width =   output_size[0] ;
            size_t height =   output_size[1] ;
            size_t depth =   output_size[2] ;

            integrated_volume->cast<curan::image::IntegratedReconstructor>()->set_fillstrategy(curan::image::reconstruction::FillingStrategy::NEAREST_NEIGHBOR);
            curan::image::reconstruction::KernelDescriptor descript;
            descript.fillType = curan::image::reconstruction::FillingStrategy::NEAREST_NEIGHBOR;
            descript.size = 7;
	        descript.stdev = 10;
	        descript.minRatio = 0.01;
	        integrated_volume->cast<curan::image::IntegratedReconstructor>()->add_kernel_descritor(descript);
            //std::memcpy(my_beatiful_pointer,image->GetBufferPointer(),numberOfPixels*sizeof(float));
            integrated_volume->cast<curan::image::IntegratedReconstructor>()->fill_holes();
   

            // Calculate and print the reconstruction time
            auto reconstruction_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            reconstruction_end_time - reconstruction_start_time);

            std::cout << "Reconstruction time: " << reconstruction_duration.count() << " milliseconds" << std::endl;

            std::stringstream texture_data_stream;
            nlohmann::json volume_file;
            volume_file["width"] = width;
            volume_file["heigth"] = height;
            volume_file["depth"] = depth;
            for (size_t z = 0; z < depth; ++z) {
                for (size_t y = 0; y < height; ++y) {
                    for (size_t x = 0; x < width; ++x) {
                        float pixel_value = integrated_volume->cast<curan::image::IntegratedReconstructor>()->get_texture_data()->at(x, y, z);
                        texture_data_stream << pixel_value << " ";
                    }
                }
            }
            // Add the texture data array to the JSON object
            volume_file["data"] = texture_data_stream.str();
            std::ofstream output_file{"test_run_5.json"};
            output_file << volume_file; 
            std::printf("Just printed the file\n");

           
            using PixelType = float;
            using ImageType = itk::Image<PixelType, 3>;
            ImageType::Pointer itkVolume = ImageType::New();

            // Set the size, spacing, and origin of the ITK Image
            ImageType::SizeType size;
            size[0] = width;
            size[1] = height;
            size[2] = depth;

            itkVolume->SetRegions(size);
            itkVolume->SetSpacing(vol_spacing.data());
            itkVolume->SetOrigin(vol_origin.data());

            itkVolume->Allocate();

            // Copy the volumetric data from integrated_volume to itkVolume
            for (long long z = 0; z < depth; ++z) {
                for (long long y = 0; y < height; ++y) {
                    for (long long x = 0; x < width; ++x) {
                        float pixel_value = integrated_volume->cast<curan::image::IntegratedReconstructor>()->get_texture_data()->at(x, y, z);
                        itkVolume->SetPixel({{x, y, z}}, pixel_value);
                    }
                }
            }

            /* // Save the ITK Image as an MHA file
            using WriterType = itk::ImageFileWriter<ImageType>;
            WriterType::Pointer writer = WriterType::New();
            writer->SetFileName("C:/Users/SURGROB7/reconstruction_results.mha");
            writer->SetInput(itkVolume);
            writer->Update(); */


            integrated_volume->cast<curan::image::IntegratedReconstructor>()->reset();
            stopping_condition = true; // apagar isto


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
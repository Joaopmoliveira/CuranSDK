#include "imageprocessing/VolumeReconstructorBoxDefiner.h"
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>
#include "Mathematics/MinimumVolumeBox3.h"
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"

constexpr long width = 100;
constexpr long height = 100;
float spacing[3] = {0.01 , 0.01 , 0.01};
float final_spacing [3] = {0.01 , 0.01 , 0.01};

using imagetype = curan::image::VolumeReconstructorBoxDefiner::output_type;

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
			auto val = 255.0;//*(std::sqrt(y*y+x*x)/std::sqrt((height-1)*(height-1)+(width-1)*(width-1)));
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

int main(){

    try{
	    std::vector<imagetype::Pointer> image_array;
	    //create_array_of_images(image_array);
        create_array_of_linear_images_in_x_direction(image_array);
        
        curan::image::VolumeReconstructorBoxDefiner box_class;

       /*  box_class.add_frames(image_array);
        box_class.update(); */
        std::cout << "final\n";

        std::printf("started volumetric reconstruction\n");
        size_t counter = 0;

        for(auto img : image_array){
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            box_class.add_frame(img);
            box_class.update();

            std::chrono::steady_clock::time_point elapsed_for_reconstruction = std::chrono::steady_clock::now();

            auto val_elapsed_for_reconstruction = (int)std::chrono::duration_cast<std::chrono::microseconds>(elapsed_for_reconstruction - begin).count();

            //std::printf("added image (volume reconstruction %d)\n",val_elapsed_for_reconstruction);

            ++counter;
        }
        curan::image::VolumeReconstructorBoxDefiner::array_type box_data;
        
        box_class.get_final_volume_vertices(box_data);

        for(const auto& arr : box_data)
            std::printf("( %f , %f , %f )\n",arr[0],arr[1],arr[2]);

        /* std::cout << box_data[0][0] << std::endl;
        std::cout << box_data[0][1] << std::endl;
        std::cout << box_data[0][2] << std::endl;
        std::cout << box_data[1][0] << std::endl;
        std::cout << box_data[1][1] << std::endl;
        std::cout << box_data[1][2] << std::endl;
        std::cout << box_data[2][0] << std::endl;
        std::cout << box_data[2][1] << std::endl;
        std::cout << box_data[2][2] << std::endl;
        std::cout << box_data[3][0] << std::endl;
        std::cout << box_data[3][1] << std::endl;
        std::cout << box_data[3][2] << std::endl;
        std::cout << box_data[4][0] << std::endl;
        std::cout << box_data[4][1] << std::endl;
        std::cout << box_data[4][2] << std::endl;
        std::cout << box_data[5][0] << std::endl;
        std::cout << box_data[5][1] << std::endl;
        std::cout << box_data[5][2] << std::endl;
        std::cout << box_data[6][0] << std::endl;
        std::cout << box_data[6][1] << std::endl;
        std::cout << box_data[6][2] << std::endl;
        std::cout << box_data[7][0] << std::endl;
        std::cout << box_data[7][1] << std::endl;
        std::cout << box_data[7][2] << std::endl; */

    } catch(std::exception& e) {
        std::cout << "exception was throuwn with error message :" << e.what() << std::endl;
    }

    return 0;
}
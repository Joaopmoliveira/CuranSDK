#include "imageprocessing/BoundingBox4Reconstruction.h"
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

using imagetype = curan::image::BoundingBox4Reconstruction::output_type;

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

	/* image_origin[0] = 0.0;
	image_origin[1] = 0.0;
	image_origin[2] = 0.0; */

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

        image_origin[0] = 0.0;
	    image_origin[1] = 0.0;
	    image_origin[2] = 1.0/(width-1)*z;

        image->SetRegions(region_1);
        image->SetDirection(image_orientation);
        image->SetOrigin(image_origin);
        image->SetSpacing(spacing);
        image->Allocate();

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
        
        curan::image::BoundingBox4Reconstruction box_class;

       /*  box_class.add_frames(image_array);
        box_class.update(); */

        std::printf("started volumetric reconstruction\n");
        size_t counter = 0;

        for(auto img : image_array){
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            box_class.add_frame(img);
            box_class.update();

            std::chrono::steady_clock::time_point elapsed_for_bound_box = std::chrono::steady_clock::now();

            auto val_elapsed_for_bound_box = (int)std::chrono::duration_cast<std::chrono::microseconds>(elapsed_for_bound_box - begin).count();

            //std::printf("added image - elapsed time: %d microseconds\n",val_elapsed_for_bound_box);

            ++counter;
        }
        //curan::image::BoundingBox4Reconstruction::array_type box_data;
        
        //box_class.get_final_volume_vertices(box_data);

        auto box_data = box_class.get_final_volume_vertices();

        std::array<gte::Vector3<double>, 8> current_corners;
		current_corners[0] = box_data.center + box_data.axis[0] * box_data.extent[0] - box_data.axis[1] * box_data.extent[1] + box_data.axis[2] * box_data.extent[2];
		current_corners[1] = box_data.center + box_data.axis[0] * box_data.extent[0] + box_data.axis[1] * box_data.extent[1] + box_data.axis[2] * box_data.extent[2];
		current_corners[2] = box_data.center - box_data.axis[0] * box_data.extent[0] + box_data.axis[1] * box_data.extent[1] + box_data.axis[2] * box_data.extent[2];
		current_corners[3] = box_data.center - box_data.axis[0] * box_data.extent[0] - box_data.axis[1] * box_data.extent[1] + box_data.axis[2] * box_data.extent[2];
		current_corners[4] = box_data.center + box_data.axis[0] * box_data.extent[0] - box_data.axis[1] * box_data.extent[1] - box_data.axis[2] * box_data.extent[2];
		current_corners[5] = box_data.center + box_data.axis[0] * box_data.extent[0] + box_data.axis[1] * box_data.extent[1] - box_data.axis[2] * box_data.extent[2];
		current_corners[6] = box_data.center - box_data.axis[0] * box_data.extent[0] + box_data.axis[1] * box_data.extent[1] - box_data.axis[2] * box_data.extent[2];
		current_corners[7] = box_data.center - box_data.axis[0] * box_data.extent[0] - box_data.axis[1] * box_data.extent[1] - box_data.axis[2] * box_data.extent[2];

        std::cout << "Final box vertices" << std::endl;
        for(const auto& arr : current_corners)
            std::printf("( %f , %f , %f )\n",arr[0],arr[1],arr[2]);

        std::printf("started volumetric reconstruction agani\n");
        //size_t counter = 0;

        for(auto img : image_array){
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            box_class.add_frame(img);
            box_class.update();

            std::chrono::steady_clock::time_point elapsed_for_bound_box = std::chrono::steady_clock::now();

            auto val_elapsed_for_bound_box = (int)std::chrono::duration_cast<std::chrono::microseconds>(elapsed_for_bound_box - begin).count();

            //std::printf("added image - elapsed time: %d microseconds\n",val_elapsed_for_bound_box);

            ++counter;
        }


    } catch(std::exception& e) {
        std::cout << "exception was throuwn with error message :" << e.what() << std::endl;
    }

    return 0;
}
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

constexpr long width = 500;
constexpr long height = 500;
float spacing[3] = {0.002 , 0.002 , 0.002};
float final_spacing [3] = {0.002 , 0.002 , 0.002};

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

        image->SetRegions(region_1);
        image->SetDirection(image_orientation);
        image->SetOrigin(image_origin);
        image->SetSpacing(spacing);
        image->Allocate();

        using IteratorType = itk::ImageRegionIteratorWithIndex<imagetype>;
        IteratorType outputIt(image, image->GetRequestedRegion());
        for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
            imagetype::IndexType idx = outputIt.GetIndex();
            imagetype::PointType world_pos;
            image->TransformIndexToPhysicalPoint(idx,world_pos);
            constexpr float frequency = 2.0;
            unsigned char val = 255.0*(sin(frequency*1.57079632679*world_pos[0])+1)*0.5*(cos(frequency*1.57079632679*world_pos[1])+1)*0.5*(sin(frequency*1.57079632679*world_pos[2])+1)*0.5;
            val = 255.0;
            outputIt.Set(val);
        }
        desired_images.push_back(image);
    }
}

int main(){


    try{
	    std::vector<imagetype::Pointer> image_array;
	    //create_array_of_images(image_array);
        create_array_of_linear_images_in_x_direction(image_array);
        
        curan::image::VolumeReconstructorBoxDefiner class_m;

        //class_m.add_frames(image_array);

        std::printf("started volumetric reconstruction\n");
        size_t counter = 0;

        for(auto img : image_array){
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            class_m.add_frame(img);
            class_m.update();

            std::chrono::steady_clock::time_point elapsed_for_reconstruction = std::chrono::steady_clock::now();

            auto val_elapsed_for_reconstruction = (int)std::chrono::duration_cast<std::chrono::microseconds>(elapsed_for_reconstruction - begin).count();

            std::printf("added image (volume reconstruction %d)\n",val_elapsed_for_reconstruction);

            ++counter;
        }

        std::array<gte::Vector3<double>, 8> box_data;
        class_m.get_final_volume_vertices(box_data);

        std::cout << box_data[7][2] << std::endl;

    } catch(std::exception& e) {
        std::cout << "exception was throuwn with error message :" << e.what() << std::endl;
    }

    return 0;
}
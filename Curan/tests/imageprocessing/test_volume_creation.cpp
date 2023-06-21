#include "imageprocessing/VolumeReconstructor.h"

using imageType = curan::image::VolumeReconstructor::output_type;

void create_array_of_linear_images_in_x_direction(std::vector<imageType::Pointer>& desired_images){
    // the volume shoud be a box with spacing 1.0 mm in all directions with a size of 2 mm in each side 
    // to fill the volume we create an image with two images 

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

    constexpr long width = 3;
    constexpr long height = 3;

    imageType::Pointer image_1 = imageType::New();
    imageType::IndexType start_1;
    start_1[0] = 0; // first index on X
    start_1[1] = 0; // first index on Y
    start_1[2] = 0; // first index on Z

    imageType::SizeType size_1;
    size_1[0] = width; // size along X
    size_1[1] = height; // size along Y
    size_1[2] = 1; // size along Z

    imageType::RegionType region_1;
    region_1.SetSize(size_1);
    region_1.SetIndex(start_1);

    image_1->SetRegions(region_1);
    image_1->SetDirection(image_orientation);
    image_1->SetOrigin(image_origin);
    image_1->Allocate();
    curan::image::char_pixel_type pixel[width*height];
    curan::image::char_pixel_type pixel_value = 0;
    for(size_t y = 0; y < height ; ++y)
        for(size_t x = 0; x < width ; ++x){
            pixel[x+y*height] = pixel_value;
            ++pixel_value;
        }

    auto pointer = image_1->GetBufferPointer();
    std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);

    imageType::Pointer image_2 = imageType::New();
    imageType::IndexType start_2;
    start_2[0] = 0; // first index on X
    start_2[1] = 0; // first index on Y
    start_2[2] = 0; // first index on Z

    imageType::SizeType size_2;
    size_2[0] = width; // size along X
    size_2[1] = height; // size along Y
    size_2[2] = 1; // size along Z

    imageType::RegionType region_2;
    region_2.SetSize(size_2);
    region_2.SetIndex(start_2);

    image_2->SetRegions(region_2);
    image_2->SetDirection(image_orientation);
    image_2->SetOrigin(image_origin);
    image_2->Allocate();

    pointer = image_2->GetBufferPointer();
    std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);

    imageType::Pointer image_3 = imageType::New();
    imageType::IndexType start_3;
    start_3[0] = 0; // first index on X
    start_3[1] = 0; // first index on Y
    start_3[2] = 0; // first index on Z

    imageType::SizeType size_3;
    size_3[0] = width; // size along X
    size_3[1] = height; // size along Y
    size_3[2] = 1; // size along Z

    imageType::RegionType region_3;
    region_3.SetSize(size_3);
    region_3.SetIndex(start_3);

    image_3->SetRegions(region_3);
    image_3->SetDirection(image_orientation);
    image_3->SetOrigin(image_origin);
    image_3->Allocate();

    pointer = image_3->GetBufferPointer();
    std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);

    desired_images.push_back(image_1);
    desired_images.push_back(image_2);
    desired_images.push_back(image_3);
}

void create_array_of_linear_images_in_y_direction(std::vector<imageType::Pointer>& desired_images){


}

void create_array_of_linear_images_in_z_direction(std::vector<imageType::Pointer>& desired_images){


}

void create_array_of_rotational_images_around_the_x_axis(std::vector<imageType::Pointer>& desired_images){


}

void create_array_of_rotational_images_around_the_y_axis(std::vector<imageType::Pointer>& desired_images){


}

void create_array_of_rotational_images_around_the_z_axis(std::vector<imageType::Pointer>& desired_images){


}

int main(){
    { // first test creates two stacked images in the x direction and reconstructs the volume
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_linear_images_in_x_direction(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }

 /*    {// first test creates two stacked images in the x direction and reconstructs the volume
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_linear_images_in_y_direction(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }


    {// first test creates two stacked images in the x direction and reconstructs the volume
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_linear_images_in_z_direction(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }


    {  // first test creates two stacked images in the x direction and reconstructs the volume 
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_rotational_images_around_the_x_axis(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }

    {// first test creates two stacked images in the x direction and reconstructs the volume
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_rotational_images_around_the_y_axis(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }

    {// first test creates two stacked images in the x direction and reconstructs the volume
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_rotational_images_around_the_z_axis(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    } */
    return 0;
}
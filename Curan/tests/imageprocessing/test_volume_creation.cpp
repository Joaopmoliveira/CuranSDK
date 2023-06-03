#include "imageprocessing/VolumeReconstructor.h"

using imageType = curan::image::VolumeReconstructor::output_type;

void create_array_of_linear_images_in_x_direction(std::vector<imageType::Pointer>& desired_images){
    // the volume shoud be a box with spacing 1.0 mm in all directions with a size of 2 mm in each side 
    // to fill the volume we create an image with two images 

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
    image_1->Allocate();

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
    image_2->Allocate();

    desired_images.push_back(image_1);
    desired_images.push_back(image_2);
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

    {// first test creates two stacked images in the x direction and reconstructs the volume
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
    }
    return 0;
}
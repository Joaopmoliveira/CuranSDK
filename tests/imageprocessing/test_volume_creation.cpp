#include "imageprocessing/VolumeReconstructor.h"

using imageType = curan::image::VolumeReconstructor::output_type;

void create_array_of_linear_images_in_x_direction(std::vector<imageType::Pointer>& desired_images){


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
    // this scripts tests the volume reconstruction implementation 
    imageType::Pointer image_to_insert = imageType::New();
    curan::image::VolumeReconstructor volume_reconstructor;
    volume_reconstructor.add_frame(image_to_insert);
    return 0;
}
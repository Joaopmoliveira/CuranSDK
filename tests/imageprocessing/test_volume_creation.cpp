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
    {
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_linear_images_in_x_direction(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }

    {
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_linear_images_in_y_direction(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }


    {
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_linear_images_in_z_direction(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }


    {   
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_rotational_images_around_the_x_axis(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }

    {
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_rotational_images_around_the_y_axis(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }

    {
        curan::image::VolumeReconstructor volume_reconstructor;
        std::vector<imageType::Pointer> images;
        create_array_of_rotational_images_around_the_z_axis(images);
        volume_reconstructor.add_frames(images);
        volume_reconstructor.update();
    }
    return 0;
}
#include "imageprocessing/StaticReconstructor.h"
#include <optional>

constexpr long width = 500;
constexpr long height = 500;
float spacing[3] = {0.002 , 0.002 , 0.002};
float final_spacing [3] = {0.002,0.002, 0.002};

void create_array_of_linear_images_in_x_direction(std::vector<curan::image::StaticReconstructor::output_type::Pointer>& desired_images){
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
	    image_origin[2] = std::sin((3.1415/2.0)*(1.0/(width-1))*z);

        auto pointer = image->GetBufferPointer();
        std::memcpy(pointer,pixel,sizeof(curan::image::char_pixel_type)*width*height);
        desired_images.push_back(image);
    }
}

int main(){
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

        auto reconstruction_thread_pool = curan::utilities::ThreadPool::create(9);
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::printf("started volumetric reconstruction\n");
        size_t counter = 0;
	    for(auto img : image_array){
		    reconstructor.add_frame(img);
		    reconstructor.multithreaded_update(reconstruction_thread_pool);
            ++counter;
	    }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        auto val = (int)std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        std::printf("finished volumetric reconstruction (time taken: %d microseconds)\n",val);
	    std::printf("per image time: %f microseconds/image\n",val/(double)counter);
        return 0;
    }catch(std::exception& e){
        std::cout << "exception was throuwn with error message :" << e.what() << std::endl;
        return 1;
    }
};
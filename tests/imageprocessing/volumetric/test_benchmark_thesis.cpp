#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "itkImportImageFilter.h"
#include "rendering/Renderable.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include <itkImage.h>
#include <nlohmann/json.hpp> // Include the JSON library
#include <optional>
#include <random> // Include the random header for generating random numbers

struct VolumeInformation{
    const long width = 400;
    const long height = 400;
    float spacing[3];
    float final_spacing[3];

    VolumeInformation(int in_width): width{in_width},height{in_width} {
        spacing[0] = 1.0 / width;
        spacing[1] = 1.0 / width;
        spacing[2] = 1.0 / width;

        final_spacing[0] = 2.0 / 300;
        final_spacing[1] = 2.0 / 300;
        final_spacing[2] = 2.0 / 300;
    }
};


using imagetype = itk::Image<unsigned char, 3>;
using IteratorType = itk::ImageRegionIteratorWithIndex<imagetype>;

auto allocate_image(double* euler_angles,const VolumeInformation& info) {
  imagetype::Pointer image = imagetype::New();
  itk::Matrix<double> image_orientation;
  itk::Point<double> image_origin;

  image_origin[0] = 0.0;
  image_origin[1] = 0.0;
  image_origin[2] = 0.0;

  imagetype::IndexType start;
  start[0] = 0; // first index on X
  start[1] = 0; // first index on Y
  start[2] = 0; // first index on Z

  imagetype::SizeType size;
  size[0] = info.width;  // size along X
  size[1] = info.height; // size along Y
  size[2] = 1;      // size along Z

  imagetype::RegionType region_1;
  region_1.SetSize(size);
  region_1.SetIndex(start);

  euler_angles[0] += 0.025;

  double t1 = cos(euler_angles[2]);
  double t2 = sin(euler_angles[2]);
  double t3 = cos(euler_angles[1]);
  double t4 = sin(euler_angles[1]);
  double t5 = cos(euler_angles[0]);
  double t6 = sin(euler_angles[0]);

  image_orientation(0, 0) = t1 * t3;
  image_orientation(1, 0) = t2 * t3;
  image_orientation(2, 0) = -t4;

  image_orientation(0, 1) = t1 * t4 * t6 - t2 * t5;
  image_orientation(1, 1) = t1 * t5 + t2 * t4 * t6;
  image_orientation(2, 1) = t3 * t6;

  image_orientation(0, 2) = t2 * t6 + t1 * t4 * t5;
  image_orientation(1, 2) = t2 * t4 * t5 - t1 * t6;
  image_orientation(2, 2) = t3 * t5;

  image->SetRegions(region_1);
  image->SetDirection(image_orientation);
  image->SetOrigin(image_origin);
  image->SetSpacing(info.spacing);
  image->Allocate();

  IteratorType outputIt(image, image->GetRequestedRegion());
  for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt) {
    imagetype::IndexType idx = outputIt.GetIndex();
    auto val = (double)idx[0] * idx[0] + (double)idx[1] * idx[1];
    val /= (double)info.height * info.height + (double)info.height * info.height;
    val *= 255.0;
    outputIt.Set(val);
  }
  return image;
}

void update_image_coordiantes(imagetype::Pointer &image,double *previous_euler_angles,const VolumeInformation& info) {
  itk::Matrix<double> image_orientation;
  itk::Point<double> image_origin;

  image_origin[0] = 0.0;
  image_origin[1] = 0.0;
  image_origin[2] = 0.0;

  imagetype::IndexType start;
  start[0] = 0; // first index on X
  start[1] = 0; // first index on Y
  start[2] = 0; // first index on Z

  imagetype::SizeType size;
  size[0] = info.width;  // size along X
  size[1] = info.height; // size along Y
  size[2] = 1;      // size along Z

  imagetype::RegionType region_1;
  region_1.SetSize(size);
  region_1.SetIndex(start);

  previous_euler_angles[0] += 0.05;

  double t1 = cos(previous_euler_angles[2]);
  double t2 = sin(previous_euler_angles[2]);
  double t3 = cos(previous_euler_angles[1]);
  double t4 = sin(previous_euler_angles[1]);
  double t5 = cos(previous_euler_angles[0]);
  double t6 = sin(previous_euler_angles[0]);

  image_orientation(0, 0) = t1 * t3;
  image_orientation(1, 0) = t2 * t3;
  image_orientation(2, 0) = -t4;

  image_orientation(0, 1) = t1 * t4 * t6 - t2 * t5;
  image_orientation(1, 1) = t1 * t5 + t2 * t4 * t6;
  image_orientation(2, 1) = t3 * t6;

  image_orientation(0, 2) = t2 * t6 + t1 * t4 * t5;
  image_orientation(1, 2) = t2 * t4 * t5 - t1 * t6;
  image_orientation(2, 2) = t3 * t5;

  image->SetRegions(region_1);
  image->SetDirection(image_orientation);
  image->SetOrigin(image_origin);
  image->SetSpacing(info.spacing);
}

int run_window(int size_to_pass,size_t thread_pool_size) {
try{
    double euler_angles[3] = {0, 0, 0};
    VolumeInformation volume_size_info{size_to_pass};
    auto reconstruction_thread_pool = curan::utilities::ThreadPool::create(thread_pool_size);
    auto image = allocate_image(&euler_angles[0],volume_size_info);


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

    std::array<double, 3> vol_origin = {-1.0, -1.0, -1.0};
    std::array<double, 3> vol_spacing = {volume_size_info.final_spacing[0], volume_size_info.final_spacing[1],
                                         volume_size_info.final_spacing[2]};
    std::array<double, 3> vol_size = {2.0, 2.0, 2.0};
    std::array<std::array<double, 3>, 3> vol_direction;
    vol_direction[0] = {1.0, 0.0, 0.0};
    vol_direction[1] = {0.0, 1.0, 0.0};
    vol_direction[2] = {0.0, 0.0, 1.0};

    curan::image::IntegratedReconstructor::Info recon_info{vol_spacing, vol_origin, vol_size, vol_direction};
    auto integrated_volume = curan::image::IntegratedReconstructor::make(recon_info);
    integrated_volume->cast<curan::image::IntegratedReconstructor>()->set_compound(curan::image::reconstruction::Compounding::LATEST_COMPOUNDING_MODE).set_interpolation(curan::image::reconstruction::Interpolation::LINEAR_INTERPOLATION);
    window << integrated_volume;

    size_t incrementer = 0;
    std::vector<double> timings;
    timings.reserve(1000);
    std::chrono::steady_clock::time_point begin =std::chrono::steady_clock::now();
    while (window.run_once() && (incrementer < 1000)) {
      update_image_coordiantes(image,&euler_angles[0],volume_size_info);
      integrated_volume->cast<curan::image::IntegratedReconstructor>()->add_frame(image);
      //std::chrono::steady_clock::time_point begin =std::chrono::steady_clock::now();
      integrated_volume->cast<curan::image::IntegratedReconstructor>()->multithreaded_update(reconstruction_thread_pool);
      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      if(incrementer>100)
        timings.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() *1e-6);
      ++incrementer;
      begin =std::chrono::steady_clock::now();
    }

    double sum = std::accumulate(timings.begin(), timings.end(), 0.0);
    double average = sum / timings.size();
    double sum_average_subtraction{0.0};
    for (auto timing : timings)
      sum_average_subtraction += (timing - average) * (timing - average);

    double stadard_deviation = sum_average_subtraction / timings.size();

    std::printf("%d %d %f %f\n",(int)thread_pool_size,size_to_pass, average, stadard_deviation);
  } catch (std::exception &e) {
    std::cout << "failed: " << e.what() << std::endl;
  }
  return 0;
}

int main(){
    std::vector<int> image_sizes{100 , 200 , 300 , 400 , 500 , 600 , 700 , 800 , 900 , 1000};
    std::vector<size_t> n_threads{1,2,4,8};

    for(auto thread_size : n_threads)
        for(auto size : image_sizes)
            run_window(size,thread_size);
    
    return 0;
}
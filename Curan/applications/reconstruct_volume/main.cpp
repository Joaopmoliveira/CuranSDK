/**

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," 
without warranty of any kind, including without limitation the warranties 
of merchantability, fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable 
for any particular purpose. In no event shall KUKA be responsible for loss 
or damages arising from the installation or use of the Software, 
including but not limited to any indirect, punitive, special, incidental 
or consequential damages of any character including, without limitation, 
damages for loss of goodwill, work stoppage, computer failure or malfunction, 
or any and all other commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by KUKA. 
Should the Software prove defective, KUKA is not liable for the entire cost 
of any service and repair.


COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2015 
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned 
to KUKA Roboter GmbH immediately upon request.  
This material and the information illustrated or contained herein may not be used, 
reproduced, stored in a retrieval system, or transmitted in whole 
or in part in any way - electronic, mechanical, photocopying, recording, 
or otherwise, without the prior written consent of KUKA Roboter GmbH.  

\file
\version {1.9}
*/

#include <fstream>
#include <csignal>
#include <chrono>
#include <thread>
#include "link_demo.h"
#include <nlohmann/json.hpp>
#include "utils/Reader.h"
#include <itkImage.h>
#include "itkImageFileWriter.h"

// Variable with the default ID of the robotic system
constexpr size_t portID = 30200;

namespace
{
  volatile std::sig_atomic_t gSignalStatus;
}

void signal_handler(int signal)
{
  std::cout << "Hey, just recevied a signal\n";
  gSignalStatus = signal;
}

int main (int argc, char** argv)
{
   std::signal(SIGINT, signal_handler);
   auto robot_state = SharedRobotState::make_shared();

try{
   // We need to read the JSON configuration file to get the calibrated configuration of the ultrasound image
   nlohmann::json calibration_data;
	std::ifstream in("C:/Users/SURGROB7/optimization_result.json");
	in >> calibration_data;

   std::string timestamp = calibration_data["timestamp"];
	std::string homogenenous_transformation = calibration_data["homogeneous_transformation"];
	double error = calibration_data["optimization_error"];
	std::printf("Using calibration with average error of : %f\n on the date ",error);
   std::cout << timestamp << std::endl;
   std::stringstream matrix_strm;
   matrix_strm << homogenenous_transformation;
   auto calibration_matrix = curan::utilities::convert_matrix(matrix_strm);
   std::cout << "with the homogeneous matrix :\n" <<  calibration_matrix << std::endl;
   for(size_t row = 0 ; row < calibration_matrix.rows(); ++row)
      for(size_t col = 0; col < calibration_matrix.cols(); ++col)
        robot_state->calibration_matrix(col,row) = calibration_matrix(row,col);

} catch(...){
       std::cout << "failure to read the calibration data, \nplease provide a file \"optimization_result.json\" \nwith the calibration of the set up";
      return 1;
}

   curan::renderable::Window::Info info;
   info.api_dump = false;
   info.display = "";
   info.full_screen = false;
   info.is_debug = false;
   info.screen_number = 0;
   info.title = "myviewer";
   curan::renderable::Window::WindowSize size{2000, 1200};
   info.window_size = size;
   curan::renderable::Window window{info};
   robot_state->window_pointer = &window;


   std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
   curan::renderable::SequencialLinks::Info create_info;
   create_info.convetion = vsg::CoordinateConvention::Y_UP;
   create_info.json_path = robot_path;
   create_info.number_of_links = 8;
   robot_state->robot = curan::renderable::SequencialLinks::make(create_info);
   window << robot_state->robot;

try{
   // We need to read the JSON configuration file to get the calibrated configuration of the ultrasound image
   nlohmann::json specified_box;
	std::ifstream in("C:/Users/SURGROB7/specified_box.json");
	in >> specified_box;

   std::string timestamp_box = specified_box["timestamp"];

   std::stringstream spacing_box_info;
	std::string spacing_box = specified_box["spacing"];
   spacing_box_info << spacing_box;
   Eigen::MatrixXd spacing = curan::utilities::convert_matrix(spacing_box_info);
   std::string origin_box = specified_box["origin"];
   std::stringstream origin_box_info;
   origin_box_info << origin_box;
   Eigen::MatrixXd origin = curan::utilities::convert_matrix(origin_box_info);
   std::string size_box = specified_box["size"];
   std::stringstream size_box_info;
   size_box_info << size_box;

   Eigen::MatrixXd size = curan::utilities::convert_matrix(size_box_info);
   std::string direction_box = specified_box["direction"];
   std::stringstream direction_box_info;
   direction_box_info << direction_box;

   Eigen::MatrixXd direction = curan::utilities::convert_matrix(direction_box_info);

   if(spacing.rows()!=1 || spacing.cols()!=3 ) 
      throw std::runtime_error("The supplied spacing has an incorrect dimension (expected : [1x3])");

   if(origin.rows()!=1 || origin.cols()!=3 ) 
      throw std::runtime_error("The supplied origin has an incorrect dimension (expected : [1x3])");

   if(size.rows()!=1 || size.cols()!=3 ) 
      throw std::runtime_error("The supplied size has an incorrect dimension (expected : [1x3])");

   if(direction.rows()!=3 || direction.cols()!=3 ) 
      throw std::runtime_error("The supplied direction has an incorrect dimension (expected : [3x3])");

   std::array<double,3> vol_origin = {origin(0,0),origin(0,1),origin(0,2)};
	std::array<double,3> vol_spacing = {spacing(0,0),spacing(0,1),spacing(0,2)};
	std::array<double,3> vol_size = {size(0,0),size(0,1),size(0,2)};
	std::array<std::array<double,3>,3> vol_direction;
	vol_direction[0] = {direction(0,0),direction(1,0),direction(2,0)};
	vol_direction[1] = {direction(0,1),direction(1,1),direction(2,1)};
	vol_direction[2] = {direction(0,2),direction(1,2),direction(2,2)};
	curan::image::IntegratedReconstructor::Info recon_info{vol_spacing,vol_origin,vol_size,vol_direction};
   
   robot_state->integrated_volume =  curan::image::IntegratedReconstructor::make(recon_info);
   robot_state->integrated_volume->cast<curan::image::IntegratedReconstructor>()->set_compound(curan::image::reconstruction::Compounding::LATEST_COMPOUNDING_MODE)
      .set_interpolation(curan::image::reconstruction::Interpolation::LINEAR_INTERPOLATION);
   
   itk::Size<3U>  output_size;
   output_size = robot_state->integrated_volume->cast<curan::image::IntegratedReconstructor>()->get_output_size();
   size_t width =   output_size[0] ;
   size_t height =   output_size[1] ;
   size_t depth =   output_size[2] ;
   
   using PixelType = float;
   using ImageType = itk::Image<PixelType, 3>;
   ImageType::Pointer itkVolume = ImageType::New();

   // Set the size, spacing, and origin of the ITK Image


   itkVolume->SetRegions(output_size);
   itkVolume->SetSpacing(vol_spacing.data());
   itkVolume->SetOrigin(vol_origin.data());

   itkVolume->Allocate();

   // Copy the volumetric data from integrated_volume to itkVolume
   for (long long z = 0; z < depth; ++z) {
         for (long long y = 0; y < height; ++y) {
            for (long long x = 0; x < width; ++x) {
               float pixel_value = robot_state->integrated_volume->cast<curan::image::IntegratedReconstructor>()->get_texture_data()->at(x, y, z);
               itkVolume->SetPixel({{x, y, z}}, pixel_value);
            }
         }
   }

   // Save the ITK Image as an MHA file
   using WriterType = itk::ImageFileWriter<ImageType>;
   WriterType::Pointer writer = WriterType::New();
   writer->SetFileName("C:/Users/SURGROB7/reconstruction_results.mha");
   writer->SetInput(itkVolume);
   writer->Update();

   window << robot_state->integrated_volume;

} catch(...){
       std::cout << "failure to read the calibration data, \nplease provide a file \"optimization_result.json\" \nwith the calibration of the set up";
      return 1;
}

   auto communication_callable = [robot_state](){
      communication(robot_state);
   };
   //here I should lauch the thread that does the communication and renders the image above the robotic system 
   std::thread communication_thread(communication_callable);

   window.run();
   robot_state->kill_yourself();
   communication_thread.join();
   std::cout << "terminated the program" << std::endl;
   return 0;
}

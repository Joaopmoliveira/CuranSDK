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

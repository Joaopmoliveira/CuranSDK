#include <fstream>
#include <csignal>
#include <chrono>
#include <thread>
#include "link_demo.h"
#include <nlohmann/json.hpp>
#include <cmath>
#include "utils/Reader.h"
#include "processmanagement/ChildProcess.h"

namespace
{
  volatile std::sig_atomic_t gSignalStatus;
}

void signal_handler(int signal)
{
  std::cout << "Hey, just recevied a signal\n";
  gSignalStatus = signal;
}

void interface(vsg::CommandBuffer& cb,std::shared_ptr<SharedRobotState>& robot_state){
   ImGui::Begin("Box Specification Selection");
   static float t = 0;
   t += ImGui::GetIO().DeltaTime;
   static bool local_record_data = false;
   ImGui::Checkbox("New Box Selection", &local_record_data); 
   robot_state->restart_volumetric_box.store(local_record_data);
   ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
   ImGui::End();
}

int main (int argv, char** argc)
{
   asio::io_context context;
   std::signal(SIGINT, signal_handler);

	std::unique_ptr<curan::process::ChildProcess> child;
	using namespace curan::communication;

	if(argv>2){ 
      // we were supplied with more arguments than expected, therefore we assume that there is no parent
		std::cout << "the supplied arguments are too many, \nthe program either accepts the one or two arguments\n";
	} else if(argv==2) {
		child = std::make_unique<curan::process::ChildProcess>(context, std::chrono::milliseconds(100), 10);
	} else {
		// we were supplied with less arguments than expected, therefore we assume that there is no parent
	}


   auto robot_state = SharedRobotState::make_shared();
   try{
   // We need to read the JSON configuration file to get the calibrated configuration of the ultrasound image
   nlohmann::json calibration_data;
	std::ifstream in(CURAN_COPIED_RESOURCE_PATH"/optimization_result.json");
	in >> calibration_data;
   std::string timestamp = calibration_data["timestamp"];
	std::string homogenenous_transformation = calibration_data["homogeneous_transformation"];
	double error = calibration_data["optimization_error"];
	std::printf("Using calibration with average error of : %f\n on the date ",error);
   std::cout << timestamp << std::endl;
   std::stringstream matrix_strm;
   matrix_strm << homogenenous_transformation;
   auto calibration_matrix = curan::utilities::convert_matrix(matrix_strm,',');
   std::cout << "with the homogeneous matrix :\n" <<  calibration_matrix << std::endl;
   for(Eigen::Index row = 0 ; row < calibration_matrix.rows(); ++row)
      for(Eigen::Index col = 0; col < calibration_matrix.cols(); ++col)
        robot_state->calibration_matrix(col,row) = calibration_matrix(row,col);

   } catch(...){
       std::cout << "failure to read the calibration data, \nplease provide a file \"optimization_result.json\" \nwith the calibration of the set up";
      return 1;
   }

   curan::renderable::ImGUIInterface::Info info_gui{[&](vsg::CommandBuffer& cb){interface(cb,robot_state);}};
   auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
   curan::renderable::Window::Info info;
   info.api_dump = false;
   info.display = "";
   info.full_screen = false;
   info.is_debug = false;
   info.screen_number = 0;
   info.title = "myviewer";
   info.imgui_interface = ui_interface;
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

   auto communication_callable = [robot_state,&context](){
      communication(robot_state,context);
   };
   //here I should lauch the thread that does the communication and renders the image above the robotic system 
   std::thread communication_thread(communication_callable);

   window.run();
   robot_state->kill_yourself();
   communication_thread.join();

   auto return_current_time_and_date = [](){
	   auto now = std::chrono::system_clock::now();
    	auto in_time_t = std::chrono::system_clock::to_time_t(now);

    	std::stringstream ss;
    	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
   		return ss.str();
	};

   auto final_box = robot_state->box_class.get_final_volume_vertices();
   vsg::dmat3 rotation_0_1;
   Eigen::Matrix<double,3,3> final_box_orientation;
    for(size_t col = 0; col < 3; ++col)
        for(size_t row = 0; row < 3; ++row){
            rotation_0_1(col,row) = final_box.axis[col][row];
            final_box_orientation(row,col) = final_box.axis[col][row];
        }

    vsg::dvec3 position_of_center_in_global_frame;
    position_of_center_in_global_frame[0] = final_box.center[0];
    position_of_center_in_global_frame[1] = final_box.center[1];
    position_of_center_in_global_frame[2] = final_box.center[2];

    vsg::dvec3 position_in_local_box_frame;
    position_in_local_box_frame[0] = final_box.extent[0];
    position_in_local_box_frame[1] = final_box.extent[1];
    position_in_local_box_frame[2] = final_box.extent[2]; 

    auto global_corner_position = position_of_center_in_global_frame-rotation_0_1*position_in_local_box_frame;


   nlohmann::json specified_box;
	specified_box["timestamp"] = return_current_time_and_date();

   constexpr size_t maximum_float_size = 125.0e6*0.5;
   double new_spacing = std::cbrt((2*final_box.extent[0] *2*final_box.extent[1] *2*final_box.extent[2] )/(maximum_float_size));

   std::stringstream ss;
   ss << new_spacing << " , " << new_spacing << " , " << new_spacing;
	specified_box["spacing"] = ss.str();
   ss.str("");
   ss << global_corner_position[0] << " , "<< global_corner_position[1] << " , " << global_corner_position[2];
   specified_box["origin"] = ss.str();
   ss.str("");
   ss << 2*final_box.extent[0] << " , " << 2*final_box.extent[1] << " , " << 2*final_box.extent[2];
   specified_box["size"] = ss.str();
   ss.str("");
   Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", " ", " ");
   ss << final_box_orientation.format(CleanFmt);
   specified_box["direction"] = ss.str();

	// write prettified JSON to another file
	std::ofstream o(CURAN_COPIED_RESOURCE_PATH"/specified_box.json");
	o << specified_box;
	std::cout << specified_box << std::endl;
   return 0;
}

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

Eigen::MatrixXd convert_matrix(std::stringstream& data)
{
 
    // the inspiration for creating this function was drawn from here (I did NOT copy and paste the code)
    // https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
     
    // the input is the file: "fileToOpen.csv":
    // a,b,c
    // d,e,f
    // This function converts input file data into the Eigen matrix format
 
 
 
    // the matrix entries are stored in this variable row-wise. For example if we have the matrix:
    // M=[a b c 
    //    d e f]
    // the entries are stored as matrixEntries=[a,b,c,d,e,f], that is the variable "matrixEntries" is a row vector
    // later on, this vector is mapped into the Eigen matrix format
    std::vector<double> matrixEntries;
 
    // this variable is used to store the row of the matrix that contains commas 
     std::string matrixRowString;
 
    // this variable is used to store the matrix entry;
     std::string matrixEntry;
 
    // this variable is used to track the number of rows
    int matrixRowNumber = 0;
 
 
    while (getline(data, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
    {
         std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.
 
        while (getline(matrixRowStringStream, matrixEntry, ',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
        {
            matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
        }
        matrixRowNumber++; //update the column numbers
    }
 
    // here we convet the vector variable into the matrix and return the resulting object, 
    // note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
    return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
 
}

int main (int argc, char** argv)
{
   std::signal(SIGINT, signal_handler);

   auto robot_state = SharedRobotState::make_shared();
   try{
   // We need to read the JSON configuration file to get the calibrated configuration of the ultrasound image
   nlohmann::json calibration_data;
	std::ifstream in("optimization_result.json");
	in >> calibration_data;
   std::string timestamp = calibration_data["timestamp"];
	std::string homogenenous_transformation = calibration_data["homogeneous_transformation"];
	double error = calibration_data["optimization_error"];
	std::printf("Using calibration with average error of : %f\n on the date",error);
   std::cout << timestamp << std::endl;
   std::stringstream matrix_strm;
   matrix_strm << homogenenous_transformation;
   robot_state->calibration_matrix = convert_matrix(matrix_strm);
   std::cout << "with the homogeneous matrix :\n" <<  robot_state->calibration_matrix << std::endl;
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

    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    state->robot = curan::renderable::SequencialLinks::make(create_info);
    window << state->robot;

    auto communication_callable = [state](){
        communication(state);
    };
    //here I should lauch the thread that does the communication and renders the image above the robotic system 
    std::thread communication_thread(communication_callable);

    kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here
	
	auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	auto iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

	// Attach tool
	double toolMass = 0.0;                                                                     // No tool for now
	RigidBodyDynamics::Math::Vector3d toolCOM = RigidBodyDynamics::Math::Vector3d::Zero(3, 1);
	RigidBodyDynamics::Math::Matrix3d toolInertia = RigidBodyDynamics::Math::Matrix3d::Zero(3, 3);
	auto myTool = std::make_unique<ToolData>(toolMass, toolCOM, toolInertia);

	robot->attachToolToRobotModel(myTool.get());

   RigidBodyDynamics::Math::VectorNd measured_torque = RigidBodyDynamics::Math::VectorNd::Zero(7,1);
   Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric
   Vector3d p_0_cur = Vector3d(0, 0, 0.045);
   RigidBodyDynamics::Math::MatrixNd Jacobian = RigidBodyDynamics::Math::MatrixNd::Zero(6, NUMBER_OF_JOINTS);

   auto robotRenderableCasted = state->robot->cast<curan::renderable::SequencialLinks>();

   while(window.run_once() && !state->should_kill_myself()) {
      auto current_reading = state->read();
      auto q_current = current_reading.getMeasuredJointPosition();
      auto tau_current = current_reading.getExternalTorque();

	   for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		   iiwa->q[i] = q_current[i];
         robotRenderableCasted->set(i,q_current[i]);
		   measured_torque[i] = tau_current[i];
	   }
      static RigidBodyDynamics::Math::VectorNd q_old = iiwa->q;
	   for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		   iiwa->qDot[i] = (q_current[i] - q_old[i]) / current_reading.getSampleTime();
	   }   

      robot->getMassMatrix(iiwa->M,iiwa->q);
	   iiwa->M(6,6) = 45 * iiwa->M(6,6);                                       // Correct mass of last body to avoid large accelerations
	   iiwa->Minv = iiwa->M.inverse();
	   robot->getCoriolisAndGravityVector(iiwa->c,iiwa->g,iiwa->q,iiwa->qDot);
	   robot->getWorldCoordinates(p_0_cur,iiwa->q,pointPosition,7);              // 3x1 position of flange (body = 7), expressed in base coordinates

      robot->getJacobian(Jacobian,iiwa->q,pointPosition,NUMBER_OF_JOINTS);

      q_old = iiwa->q;
   }
   communication_thread.join();
   std::cout << "terminated the program" << std::endl;
   return 0;
}

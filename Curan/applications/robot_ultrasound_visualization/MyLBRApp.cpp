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

#include "MyLBRClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <fstream>
#include <csignal>
#include <chrono>
#include <thread>
#include "link_demo.h"

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
   // We need to read the JSON configuration file to get the calibrated configuration of the ultrasound image
   // TODO

   std::signal(SIGINT, signal_handler);

   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /***************************************************************************/
   auto robot_state = SharedRobotState::make_shared();
   // create new client
	MyLBRClient client{robot_state};

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Configuration                                                         */
   /*                                                                         */
   /***************************************************************************/

   // create new udp connection	
   KUKA::FRI::UdpConnection connection;

   // pass connection and client to a new FRI client application
   KUKA::FRI::ClientApplication app(connection, client);
   
   // Connect client application to KUKA Sunrise controller.
   // Parameter NULL means: repeat to the address, which sends the data
   app.connect(portID, NULL);

   app.step(); //this blocks until the first message is read
   //we only want to lauch the visualization once one message
   //has been shared
   auto capture = [robot_state](){
      render(robot_state);
   };
   std::thread render_process(capture);

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Execution mainloop                                                    */
   /*                                                                         */
   /***************************************************************************/

   // repeatedly call the step routine to receive and process FRI packets
   try
   {
      bool success = true;
      while ( !gSignalStatus && success)
      {
         success = app.step();
      }
   } catch(...){
      robot_state->kill_yourself();
      return 1;
   }

   robot_state->kill_yourself();

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Dispose                                                               */
   /*                                                                         */
   /***************************************************************************/
   // disconnect from controller
   app.disconnect();
   render_process.join();
   std::cout << "terminated the program" << std::endl;
   return 0;
}

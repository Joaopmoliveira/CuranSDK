#include "robotutils/LBRController.h"
#include "robotutils/WallAvoidanceData.h"
#include "utils/Logger.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include <csignal>

curan::robotic::RobotLBR* robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal)
{
	if(robot_pointer)
        robot_pointer->cancel();
}

int main(){
    Eigen::Vector3d plane_point{{-0.658657 , 0.276677 , 0.404548}};
    Eigen::Vector3d plane_direction{{0.0 , 1.0 , 0.0}};
    std::unique_ptr<curan::robotic::WallAvoidanceData> handguinding_controller = std::make_unique<curan::robotic::WallAvoidanceData>(plane_point,plane_direction);
    curan::robotic::RobotLBR client{handguinding_controller.get()};
	try
	{
		curan::utilities::cout << "Lauching robot control thread\n";
		
		KUKA::FRI::UdpConnection connection;
		KUKA::FRI::ClientApplication app(connection, client);
		bool success = app.connect(DEFAULT_PORTID, NULL);
		success = app.step();
		while (success && client)
			success = app.step();
		app.disconnect();
		return 0;
	}
	catch (...)
	{
		std::cout << "robot control exception\n";
		return 1;
	}
}
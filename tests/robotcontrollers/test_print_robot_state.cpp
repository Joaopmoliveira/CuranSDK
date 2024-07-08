#include "robotutils/LBRController.h"
#include "robotutils/HandGuidance.h"
#include "utils/Logger.h"
#include "utils/TheadPool.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include <csignal>
#include <thread>
#include <chrono>

curan::robotic::RobotLBR* robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal)
{
	if(robot_pointer)
        robot_pointer->cancel();
}



int main(){
    std::unique_ptr<curan::robotic::HandGuidance> handguinding_controller = std::make_unique<curan::robotic::HandGuidance>();
    curan::robotic::RobotLBR client{handguinding_controller.get(),CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_mass_data.json",CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/robot_kinematic_limits.json"};
	auto pool = curan::utilities::ThreadPool::create(1);
	pool->submit(curan::utilities::Job{"printing_robot_state",[&](){
        const auto& reading_point = client.atomic_acess();
        while(client){
            auto current = reading_point.load(std::memory_order_relaxed);
			current.print_state = static_cast<curan::robotic::PrintInfo>(curan::robotic::PrintInfo::TRANSLATION | curan::robotic::PrintInfo::ROTATION);
            std::cout << current << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }});

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
        client.cancel();
		std::cout << "robot control exception\n";
		return 1;
	}
}
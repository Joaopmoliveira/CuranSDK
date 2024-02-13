#include "robotutils/LBRController.h"
#include "robotutils/HandGuidance.h"
#include "utils/Logger.h"
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
    curan::robotic::RobotLBR client{handguinding_controller.get()};

    std::thread thr{[&](){
        const auto& reading_point = client.atomic_acess();
        while(client){
            auto current = reading_point.load(std::memory_order_relaxed);
/*             std::cout << "translation raw:\n";
            for(const auto& t : current.translation)
                std::cout << " " << t; */
            std::cout << "\n";
            auto current_value = current.converteigen();
            std::cout << "=========================================\nrotation:\n" <<  current_value.rotation 
                      << "\ntranslation:" << current_value.translation.transpose() 
                      << "\njoints:\n" << current_value.q
                      << "\njacobian:\n" << current_value.jacobian
                      << "\nmass:\n" << current_value.massmatrix                                         
                      << "\n=========================================\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }};

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
        thr.join();
		return 0;
	}
	catch (...)
	{
        client.cancel();
        thr.join();
		std::cout << "robot control exception\n";
		return 1;
	}
}
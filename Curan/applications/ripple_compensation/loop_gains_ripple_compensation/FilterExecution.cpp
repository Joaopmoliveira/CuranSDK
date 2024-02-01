#include "MyLBRClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <fstream>
#include <csignal>
#include <chrono>

// Variable with the default ID of the robotic system
constexpr size_t portID = 30200;

int main (int argc, char** argv)
{
	std::deque<Observation> observations;
	
	// create new client
	MyLBRClient client{  };

	// create new udp connection	
	KUKA::FRI::UdpConnection connection;

	// pass connection and client to a new FRI client application
	KUKA::FRI::ClientApplication app(connection, client);
   
	// Connect client application to KUKA Sunrise controller.
	// Parameter NULL means: repeat to the address, which sends the data
	app.connect(portID, NULL);

	// repeatedly call the step routine to receive and process FRI packets
	bool success = true;
	while (success){
		success = app.step();
	}

	// disconnect from controller
	app.disconnect();

	std::cout << "terminated the program" << std::endl;
	return 0;
}

#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>

int main() {
	try {
		curan::utilities::cout << "started running";
		unsigned short port = 50000;
		asio::io_context io_context;
		curan::communicationinterface_igtl igtlink_interface;
		curan::communication::Client::Info construction{ io_context,igtlink_interface };
		asio::ip::tcp::resolver resolver(io_context);
		auto endpoints = resolver.resolve("localhost", std::to_string(port));
		construction.endpoints = endpoints;
	    curan::communicatio::Client client{ construction };
		auto connectionstatus = client.connect(bar);
		auto val = io_context.run();
		curan::utilities::cout << "stopped running";
		laucher.join();
	}
	catch (std::exception& e) {
		curan::utilities::cout << "CLient exception was thrown"+std::string(e.what());
		return 1;
	}
	return 0;
}
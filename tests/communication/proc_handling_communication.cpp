#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoProcHandler.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>
#include <cmath>

int foo(asio::io_context& cxt, unsigned short port) {
	using namespace curan::communication;
	try {
		interface_prochandler igtlink_interface;
		Server::Info construction{ cxt,igtlink_interface ,port };

        
		auto server = Server::make(construction);

        size_t counter = 0;
		while (counter< 1000) {
			auto start = std::chrono::high_resolution_clock::now();

            auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
            val->serialize();
			auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(),val);
			server->write(to_send);
			
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(10) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
			++counter;
		}
		curan::utilities::cout << "Stopping context";
		cxt.stop();
	}
	catch (std::exception& e) {
		curan::utilities::cout << "CLient exception was thrown" + std::string(e.what());
        return 1;
	}
    return 0;
}


void bar(const size_t& protocol_defined_val,const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val) {
	curan::utilities::cout << "received message";
    switch(val->signal_to_process){
        case curan::communication::ProcessHandler::Signals::HEART_BEAT: 
        case curan::communication::ProcessHandler::Signals::ACKNOLEGE_HEAR_BEAT:
        case curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY:
            std::cout << " valid \n";
        break;
        default: 
            std::cout << " invalid \n";
        break;
    }
}

int main() {
	try {
		curan::utilities::cout << "started running";
		using namespace curan::communication;
		unsigned short port = 50000;
		asio::io_context io_context;
		auto lauchfunctor = [&io_context, port]() {
			foo(io_context, port);
            return;
		};
		std::thread laucher(lauchfunctor);
		interface_prochandler igtlink_interface;
		Client::Info construction{ io_context,igtlink_interface };
		asio::ip::tcp::resolver resolver(io_context);
		auto endpoints = resolver.resolve("localhost", std::to_string(port));
		construction.endpoints = endpoints;
		auto client = Client::make(construction);
		client->connect(bar);
		io_context.run();
		curan::utilities::cout << "stopped running";
		laucher.join();
	}
	catch (std::exception& e) {
		curan::utilities::cout << "CLient exception was thrown"+std::string(e.what());
		return 1;
	}
	return 0;
}
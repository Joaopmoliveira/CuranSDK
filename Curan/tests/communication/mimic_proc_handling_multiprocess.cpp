#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoProcHandler.h"
#include "utils/Flag.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>
#include <cmath>

constexpr size_t time_taken = 200;

void child_callback(const size_t& protocol_defined_val,const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val,curan::utilities::Flag& flag1) {
	if(er){
		std::cout << er.message();
		return;
	}
	std::cout << "child received message\n";
    switch(val->signal_to_process){
        case curan::communication::ProcessHandler::Signals::HEART_BEAT: 
			std::cout << "child heart_beat\n";
		break;
        case curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY:
            std::cout << "child valid \n";
			flag1.set(true);
        break;
        default: 
            std::cout << "child invalid \n";
        break;
    }
}

int child_proc(asio::io_context& context, unsigned short port){
	using namespace curan::communication;
	curan::utilities::Flag flag1;
	flag1.set(false);
	interface_prochandler igtlink_interface;
	Client::Info construction{ context,igtlink_interface };
	asio::ip::tcp::resolver resolver(context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));
	construction.endpoints = endpoints;
	Client client{ construction };
	auto connectionstatus = client.connect(
		[&](const size_t& prot,
			const std::error_code& er, 
			std::shared_ptr<curan::communication::ProcessHandler> val){
		child_callback(prot,er,val,flag1);
	});
	if(!connectionstatus)
		throw std::runtime_error("failure to connect");
	while(!flag1.value()){
		auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
        val->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(),val);
		std::this_thread::sleep_for(std::chrono::milliseconds(time_taken));
		client.write(to_send);
		std::cout << "sending client data\n";
	}
	std::cout << "====== stopping client ==============\n";
}

void parent_callback(const size_t& protocol_defined_val,const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val,curan::communication::Server& server) {
	if(er){
		std::cout << er.message();
		return;
	}
	curan::utilities::cout << "server received message\n";
    switch(val->signal_to_process){
        case curan::communication::ProcessHandler::Signals::HEART_BEAT: 
			std::cout << "server heart_beat\n";
		break;
        case curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY:
            std::cout << "server valid \n";
        break;
        default: 
            std::cout << "server invalid \n";
        break;
    }
}

int parent_proc(asio::io_context& context,curan::utilities::Flag& flag){
	curan::utilities::cout << "started running server\n";
	using namespace curan::communication;
	unsigned short port = 50000;

	interface_prochandler igtlink_interface;
	Server::Info construction{ context,igtlink_interface ,port };
        
	Server server{ construction };

	auto value = server.connect(
			[&](const size_t& protocol_defined_val,
				const std::error_code& er, 
				std::shared_ptr<curan::communication::ProcessHandler> val){
		parent_callback(protocol_defined_val,er,val,server);
	});
	if(!value)
		throw std::runtime_error("failure to connect");

	auto lauchfunctor = [&context, port]() {
		child_proc(context, port);
        return;
	};

	std::thread laucher(lauchfunctor);

	flag.set(true);

	for(size_t counter = 0 ; counter < 10 ; ++counter){
		auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
        val->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(),val);
		std::this_thread::sleep_for(std::chrono::milliseconds(time_taken));
		std::cout << "sending server data\n";
		server.write(to_send);
	}

	auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY);

	laucher.join();
	std::cout << "====== stopping server ============\n";
}

int main() {
	try {
		curan::utilities::cout << "-> started running\n";
		using namespace curan::communication;
		curan::utilities::Flag flag;
		flag.set(false);
		asio::io_context io_context;
		std::thread thr{[&](){parent_proc(io_context,flag);}};
		flag.wait();
		curan::utilities::cout << "-> running context\n";
		io_context.run();
		curan::utilities::cout << "-> not running context\n";
		thr.join();
	}
	catch (std::exception& e) {
		curan::utilities::cout << "-> CLient exception was thrown"+std::string(e.what());
		return 1;
	}
	return 0;
}
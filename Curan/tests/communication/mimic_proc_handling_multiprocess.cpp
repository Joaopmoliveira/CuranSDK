#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoProcHandler.h"
#include "utils/Flag.h"
#include "utils/TheadPool.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>
#include <cmath>

constexpr size_t time_taken = 200;

class ProcessLaucher{
	asio::io_context hidden_context;
	asio::high_resolution_timer timer;
	std::chrono::nanoseconds duration;
	size_t number_of_violations;
	std::atomic<bool> was_violated;
	const size_t max_num_violations;
	std::unique_ptr<curan::communication::Server> server;

	template <class _Rep, class _Period , size_t max_viols>
	ProcessLaucher(asio::io_context client_ctx,std::shared_ptr<curan::utilities::ThreadPool> pool,const std::chrono::duration<_Rep, _Period>& deadline,unsigned short port = 50000) : 
			hidden_context{}, 
			timer{hidden_context},
			number_of_violations{0},
			was_violated = true,
			max_num_violations{max_viols}
	{
		using namespace curan::communication;
		interface_prochandler igtlink_interface;
		Server::Info construction{ client_ctx,igtlink_interface ,port };
		server = std::make_unique<Server>(construction);

		auto value = server->connect([this](const size_t& protocol_defined_val,
											const std::error_code& er, 
											std::shared_ptr<curan::communication::ProcessHandler> val){
			message_callback(protocol_defined_val,er,val);
		});

		timer.expires_from_now(duration);
		timer.async_wait([this](asio::error_code ec) {
			number_of_violations = was_violated ? number_of_violations+1 : 0;
			if(number_of_violations>max_num_violations){
				hidden_context.stop();
				std::raise(SIGINT);
			}
			auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
        	val->serialize();
			auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(),val);
			server->write(to_send);
    	}); 

		pool->submit(curan::utilities::Job{"run hidden proc laucher handler",[](){
			hidden_context.run();
		}})
  	}

	void message_callback(const size_t& protocol_defined_val,const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val){

	}
};

class ChildProcess{
	asio::io_context hidden_context;
	asio::high_resolution_timer timer;
	std::chrono::nanoseconds duration;
	size_t number_of_violations;
	std::atomic<bool> was_violated;
	const size_t max_num_violations;
	std::unique_ptr<curan::communication::Client> client;

	template <class _Rep, class _Period, size_t max_viols>
	ChildProcess(asio::io_context client_ctx,const std::chrono::duration<_Rep, _Period>& deadline,std::shared_ptr<curan::utilities::ThreadPool> pool,unsigned short port = 50000) : 
			hidden_context{}, 
			timer{hidden_context},
			number_of_violations{0},
			was_violated = true,
			max_num_violations{max_viols}
	{
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);

		curan::communication::interface_prochandler igtlink_interface;
		curan::communication::Client::Info construction{ client_ctx,igtlink_interface };
		asio::ip::tcp::resolver resolver(client_ctx);
		auto endpoints = resolver.resolve("localhost", std::to_string(port));
		construction.endpoints = endpoints;
		client = std::make_unique<curan::communication::Client>(construction);
		client->connect([this](const size_t& prot,
							const std::error_code& er, 
							std::shared_ptr<curan::communication::ProcessHandler> val){
			message_callback(prot,er,val);
		});

		timer.expires_from_now(duration);
		timer.async_wait([this](asio::error_code ec) {
			number_of_violations = was_violated ? number_of_violations+1 : 0;
			if(number_of_violations>max_num_violations){
				hidden_context.stop();
				std::raise(SIGINT);
			}
			auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
        	val->serialize();
			auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(),val);
			client->write(to_send);
    	}); 

		pool->submit(curan::utilities::Job{"run hidden proc laucher handler",[](){
			hidden_context.run();
		}})
	}

	void message_callback(const size_t& protocol_defined_val,const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val){
		if(er){
			std::cout << er.message();
			hidden_context.stop();
			std::raise(SIGINT);
			return;
		}
		switch(val->signal_to_process){
        case curan::communication::ProcessHandler::Signals::HEART_BEAT: 
			was_violated = false;
		break;
        case curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY:
        default: 
			hidden_context.stop();
			std::raise(SIGINT);
        break;
    	}
	}
};

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
            std::cout << "!!!!!!! child shutdown !!!! \n";
			flag1.set(true);
        break;
        default: 
            std::cout << "!!!!!!! child unknown !!!! \n";
			flag1.set(true);
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
	client.connect(
		[&](const size_t& prot,
			const std::error_code& er, 
			std::shared_ptr<curan::communication::ProcessHandler> val){
		child_callback(prot,er,val,flag1);
	});
	while(!flag1.value()){
		auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
        val->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(),val);
		std::this_thread::sleep_for(std::chrono::milliseconds(time_taken));
		client.write(to_send);
		std::cout << "sending client data\n";
	}
	std::cout << "====== stopping client ==============\n";
	return 1;
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

	server.connect(
			[&](const size_t& protocol_defined_val,
				const std::error_code& er, 
				std::shared_ptr<curan::communication::ProcessHandler> val){
		parent_callback(protocol_defined_val,er,val,server);
	});

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
		server.write(to_send);
	}

	auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY);
	val->serialize();
	auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(),val);

	server.write(to_send);

	laucher.join();
	std::cout << "====== stopping server ============\n";
	return 1;
	
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
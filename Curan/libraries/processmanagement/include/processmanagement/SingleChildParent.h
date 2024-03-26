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
#include <csignal>
#include <system_error>

#ifdef CURAN_WINDOWS
#include <tchar.h>
#include <windows.h>
#elif CURAN_LINUX
#define _OPEN_SYS
#include <sys/wait.h>
#include <unistd.h>
#include <sys/types.h>
#endif

#include <stdio.h>
#include <filesystem>

namespace curan{
namespace process{

struct PlatformAgnosticCmdArgs{
	
#ifdef CURAN_WINDOWS
		std::string cmd;

		PlatformAgnosticCmdArgs(const std::string& in_cmd) : cmd{in_cmd} {}

#elif CURAN_LINUX
		std::vector<std::string> cmd;

		PlatformAgnosticCmdArgs(const std::vector<std::string>& in_cmd) : cmd{in_cmd} {}
#endif 
};

std::ostream& operator<< (std::ostream& o, PlatformAgnosticCmdArgs args){
#ifdef CURAN_WINDOWS
	o << args.cmd;
#elif CURAN_LINUX	
	std::vector<std::string> cmd;
	for(const auto& val : args.cmd)
		o << val << " ";
#endif 	
	return o;
}

class ProcessHandles {



public:
	ProcessHandles() {
#ifdef CURAN_WINDOWS
	ZeroMemory(&pi, sizeof(pi));
#elif CURAN_LINUX
	pi = 0;
#endif	
	}

	operator bool() const;
	template<class _Rep, class _Period>
	void close(const std::chrono::duration<_Rep, _Period>& deadline);
	bool open(PlatformAgnosticCmdArgs& s);


#ifdef CURAN_WINDOWS
	PROCESS_INFORMATION pi;
#elif CURAN_LINUX
	pid_t pi = 0;
#endif

};

template<typename... Args>
PlatformAgnosticCmdArgs create_command(unsigned short port, Args ... arg) {
	constexpr size_t size = sizeof ...(Args);
	const char* loc[size] = { arg... };
	std::string command;

	//first we check if the executable exists in the current computer
	std::string executable_directory_sanity_check{loc[0]};
	std::filesystem::path path{ executable_directory_sanity_check };

	if (!std::filesystem::exists(path))
		throw std::runtime_error("the specified file does not exist");

#ifdef CURAN_WINDOWS
	std::string cmd;

	// now depending if we are on linux or windows we need distinct behavior	
	for (const auto& val : loc)
		cmd += std::string(val) + " ";

	cmd += std::to_string(port);	

	PlatformAgnosticCmdArgs args{cmd};

#elif CURAN_LINUX
	std::vector<std::string> cmd;

	// now depending if we are on linux or windows we need distinct behavior	
	for (const auto& val : loc)
		cmd.push_back(std::string(val));;

	cmd.push_back(std::to_string(port));

	PlatformAgnosticCmdArgs args{cmd};

#endif 
	return args;
}

class ProcessLaucher {

	asio::io_context& hidden_context;
	asio::high_resolution_timer connection_timer;
	asio::high_resolution_timer closing_process_timer;
	std::chrono::nanoseconds duration;
	size_t number_of_violations;
	bool was_violated;
	const size_t max_num_violations;
	std::shared_ptr<curan::communication::Server> server;
	bool connection_established = false;
	unsigned short port;

public:

	ProcessHandles handles;

template <class _Rep, class _Period>
	ProcessLaucher(asio::io_context& client_ctx, const std::chrono::duration<_Rep, _Period>& deadline, size_t max_viols, unsigned short in_port = 50000) :
		hidden_context{ client_ctx },
		connection_timer{ client_ctx },
		closing_process_timer{ client_ctx },
		number_of_violations{ 0 },
		was_violated{ true },
		max_num_violations{ max_viols },
		port{ in_port },
		handles{}
	{
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);
		using namespace curan::communication;
		interface_prochandler igtlink_interface;
		Server::Info construction{ client_ctx,igtlink_interface ,port };
		server = Server::make(construction,
			[this](std::error_code er) {
				if (!er && !connection_established) {
					connection_established = true;
					return true;
				} 
				if (connection_established)
					return false;
				server->cancel();
				return false;
			}
		);

		server->connect([this](const size_t& protocol_defined_val,
			const std::error_code& er,
			std::shared_ptr<curan::communication::ProcessHandler> val) {
				message_callback(protocol_defined_val, er, val);
			});

		connection_timer.expires_from_now(duration);
		connection_timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	void timer_callback(asio::error_code ec);

	~ProcessLaucher();

	void sync_internal_terminate_pending_process_and_connections();

	void message_callback(const size_t& protocol_defined_val, const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val);

	/*
	This function takes the arguments, appends them with a space and then adds at the end the port of the current server
	*/
	template<typename... Args>
	bool lauch_process(Args ... arg) {
		if (handles) {
			return false;
		}
		auto command = create_command(port,arg...);
		return handles.open(command);
	}

	/*
	This function takes the arguments, appends them with a space and then adds at the end the port of the current server
	*/
	template<typename... Args>
	void async_lauch_process(std::function<void(bool)> async_handler, Args ... arg) {
		if (handles) { // we cannot lauch an asycn proces without closing the previous handles
			async_handler(false);
			return;
		}
		auto command = create_command(port, arg...);
		hidden_context.post([this, command, async_handler]() mutable {
				async_handler(handles.open(command));
			}
		);

	}

	/*
	This is a blocking call which waits until the other process is stopped. 
	don't use it inside the asio executor or you block all asyncronous operations
	*/
	template <class _Rep, class _Period>
	void terminate(const std::chrono::duration<_Rep, _Period>& deadline) {
		if (!handles) //if the handles are already closed then we don't need to close anything
			return;
		
		auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::SHUTDOWN_SAFELY);
		val->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
		server->write(to_send);

		handles.close(deadline);
	}

	template <class _Rep, class _Period>
	void async_terminate(const std::chrono::duration<_Rep, _Period>& deadline,std::function<void(void)> termination_handler) {
		if (!handles)
			return;

		auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::SHUTDOWN_SAFELY);
		val->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
		server->write(to_send);

		closing_process_timer.expires_from_now(deadline);
		closing_process_timer.async_wait(
			[this,termination_handler](asio::error_code ec) {
				handles.close(std::chrono::milliseconds(0)); // the method attempts to check if the other process is close and then is kills the other process automatically
				termination_handler();
			}
		);

	}
};

}
}
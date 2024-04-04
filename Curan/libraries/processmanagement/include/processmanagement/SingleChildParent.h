#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoProcHandler.h"

#include <thread>
#include <csignal>
#include <chrono>
#include <atomic>
#include <cmath>
#include <csignal>
#include <system_error>

#include <iostream>
#include <thread>

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

std::ostream& operator<< (std::ostream& o, PlatformAgnosticCmdArgs args);

class ProcessHandles {
#ifdef CURAN_WINDOWS
	PROCESS_INFORMATION pi;
#elif CURAN_LINUX
	pid_t pi = 0;
#endif
public:
	ProcessHandles();

	~ProcessHandles();

	operator bool() const;

	template<class _Rep, class _Period>
	void close(const std::chrono::duration<_Rep, _Period>& deadline) {
		if (!(*this)) {
			return;
		}
		
#ifdef CURAN_WINDOWS
		std::chrono::milliseconds transformed_deadline = std::chrono::duration_cast<std::chrono::milliseconds>(deadline);
		auto return_value = WaitForSingleObject(pi.hProcess, transformed_deadline.count());
		switch (return_value) {
		case WAIT_ABANDONED: //this should never happen? it happears to be related with mutex handles
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		case WAIT_OBJECT_0: // this means that the operation was successeful
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		case WAIT_TIMEOUT: // the wait operation timed out thus unsuccesefull
			TerminateProcess(pi.hProcess, 3);
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		case WAIT_FAILED: // the wait failed for obscure reasons
			TerminateProcess(pi.hProcess, 3);
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		default:
			TerminateProcess(pi.hProcess, 3);
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		}
		// if the waiting operation does not terminate in the alloted time we force the process to terminate manually

#elif CURAN_LINUX
		int status = -10;
		int vals = -10;
		// the behavior is different if we need to wait for a larger amount of time than 0 or if it is zero
		std::chrono::nanoseconds transformed_deadline = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);
		if(transformed_deadline.count() == 0){
			vals = waitpid(pi, &status, WNOHANG);
        	if (vals == -1) {
            	// failure to wait, (basically the wait itself could not run to check on the child process)
				// I think I should still try to kill the proccess in this case
				kill(pi,SIGINT);
				pi = 0;
       	 	}

        	if (vals>0) { // this is true, it means that the wait was suceesefull and we can anlyse the status flag to see the status of the other process
            	if (WIFEXITED(status)) { // if this is true the other process is closed, so we don't need to do anything
                	pi = 0;
            	} else { // if this is triggered then the process was terminated for some other reason, still don't know if I should kill it in this situation or not
					pi = 0;
				}
        	} else if(pi){ // the wait did not finish, meaning that the process is still active,
				kill(pi,SIGINT);
				pi = 0;
			}

		} else {
			auto start_wait_time = std::chrono::high_resolution_clock::now();
			auto current_time = std::chrono::high_resolution_clock::now();
			auto copy_of_pid = pi;
    		do {
				vals = waitpid(copy_of_pid, &status, WNOHANG);
        		if (vals == -1) {
            		// failure to wait, (basically the wait itself could not run to check on the child process)
					// I think I should still try to kill the proccess in this case 
       	 		}

        		if (vals) { // this is true, it means that the wait was suceesefull and we can anlyse the status flag to see the status of the other process
            		if (WIFEXITED(status)) { // if this is true the other process is closed, so we don't need to do anything
						copy_of_pid = 0;
						break;
            		} else { // if this is triggered then the process was terminated for some other reason, still don't know if I should kill it in this situation or not
						copy_of_pid = 0;
						break;
					}
        		} else { // the wait did not finish, meaning that the process is still active,
				}
				std::this_thread::sleep_for(transformed_deadline/10);
				current_time = std::chrono::high_resolution_clock::now();
				
    		} while (std::chrono::duration_cast<std::chrono::nanoseconds>(current_time-start_wait_time)<transformed_deadline);
			if(copy_of_pid){
				kill(pi,SIGINT);
				pi = 0;
			}
		}


#endif // CURAN_WINDOWS

	}

	bool open(PlatformAgnosticCmdArgs& s);

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

	enum Failure{
		SERVER_TIMEOUT_REACHED,
		CLIENT_FAILURE_MAX_FAILED_HEARBEATS
	};

	asio::io_context& hidden_context;
	asio::high_resolution_timer connection_timer;
	asio::high_resolution_timer server_connection_timer;
	asio::high_resolution_timer closing_process_timer;
	std::chrono::nanoseconds duration;
	std::chrono::nanoseconds after_lauch_duration;
	size_t number_of_violations;
	bool was_violated;
	const size_t max_num_violations;
	std::shared_ptr<curan::communication::Server> server;
	bool connection_established = false;
	unsigned short port;
	std::function<void(bool)> connection_callback;

public:

	ProcessHandles handles;

	template <class _Rep, class _Period>
	ProcessLaucher(asio::io_context& client_ctx, 
					const std::chrono::duration<_Rep, _Period>& deadline, 
					const std::chrono::duration<_Rep, _Period>& server_connection_deadline, 
					size_t max_viols, unsigned short in_port = 50000) :
		hidden_context{ client_ctx },
		connection_timer{ client_ctx },
		server_connection_timer{ client_ctx },
		closing_process_timer{ client_ctx },
		number_of_violations{ 0 },
		was_violated{ true },
		max_num_violations{ max_viols },
		port{ in_port },
		handles{}
	{
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);
		after_lauch_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(server_connection_deadline);
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

	template <class _Rep, class _Period>
	ProcessLaucher(asio::io_context& client_ctx, 
					const std::chrono::duration<_Rep, _Period>& deadline, 
					const std::chrono::duration<_Rep, _Period>& server_connection_deadline, 
					size_t max_viols, 
					std::function<void(bool)> in_connection_callback, 
					unsigned short in_port = 50000) :
		hidden_context{ client_ctx },
		connection_timer{ client_ctx },
		server_connection_timer{ client_ctx },
		closing_process_timer{ client_ctx },
		number_of_violations{ 0 },
		was_violated{ true },
		max_num_violations{ max_viols },
		port{ in_port },
		handles{},
		connection_callback{in_connection_callback}
	{
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);
		after_lauch_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(server_connection_deadline);
		using namespace curan::communication;
		interface_prochandler igtlink_interface;
		Server::Info construction{ client_ctx,igtlink_interface ,port };
		server = Server::make(construction,
			[this](std::error_code er) {
				if (!er && !connection_established) {
					connection_established = true;
					return true;
				} 
				if (connection_established){
					return false;
				}
				server->close();
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

	void sync_internal_terminate_pending_process_and_connections(const Failure& failure_reason );

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
		assert(!connection_established);
		if(after_lauch_duration.count()>0){
			server_connection_timer.expires_from_now(after_lauch_duration);
			server_connection_timer.async_wait([this](asio::error_code ec) {
				if (!connection_established){
					sync_internal_terminate_pending_process_and_connections(SERVER_TIMEOUT_REACHED);
				}
			});		
		}
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
		assert(!connection_established);
		if(after_lauch_duration.count()>0){
			server_connection_timer.expires_from_now(after_lauch_duration);
			server_connection_timer.async_wait([this](asio::error_code ec) {
				if (!connection_established){
					sync_internal_terminate_pending_process_and_connections(SERVER_TIMEOUT_REACHED);
				}
			});		
		}
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
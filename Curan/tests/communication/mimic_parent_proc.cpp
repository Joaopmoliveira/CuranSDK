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
#include <tchar.h>
#include <windows.h>
#include <stdio.h>

asio::io_context* ptr_ctx = nullptr;

void signal_handler(int signal)
{
	if (ptr_ctx) ptr_ctx->stop();
}


class ProcessHandles {
#ifdef CURAN_WINDOWS
	PROCESS_INFORMATION pi;
#elif CURAN_LINUX

#endif

	bool operator()() {
#ifdef CURAN_WINDOWS
		PROCESS_INFORMATION local;
		ZeroMemory(&local, sizeof(local));
		// memcmp(&local, &pi, sizeof(local)); returns 0 if all bytes are equal, i.e., if the memory of pi is zeroed out. If it is zeroed then it mean that the 
		// handles are closed, if they are different then returns number different from zero
		return memcmp(&local, &pi, sizeof(local)); // compare if the handles are nullified

#elif CURAN_LINUX

#endif 
	}

	template<class _Rep, class _Period>
	bool close(const std::chrono::duration<_Rep, _Period>& deadline) {
		auto transformed_deadline = std::chrono::duration_cast<std::chrono::milliseconds>(deadline);
#ifdef CURAN_WINDOWS
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
			std::cout << "wait operation did not return in time\n";
			TerminateProcess(pi.hProcess, 3);
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		case WAIT_FAILED: // the wait failed for obscure reasons
			std::cout << "wait operation failed\n";
			TerminateProcess(pi.hProcess, 3);
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		default:
			std::cout << "unknown error\n";
			TerminateProcess(pi.hProcess, 3);
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		}
		// if the waiting operation does not terminate in the alloted time we force the process to terminate manually

#elif CURAN_LINUX

#endif // CURAN_WINDOWS
	}

	bool open(std::string s) {
#ifdef CURAN_WINDOWS
		STARTUPINFO si;

		ZeroMemory(&si, sizeof(si));
		si.cb = sizeof(si);
		ZeroMemory(&pi, sizeof(pi));
		TCHAR* val = s.data();
		//const TCHAR* data = static_cast<const TCHAR*>(command.data());
		;
		// Start the child process. 
		if (!CreateProcess(NULL,   // No module name (use command line)
			val,        // Command line
			NULL,           // Process handle not inheritable
			NULL,           // Thread handle not inheritable
			FALSE,          // Set handle inheritance to FALSE
			0,              // No creation flags
			NULL,           // Use parent's environment block
			NULL,           // Use parent's starting directory 
			&si,            // Pointer to STARTUPINFO structure
			&pi)           // Pointer to PROCESS_INFORMATION structure
			) {
			std::cout << "launched process\n";
		}
		else {
			std::cout << "failed to launch process\n";
			return false;
		}

#elif CURAN_LINUX
		return false;
#endif 
	}

};

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
	ProcessHandles handles;

public:

	template <class _Rep, class _Period>
	ProcessLaucher(asio::io_context& client_ctx, const std::chrono::duration<_Rep, _Period>& deadline, size_t max_viols, unsigned short in_port = 50000) :
		hidden_context{ client_ctx },
		connection_timer{ client_ctx },
		number_of_violations{ 0 },
		was_violated{ true },
		max_num_violations{ max_viols },
		port{ in_port }
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

	void timer_callback(asio::error_code ec) {
		if (ec)
			return;
		if (connection_established) {
			number_of_violations = was_violated ? number_of_violations + 1 : 0;
			if (number_of_violations > max_num_violations) {
				server->cancel();
				connection_timer.cancel(std::make_error_code(std::errc::timed_out));
				return;
			}
			auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
			val->serialize();
			auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);

			server->write(to_send);
		}
		connection_timer.expires_from_now(duration);
		connection_timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	~ProcessLaucher() {
		std::cout << "destroying parent proc" << std::endl;
		connection_timer.cancel(std::make_error_code(std::errc::timed_out));
		closing_process_timer.cancel(std::make_error_code(std::errc::timed_out));
		sync_internal_terminate_pending_process_and_connections();
	}

	void sync_internal_terminate_pending_process_and_connections() {
		connection_established = false;
		server->cancel();
	}

	void message_callback(const size_t& protocol_defined_val, const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val) {
		if (er) {
			sync_internal_terminate_pending_process_and_connections();
			return;
		}
		switch (val->signal_to_process) {
		case curan::communication::ProcessHandler::Signals::HEART_BEAT:
			was_violated = false;
			number_of_violations = 0;
			std::cout << "!";
			break;
		case curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY:
		default:
			number_of_violations = max_num_violations;
			break;
		}
	}

	/*
	This function takes the arguments, appends them with a space and then adds at the end the port of the current server
	*/
	template<typename... Args>
	bool lauch_process(Args ... arg) {
		if (handles)
			return false;
		constexpr size_t size = sizeof ...(Args);
		const char* loc[size] = { arg... };
		std::string command;
		for (const auto& val : loc)
			command += std::string(val) + " ";

		command += std::to_string(port);

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
		constexpr size_t size = sizeof ...(Args);
		const char* loc[size] = { arg... };

		std::string command;
		for (const auto& val : loc)
			command += std::string(val) + " ";

		command += std::to_string(port);

		hidden_context.post([this, command](asio::error_code ec) {
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
	void async_terminate(const std::chrono::duration<_Rep, _Period>& deadline) {
		if (!handles)
			return;

		auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::SHUTDOWN_SAFELY);
		val->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
		server->write(to_send);

		auto transformed_deadline = std::chrono::duration_cast<std::chrono::milliseconds>(deadline);
		closing_process_timer.expires_from_now(deadline);
		closing_process_timer.async_wait(
			[this](asio::error_code ec) {
				handles.close(std::chrono::milliseconds(0)); // the method attempts to check if the other process is close and then is kills the other process automatically
			}
		);

	}

};

int main() {
	try {
		std::cout << "started running\n";
		using namespace curan::communication;
		std::signal(SIGINT, signal_handler);
		asio::io_context io_context;
		ptr_ctx = &io_context;
		auto parent = std::make_unique<ProcessLaucher>(io_context, std::chrono::milliseconds(100), 10);
		parent->async_lauch_process([](bool sucess) {
			},"notepad");
		curan::utilities::cout << "running context\n";
		io_context.run();
		curan::utilities::cout << "not running context\n";
	}
	catch (std::exception& e) {
		std::cout << "Client exception was thrown" + std::string(e.what()) << std::endl;
		return 1;
	}
	catch (...) {
		std::cout << "Unknown exception" << std::endl;
		return 1;
	}
	return 0;
}
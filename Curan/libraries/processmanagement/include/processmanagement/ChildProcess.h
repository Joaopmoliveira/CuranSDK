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

namespace curan{
namespace process{

class ChildProcess {
	asio::io_context& hidden_context;
	asio::high_resolution_timer timer;
	std::chrono::nanoseconds duration;
	size_t number_of_violations;
	bool was_violated;
	const size_t max_num_violations;
	std::shared_ptr<curan::communication::Client> client;
	bool first_connection_established = false;
	std::function<void(bool)> connection_callback;
	
public:
	enum Failure{
		CLIENT_TIMEOUT_REACHED,
		SERVER_FAILURE_MAX_FAILED_HEARBEATS
	};

	template <class _Rep, class _Period>
	ChildProcess(asio::io_context& client_ctx, const std::chrono::duration<_Rep, _Period>& deadline, size_t max_violations, unsigned short port = 50000) :
		hidden_context{ client_ctx },
		timer{ hidden_context },
		number_of_violations{ 0 },
		was_violated{ true },
		max_num_violations{ max_violations }
	{
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);
		curan::communication::interface_prochandler igtlink_interface;
		curan::communication::Client::Info construction{ hidden_context,igtlink_interface };
		asio::ip::tcp::resolver resolver(hidden_context);
		auto endpoints = resolver.resolve("localhost", std::to_string(port));
		construction.endpoints = endpoints;
		client = curan::communication::Client::make(construction,
			std::chrono::seconds(10),
			[this](std::error_code er) {
				if (!er) {
					first_connection_established = true;
				}
				else {
					sync_internal_terminate_pending_process_and_connections(CLIENT_TIMEOUT_REACHED);					
				}
			}
		);
		client->connect([this](const size_t& prot,
			const std::error_code& er,
			std::shared_ptr<curan::communication::ProcessHandler> val) {
				message_callback(prot, er, val);
			});

		timer.expires_from_now(duration);
		timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	template <class _Rep, class _Period>
	ChildProcess(asio::io_context& client_ctx, const std::chrono::duration<_Rep, _Period>& deadline, size_t max_violations, std::function<void(bool)> in_connection_callback ,unsigned short port = 50000) :
		hidden_context{ client_ctx },
		timer{ hidden_context },
		number_of_violations{ 0 },
		was_violated{ true },
		max_num_violations{ max_violations },
		connection_callback{in_connection_callback}
	{
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);
		curan::communication::interface_prochandler igtlink_interface;
		curan::communication::Client::Info construction{ hidden_context,igtlink_interface };
		asio::ip::tcp::resolver resolver(hidden_context);
		auto endpoints = resolver.resolve("localhost", std::to_string(port));
		construction.endpoints = endpoints;
		client = curan::communication::Client::make(construction,
			std::chrono::seconds(10),
			[this](std::error_code er) {
				if (!er) {
					first_connection_established = true;
				}
				else {
					sync_internal_terminate_pending_process_and_connections(CLIENT_TIMEOUT_REACHED);
				}
			}
		);
		client->connect([this](const size_t& prot,
			const std::error_code& er,
			std::shared_ptr<curan::communication::ProcessHandler> val) {
				message_callback(prot, er, val);
			});

		timer.expires_from_now(duration);
		timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	void timer_callback(asio::error_code ec);

	void message_callback(const size_t& protocol_defined_val, const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val);

	void sync_internal_terminate_pending_process_and_connections(const Failure&);

	~ChildProcess();
};

}
}

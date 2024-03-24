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

asio::io_context* ptr_ctx = nullptr;

void signal_handler(int signal)
{
	if (ptr_ctx) ptr_ctx->stop();
}

class ProcessLaucher {

	asio::io_context& hidden_context;
	asio::high_resolution_timer timer;
	std::chrono::nanoseconds duration;
	size_t number_of_violations;
	bool was_violated;
	const size_t max_num_violations;
	std::shared_ptr<curan::communication::Server> server;
	bool first_connection_established = false;
	size_t temp_counter = 0;

public:

	template <class _Rep, class _Period>
	ProcessLaucher(asio::io_context& client_ctx, const std::chrono::duration<_Rep, _Period>& deadline, size_t max_viols, unsigned short port = 50000) :
		hidden_context{ client_ctx },
		timer{ client_ctx },
		number_of_violations{ 0 },
		was_violated{ true },
		max_num_violations{ max_viols }
	{
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);
		using namespace curan::communication;
		interface_prochandler igtlink_interface;
		Server::Info construction{ client_ctx,igtlink_interface ,port };
		server = Server::make(construction,
			[this](std::error_code er) {
				if (!er) {
					first_connection_established = true;
					return;
				}
				server->cancel();
			}
		);

		server->connect([this](const size_t& protocol_defined_val,
			const std::error_code& er,
			std::shared_ptr<curan::communication::ProcessHandler> val) {
				message_callback(protocol_defined_val, er, val);
			});

		timer.expires_from_now(duration);
		timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	void timer_callback(asio::error_code ec) {
		++temp_counter;
		if (ec)
			return;
		if (temp_counter > 10) { return; }
		if (first_connection_established) number_of_violations = was_violated ? number_of_violations + 1 : 0;
		if (number_of_violations > max_num_violations) {
			server->cancel();
			timer.cancel(std::make_error_code(std::errc::timed_out));
			return;
		}
		auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
		val->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
		if (first_connection_established) {
			server->write(to_send);
		}
		timer.expires_from_now(duration);
		timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	~ProcessLaucher() {
		std::cout << "destroying parent proc" << std::endl;
		timer.cancel(std::make_error_code(std::errc::timed_out));
		server->cancel();
	}

	void message_callback(const size_t& protocol_defined_val, const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val) {
		if (er) {
			std::cout << er.message();
			timer.cancel(std::make_error_code(std::errc::timed_out));
			server->cancel();
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
};

int main() {
	try {
		std::cout << "started running\n";
		using namespace curan::communication;
		std::signal(SIGINT, signal_handler);
		asio::io_context io_context;
		ptr_ctx = &io_context;
		auto parent = std::make_unique<ProcessLaucher>(io_context, std::chrono::milliseconds(1000), 30);
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
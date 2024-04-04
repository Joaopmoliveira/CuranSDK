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

void signal_handler(int signal){	
	if (ptr_ctx) ptr_ctx->stop();
}

class ChildProcess {
	asio::io_context& hidden_context;
	asio::high_resolution_timer timer;
	std::chrono::nanoseconds duration;
	size_t number_of_violations;
	bool was_violated;
	const size_t max_num_violations;
	std::shared_ptr<curan::communication::Client> client;
	bool first_connection_established = false;
	size_t numbers_of_triggered_connections = 0;
public:
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
					client->get_socket().close();
					timer.cancel();
					
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

	void timer_callback(asio::error_code ec) {
		if (ec)
			return;
		if (first_connection_established) number_of_violations = was_violated ? number_of_violations + 1 : 0;
		
		was_violated = true;
		if (number_of_violations > max_num_violations) {
			timer.cancel();
			client->get_socket().close();
			return;
		}
		if(numbers_of_triggered_connections<10){
			auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
			val->serialize();
			auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
			if (first_connection_established) {
				client->write(to_send);
			}
		} else {

		}

		timer.expires_from_now(duration);
		timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	void message_callback(const size_t& protocol_defined_val, const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val) {
		if (er) {
			timer.cancel();
			client->get_socket().close();
			return;
		}
		switch (val->signal_to_process) {
		case curan::communication::ProcessHandler::Signals::HEART_BEAT:
			was_violated = false;
			number_of_violations = 0;
			break;
		case curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY:
		default:
			timer.cancel();
			client->get_socket().close();
			hidden_context.get_executor().on_work_finished();
			break;
		}
	}

	~ChildProcess() {
		timer.cancel();
		client->get_socket().close();
	}
};


int main() {
	try {
		using namespace curan::communication;
		std::signal(SIGINT, signal_handler);
		asio::io_context io_context;
		ptr_ctx = &io_context;
		std::unique_ptr<ChildProcess> child;
		
		child = std::make_unique<ChildProcess>(io_context, std::chrono::milliseconds(100), 10);
		io_context.run();
	}
	catch (std::exception& e) {
		return 1;
	}
	catch (...) {
		return 1;
	}
	return 0;
}
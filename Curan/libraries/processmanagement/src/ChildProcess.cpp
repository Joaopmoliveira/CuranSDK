#include "processmanagement/ChildProcess.h"

namespace curan{
namespace process{

	void ChildProcess::timer_callback(asio::error_code ec) {
		if (ec)
			return;
		if (first_connection_established) number_of_violations = was_violated ? number_of_violations + 1 : 0;
		
		was_violated = true;
		if (number_of_violations > max_num_violations) {
			timer.cancel();
			client->get_socket().close();
			hidden_context.get_executor().on_work_finished();
			return;
		}


		auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
		val->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
		if (first_connection_established) {
			client->write(to_send);
		}

		timer.expires_from_now(duration);
		timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	void ChildProcess::message_callback(const size_t& protocol_defined_val, const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val) {
		if (er) {
			timer.cancel();
			client->get_socket().close();
			hidden_context.get_executor().on_work_finished();
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

	ChildProcess::~ChildProcess() {
		timer.cancel();
		client->get_socket().close();
		hidden_context.get_executor().on_work_finished();
	}

}
}
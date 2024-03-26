void ProcessLaucher::timer_callback(asio::error_code ec) {
		if (ec)
			return;
		if (connection_established) {
			
			number_of_violations = was_violated ? number_of_violations + 1 : 0;
			was_violated = true;
			if (number_of_violations > max_num_violations) {
				auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::SHUTDOWN_SAFELY);
				val->serialize();
				auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
				server->write(to_send);
				async_terminate(std::chrono::seconds(3),[this](){ 
					connection_timer.cancel();
					closing_process_timer.cancel();
					sync_internal_terminate_pending_process_and_connections();
					hidden_context.get_executor().on_work_finished();
				});
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

	ProcessLaucher::~ProcessLaucher() {
		connection_timer.cancel();
		closing_process_timer.cancel();
		sync_internal_terminate_pending_process_and_connections();
	}

	void ProcessLaucher::sync_internal_terminate_pending_process_and_connections() {
		connection_established = false;
		server->cancel();
	}

	void ProcessLaucher::message_callback(const size_t& protocol_defined_val, const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val) {
		if (er) {
			sync_internal_terminate_pending_process_and_connections();
			return;
		}
		switch (val->signal_to_process) {
		case curan::communication::ProcessHandler::Signals::HEART_BEAT:
			was_violated = false;
			number_of_violations = 0;
			break;
		case curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY:
		default:
			break;
		}
	}
};
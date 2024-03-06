#include "LevelImplementation.h"
#include <vector>
#include <memory>

struct ProcessingMessage {
	curan::ui::ImageDisplay* processed_viwer = nullptr;
	curan::utilities::Flag connection_status;
	curan::ui::Button* button;
	asio::io_context io_context;
	std::atomic<bool> should_record = false;
	std::atomic<bool> show_circles = false;
	short port = 10000;

	ProcessingMessage(curan::ui::ImageDisplay* in_processed_viwer) : processed_viwer{ in_processed_viwer }
	{
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};
#include "MessageProcessing.h"


bool process_transform_message(ProcessingMessage* processor,igtl::MessageBase::Pointer val){
	igtl::TransformMessage::Pointer transform_message = igtl::TransformMessage::New();
	transform_message->Copy(val);
	int c = transform_message->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; //failed to unpack message, therefore returning without doing anything
	return true;
}

bool process_image_message(ProcessingMessage* processor,igtl::MessageBase::Pointer val){
	return true;
}

std::map<std::string,std::function<bool(ProcessingMessage*,igtl::MessageBase::Pointer val)>> openigtlink_callbacks{
	{"TRANSFORM",process_transform_message},
	{"IMAGE",process_image_message}
};

bool ProcessingMessage::process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
	//std::cout << "received message\n";
	assert(val.IsNotNull());
	if (er){
        return true;
    } 
    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(this,val);
    else
        std::cout << "No functionality for function received\n";
    return false;
}

void ProcessingMessage::communicate() {
	std::cout << "connecting to server" << std::endl;
	{
		std::lock_guard<std::mutex> g{mut};
		list_of_recorded_points.clear();
		
	}
	button->set_waiting_color(SK_ColorGREEN);
	io_context.reset();
	asio::ip::tcp::resolver resolver(io_context);
	auto client = curan::communication::Client<curan::communication::protocols::igtlink>::make(io_context,resolver.resolve("localhost", std::to_string(port)));
	connection_status.set(true);

	client->connect([this](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
		try{
			if (process_message(protocol_defined_val, er, val))
			{
				attempt_stop();
			}
		}catch(...){
			std::cout << "Exception was thrown\n";
		}
	});
	io_context.run();
	connection_status.set(false);
	button->set_waiting_color(SK_ColorRED);
	return;
}

void ProcessingMessage::attempt_stop() {
	io_context.stop();
}
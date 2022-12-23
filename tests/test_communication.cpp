#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#include <windows.h>

#include "communication/Functor.h"
#include "communication/ChannelManager.h"
#include "communication/ProcessorOIGTL.h"


void testsender() {

}

void testreceiver() {

}

struct CommunicationManager {

	void handle_incoming_message(igtl::MessageBase::Pointer p) {
	
	}
	void handle_connection_status(curan::communication::ProcessorOIGTL::Status stat) {
	
	}
};



void testFunctor(CommunicationManager* receiver_manager) {
	curan::communication::ChannelManager* manager = curan::communication::ChannelManager::Get();
	asio::io_context* context = nullptr;
	curan::communication::GetIOContext(&context);
	std::string chanel_name = "channel1";
	manager->start_channel(chanel_name, *context);
	std::shared_ptr<curan::communication::ProcessorOIGTL> openigtlink_channel;
	auto err = manager->get_channel(chanel_name, openigtlink_channel);
	openigtlink_channel->signal_connection_status(CommunicationManager::handle_connection_status, receiver_manager);
	openigtlink_channel->signal_received(CommunicationManager::handle_connection_status, receiver_manager);
};

int main() {
	CommunicationManager manager;
	return 0;
}
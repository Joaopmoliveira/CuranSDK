#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <thread>

void foo(asio::io_context& cxt, short port) {
	using namespace curan::communication;
	protocols::igtlink::interface_igtl igtlink_interface;
	Server::Info construction{ cxt,igtlink_interface ,port };
	Server server{ construction };
	for (size_t message_counter = 0; message_counter < 20; ++message_counter) {

	}
}

void bar(igtl::MessageBase::Pointer val) {

}

int main() {
	short port = 50000;
	asio::io_context io_context;
	auto lauchfunctor = [&io_context, port]() {
		foo(io_context, port);
	};
	std::jthread laucher(lauchfunctor);
	protocols::igtlink::interface_igtl igtlink_interface;
	Client::Info construction{ io_context,igtlink_interface };
	Client client{ construction };
	while (true) {

	}
	return 0;
}
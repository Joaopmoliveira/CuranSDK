#include "communication/ProtoFRI.h"
#include "communication/Client.h"
#include "utils/Logger.h"

namespace curan {
namespace communication {
namespace protocols {
namespace frimessage {
namespace implementation {

FRIClientConnection::FRIClientConnection(Client* supplied_owner) : owner{ supplied_owner } 
{
}

void read_header_first_time(FRIClientConnection val) {
	asio::async_read(val.owner->get_socket().get_underlying_socket(),
		asio::buffer(val.message.get_buffer(), val.message.get_header_size()), asio::transfer_all(),
		[val](std::error_code ec, std::size_t len) {
		if (!ec) {
			read_body(val, ec);
		} else {
			val.owner->transverse_callables<interface_fri>(0, ec, val.message_to_receive);
			val.owner->get_socket().get_underlying_socket().close();
		}				
		}
	);
}

void read_header(FRIClientConnection val, std::error_code ec) {
	//we have a message fully unpacked in memory that we must broadcast to all
	//listeners of the interface. We do this by calling the templated broadcast method
	size_t temp = 0;
    val.message.deserialize();
	val.owner->transverse_callables<interface_fri>(temp, ec, val.message);
	asio::async_read(val.owner->get_socket().get_underlying_socket(),
		asio::buffer(val.message.get_buffer(), val.message.get_header_size()), asio::transfer_all(),
		[val](std::error_code ec, std::size_t len){
		if (!ec) {
			read_body(val, ec);
		} else {
			val.owner->transverse_callables<interface_fri>(0, ec, val.message);
			val.owner->get_socket().get_underlying_socket().close();
		}}
	);
}

void read_body(FRIClientConnection val, std::error_code ec) {
	val.message.deserialize_header();

	asio::async_read(val.owner->get_socket().get_underlying_socket(),
		asio::buffer(val.message.get_body_buffer(), val.message.get_body_size()), asio::transfer_all(),
		[val](std::error_code ec, std::size_t len){
		if (!ec)
			read_header(val, ec);
		else {
			val.owner->transverse_callables<interface_fri>(0, ec, val.message);
			val.owner->get_socket().get_underlying_socket().close();
		}}
	);
}
}

void start(Client* client_pointer) {
	utilities::cout << "starting to:";
	implementation::FRIClientConnection val{ client_pointer };
	implementation::read_header_first_time(std::move(val));	
}


};
}
}
}
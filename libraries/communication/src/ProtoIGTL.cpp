#include "communication/ProtoIGTL.h"
#include "communication/Client.h"
#include "utils/Logger.h"

namespace curan {
namespace communication {
namespace protocols {
namespace igtlink {
namespace implementation {

IgtlinkClientConnection::IgtlinkClientConnection(Client* supplied_owner) : owner{ supplied_owner } 
{
}

void read_header_first_time(IgtlinkClientConnection val) {
	val.header_to_receive = igtl::MessageHeader::New();
	val.header_to_receive->InitPack();
	assert(val.header_to_receive->GetPackPointer()!=nullptr);
	assert(val.header_to_receive->GetPackSize()>=0);
	asio::async_read(val.owner->get_socket().get_underlying_socket(),
		asio::buffer(val.header_to_receive->GetPackPointer(), val.header_to_receive->GetPackSize()), asio::transfer_all(),
		[val](std::error_code ec, std::size_t len) {
		if (!ec) {
			read_body(val, ec);
		} else {
			val.owner->transverse_callables<interface_igtl>(0, ec, val.message_to_receive);
			val.owner->get_socket().get_underlying_socket().close();
		}				
		}
	);
}

void read_header(IgtlinkClientConnection val, std::error_code ec) {
	//we have a message fully unpacked in memory that we must broadcast to all
	//listeners of the interface. We do this by calling the templated broadcast method
	size_t temp = 0;
	assert(val.message_to_receive.IsNotNull());

	val.owner->transverse_callables<interface_igtl>(temp, ec, val.message_to_receive);

	val.header_to_receive = igtl::MessageHeader::New();
	val.header_to_receive->InitPack();
	val.message_to_receive = igtl::MessageBase::New();
	asio::async_read(val.owner->get_socket().get_underlying_socket(),
		asio::buffer(val.header_to_receive->GetPackPointer(), val.header_to_receive->GetPackSize()), asio::transfer_all(),
		[val](std::error_code ec, std::size_t len){
		if (!ec) {
			read_body(val, ec);
		} else {
			val.owner->transverse_callables<interface_igtl>(0, ec, val.message_to_receive);
			val.owner->get_socket().get_underlying_socket().close();
		}}
	);
}

void read_body(IgtlinkClientConnection val, std::error_code ec) {
	val.header_to_receive->Unpack();
	val.message_to_receive = igtl::MessageBase::New();
	val.message_to_receive->SetMessageHeader(val.header_to_receive);
	val.message_to_receive->AllocatePack();

	asio::async_read(val.owner->get_socket().get_underlying_socket(),
		asio::buffer(val.message_to_receive->GetPackBodyPointer(), val.message_to_receive->GetPackBodySize()), asio::transfer_all(),
		[val](std::error_code ec, std::size_t len){
		if (!ec)
			read_header(val, ec);
		else {
			val.owner->transverse_callables<interface_igtl>(0, ec, val.message_to_receive);
			val.owner->get_socket().get_underlying_socket().close();
		}}
	);
}
}

void start(Client* client_pointer) {
	curan::utils::cout << "starting to:";
	implementation::IgtlinkClientConnection val{ client_pointer };
	implementation::read_header_first_time(std::move(val));	
}


};
}
}
}
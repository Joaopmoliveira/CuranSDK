#include "communication/ProtoIGTL.h"
#include "communication/Client.h"
#include "utils/Logger.h"

namespace curan {
namespace communication {
namespace protocols {
namespace igtlinkimplementation {

IgtlinkClientConnection::IgtlinkClientConnection(std::shared_ptr<Client<igtlink>> supplied_owner) : owner{ supplied_owner }
{
}

}

void igtlink::read_header_first_time(igtlinkimplementation::IgtlinkClientConnection val) {
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
			val.owner->transverse_callables(0, ec, val.message_to_receive);
			val.owner->get_socket().close();
		}				
		}
	);
}

void igtlink::read_header(igtlinkimplementation::IgtlinkClientConnection val, std::error_code ec) {
	//we have a message fully unpacked in memory that we must broadcast to all
	//listeners of the interface. We do this by calling the templated broadcast method
	size_t temp = 0;
	assert(val.message_to_receive.IsNotNull());

	val.owner->transverse_callables(temp, ec, val.message_to_receive);

	val.header_to_receive = igtl::MessageHeader::New();
	val.header_to_receive->InitPack();
	val.message_to_receive = igtl::MessageBase::New();
	asio::async_read(val.owner->get_socket().get_underlying_socket(),
		asio::buffer(val.header_to_receive->GetPackPointer(), val.header_to_receive->GetPackSize()), asio::transfer_all(),
		[val](std::error_code ec, std::size_t len){
		if (!ec) {
			read_body(val, ec);
		} else {
			val.owner->transverse_callables(0, ec, val.message_to_receive);
			val.owner->get_socket().close();
		}}
	);
}

void igtlink::read_body(igtlinkimplementation::IgtlinkClientConnection val, std::error_code ec) {
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
			val.owner->transverse_callables(0, ec, val.message_to_receive);
			val.owner->get_socket().close();
		}}
	);
}

void igtlink::start(std::shared_ptr<Client<igtlink>> client_pointer) {
	read_header_first_time(igtlinkimplementation::IgtlinkClientConnection{client_pointer});	
}


};
}
}
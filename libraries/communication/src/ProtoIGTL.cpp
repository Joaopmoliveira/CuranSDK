#include "communication/ProtoIGTL.h"
#include "communication/Client.h"

namespace curan {
	namespace communication {
		namespace protocols {
			namespace igtlink {
				namespace implementation {

					IgtlinkClientConnection::IgtlinkClientConnection(Client* supplied_owner) : owner{ supplied_owner } {
						header_to_receive = igtl::MessageHeader::New();
						header_to_receive->InitPack();
					}

					void read_header_first_time(IgtlinkClientConnection val) {
						asio::async_read(val.owner->get_socket().get_underlying_socket(),
							asio::buffer(val.header_to_receive->GetPackPointer(), val.header_to_receive->GetPackSize()),
							[val](std::error_code ec, std::size_t len)
							{
								if (!ec && val.header_to_receive->Unpack())
									read_body(val, ec);
								else
									val.owner->get_socket().get_underlying_socket().close();
							});
					}

					void read_header(IgtlinkClientConnection val, std::error_code ec) {
						//we have a message fully unpacked in memory that we must broadcast to all
						//listeners of the interface. We do this by calling the templated broadcast method
						auto temp = (size_t)status::OK;

						val.owner->transverse_callables<interface_igtl>(temp, ec, val.message_to_receive);

						asio::async_read(val.owner->get_socket().get_underlying_socket(),
							asio::buffer(val.header_to_receive->GetPackPointer(), val.header_to_receive->GetPackSize()),
							[val](std::error_code ec, std::size_t len)
							{
								if (!ec && val.header_to_receive->Unpack())
									read_body(val, ec);
								else
									val.owner->get_socket().get_underlying_socket().close();
							});
					}

					void read_body(IgtlinkClientConnection val, std::error_code ec) {
						val.header_to_receive->Unpack();
						val.message_to_receive->SetMessageHeader(val.header_to_receive);
						val.message_to_receive->AllocatePack();
						asio::async_read(val.owner->get_socket().get_underlying_socket(),
							asio::buffer(val.message_to_receive->GetPackBodyPointer(), val.message_to_receive->GetPackBodySize()),
							[val](std::error_code ec, std::size_t len)
							{
								if (!ec && val.header_to_receive->Unpack())
									read_header(val, ec);
								else
									val.owner->get_socket().get_underlying_socket().close();
							});
					}
				}

				void start(Client* client_pointer) {
					std::cout << "starting to print stuff\n";
					implementation::IgtlinkClientConnection val{ client_pointer };
					implementation::read_header_first_time(std::move(val));
				}


			};
		}

	}
}
#include "communication/ProtoIGTL.h"
#include "communication/Client.h"
#include "utils/Logger.h"

namespace curan {
	namespace communication {
		namespace protocols {
			namespace igtlink {
				namespace implementation {

					IgtlinkClientConnection::IgtlinkClientConnection(Client* supplied_owner) : owner{ supplied_owner } {

					}

					void read_header_first_time(IgtlinkClientConnection val) {
						val.header_to_receive = igtl::MessageHeader::New();
						val.header_to_receive->InitPack();
						assert(val.header_to_receive->GetPackPointer()!=nullptr);
						assert(val.header_to_receive->GetPackSize()>=0);
						asio::async_read(val.owner->get_socket().get_underlying_socket(),
							asio::buffer(val.header_to_receive->GetPackPointer(), val.header_to_receive->GetPackSize()), asio::transfer_all(),
							[val](std::error_code ec, std::size_t len)
							{
								if (!ec) {
									read_body(val, ec);
								}
								else {
									utils::console->info("failed to read first header");
									val.owner->get_socket().get_underlying_socket().close();
								}
									
							});
					}

					void read_header(IgtlinkClientConnection val, std::error_code ec) {
						try {
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
								[val](std::error_code ec, std::size_t len)
								{
									if (!ec) {
										read_body(val, ec);
									}
									else {
										utils::console->info("failed to read header");
										val.owner->get_socket().get_underlying_socket().close();
									}
								});
						}
						catch (...) {
							utils::console->info("exception thrown");
						}
					}

					void read_body(IgtlinkClientConnection val, std::error_code ec) {
						try {
							val.header_to_receive->Unpack();
							val.message_to_receive = igtl::MessageBase::New();
							val.message_to_receive->SetMessageHeader(val.header_to_receive);
							val.message_to_receive->AllocatePack();

							asio::async_read(val.owner->get_socket().get_underlying_socket(),
								asio::buffer(val.message_to_receive->GetPackBodyPointer(), val.message_to_receive->GetPackBodySize()), asio::transfer_all(),
								[val](std::error_code ec, std::size_t len)
								{
									if (!ec)
										read_header(val, ec);
									else {
										utils::console->info("failed to read body");
										val.owner->get_socket().get_underlying_socket().close();

									}

								});
						}
						catch (...) {
							utils::console->info("exception thrown");
						}
					}
				}

				void start(Client* client_pointer) {
					utils::console->info("starting to:");
					implementation::IgtlinkClientConnection val{ client_pointer };
					implementation::read_header_first_time(std::move(val));
					
				}


			};
		}

	}
}
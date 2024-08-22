#include "communication/ProtoFRI.h"
#include "communication/Client.h"
#include "utils/Logger.h"

namespace curan
{
	namespace communication
	{
		namespace protocols
		{
			namespace friimplementation
			{

				FRIClientConnection::FRIClientConnection(std::shared_ptr<Client<fri>> supplied_owner) : owner{supplied_owner}
				{
				}

			};

			void fri::read_header_first_time(friimplementation::FRIClientConnection val)
			{
				asio::async_read(val.owner->get_socket().get_underlying_socket(),
								 asio::buffer(val.message->get_buffer(), val.message->get_header_size()), asio::transfer_all(),
								 [val](std::error_code ec, std::size_t len)
								 {
									 if (!ec)
									 {
										 read_body(val, ec);
									 }
									 else
									 {
										 val.owner->transverse_callables(0, ec, val.message);
										 val.owner->get_socket().close();
									 }
								 });
			}

			void fri::read_header(friimplementation::FRIClientConnection val, std::error_code ec)
			{
				// we have a message fully unpacked in memory that we must broadcast to all
				// listeners of the interface. We do this by calling the templated broadcast method
				size_t temp = 0;
				val.message->deserialize();
				val.owner->transverse_callables(temp, ec, val.message);
				val.message = std::make_shared<FRIMessage>();
				asio::async_read(val.owner->get_socket().get_underlying_socket(),
								 asio::buffer(val.message->get_buffer(), val.message->get_header_size()), asio::transfer_all(),
								 [val](std::error_code ec, std::size_t len)
								 {
		if (!ec) {
			read_body(val, ec);
		} else {
			val.owner->transverse_callables(0, ec, val.message);
			val.owner->get_socket().close();
		} });
			}

			void fri::read_body(friimplementation::FRIClientConnection val, std::error_code ec)
			{
				val.message->deserialize_header();

				asio::async_read(val.owner->get_socket().get_underlying_socket(),
								 asio::buffer(val.message->get_body_buffer(), val.message->get_body_size()), asio::transfer_all(),
								 [val](std::error_code ec, std::size_t len)
								 {
		if (!ec)
			read_header(val, ec);
		else {
			val.owner->transverse_callables(0, ec, val.message);
			val.owner->get_socket().close();
		} });
			}

			void fri::start(std::shared_ptr<Client<fri>> client_pointer)
			{
				friimplementation::FRIClientConnection val{client_pointer};
				val.message = std::make_shared<FRIMessage>();
				fri::read_header_first_time(std::move(val));
			}

		}

	};
}
#include "communication/ProcessorOIGTL.h"

namespace curan{
    namespace communication{
        
        ProcessorOIGTL::ProcessorOIGTL(asio::io_context& io_context) : io_context_(io_context),
			socket_(io_context)
		{

		}

		ProcessorOIGTL::~ProcessorOIGTL()
		{
			this->close();
		}

		void ProcessorOIGTL::write(igtl::MessageBase::Pointer msg)
		{
			curan::utils::console->debug("asyncronous write has been submited to the io_context");
			asio::post(io_context_,
				std::bind(&ProcessorOIGTL::do_write, this, msg));
		}

		void ProcessorOIGTL::close()
		{
			change_status(Status::CK_CLOSED);
			curan::utils::console->debug("asyncronous closure has been submited to the io_context");
			asio::post(io_context_,
				std::bind(&ProcessorOIGTL::do_close, this));
		}

		void ProcessorOIGTL::connect(const asio::ip::tcp::resolver::results_type& endpoints)
		{
			if (current_status == Status::CK_OPEN)
				return;
			change_status(Status::CK_CLOSED);
			curan::utils::console->debug("asyncronous connect has been submited to the io_context");
			asio::async_connect(socket_, endpoints,
				std::bind(&ProcessorOIGTL::handle_connect,
					this, std::placeholders::_1));
		}

		ProcessorOIGTL::Status ProcessorOIGTL::getStatus()
		{
			return current_status;
		}

		void ProcessorOIGTL::change_status(Status status)
		{
			current_status = status;
			signal_connection_status(status);
		}

		void ProcessorOIGTL::handle_connect(const asio::error_code& error)
		{
			if (!error)
			{
				change_status(Status::CK_OPEN);
				message_header = igtl::MessageHeader::New();
				message_header->InitPack();
				curan::utils::console->debug("connection established and asyncronous read request has been submited to the io_context");
				asio::async_read(socket_,
					asio::buffer(message_header->GetPackPointer(), (size_t)message_header->GetPackSize()),
					std::bind(&ProcessorOIGTL::handle_read_header, this,
						std::placeholders::_1));
			}
			else
			{
				curan::utils::console->error("connection error happened: {}", error.value());
				close();
			}
		}

		void ProcessorOIGTL::handle_read_header(const asio::error_code& error)
		{
			if (!error)
			{
				// Deserialize the header
				message_header->Unpack();
				body_data.reserve((size_t)message_header->GetBodySizeToRead());
				asio::async_read(socket_,
					asio::buffer(body_data.data(), (size_t)message_header->GetBodySizeToRead()),
					std::bind(&ProcessorOIGTL::handle_read_body, this,
						std::placeholders::_1));
				curan::utils::console->debug("asyncronous read body request has been submited to the io_context");
			}
			else
			{
				curan::utils::console->error("connection error happened: {}", error.value());
				close();
			}
		}

		void ProcessorOIGTL::handle_read_body(const asio::error_code& error)
		{
			if (!error)
			{
				curan::utils::console->debug("successfully read message body from socket");

				igtl::MessageBase::Pointer message_pointer = igtl::MessageBase::New();
				message_pointer->SetMessageHeader(message_header);
				message_pointer->AllocatePack();
				std::memcpy(message_pointer->GetPackBodyPointer(), body_data.data(), (size_t)message_header->GetBodySizeToRead());
				signal_received(message_pointer);

				message_header = igtl::MessageHeader::New();
				message_header->InitPack();
				asio::async_read(socket_,
					asio::buffer(message_header->GetPackPointer(), (size_t)message_header->GetPackSize()),
					std::bind(&ProcessorOIGTL::handle_read_header, this,
						std::placeholders::_1));
				curan::utils::console->debug("asyncronous read header request has been submited to the io_context");
			}
			else
			{
				curan::utils::console->error("connection error happened: {}", error.value());
				close();
			}
		}

		void ProcessorOIGTL::do_write(igtl::MessageBase::Pointer msg)
		{
			bool write_in_progress = !outbond_messages.empty();
			curan::utils::console->debug("do write message has been called, is there current work? {}", write_in_progress);
			outbond_messages.push(msg);
			if (!write_in_progress)
			{
				auto msn = outbond_messages.try_front();
				asio::const_buffer buffer{ msn->GetBufferPointer(),(size_t)msn->GetBufferSize() };
				asio::async_write(socket_,
					buffer,
					std::bind(&ProcessorOIGTL::handle_write, this,
						std::placeholders::_1));
				curan::utils::console->info("asyncronous write message request has been submited to the io_context");
			}
		}

		void ProcessorOIGTL::handle_write(const asio::error_code& error)
		{
			if (!error)
			{
				curan::utils::console->debug("message has been sent");
				outbond_messages.try_pop();
				if (!outbond_messages.empty())
				{
					auto msn = outbond_messages.try_front();
					asio::const_buffer buffer{ msn->GetBufferPointer(),(size_t)msn->GetBufferSize() };
					asio::async_write(socket_,
						buffer,
						std::bind(&ProcessorOIGTL::handle_write, this,
							std::placeholders::_1));
					curan::utils::console->debug("asyncronous write message request has been submited to the io_context");
				}
			}
			else
			{
				curan::utils::console->error("connection error happened: {}", error.value());
				close();
			}
		}

		void ProcessorOIGTL::do_close()
		{
			//this is a stupid fix... given that the "main thread" is blocked while waiting for message inputs then we need to create
			// a signal to jump start the thread and clean up all resources
			socket_.close();
			io_context_.stop();
			signal_connection_status(Status::CK_CLOSED);
		}

		void GetIOContext(asio::io_context** io_context)
		{
			static asio::io_context io_ctxt;
			*io_context = &io_ctxt;
		}
    }
}
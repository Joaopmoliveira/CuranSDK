#include "CkLibrary.h"

namespace curan {
	namespace communication {
		Functor::Functor()
		{
			GetIOContext(&io_context_);
		}

		void Functor::operator()() {
			try {
				using work_guard_type = asio::executor_work_guard<asio::io_context::executor_type>;
				work_guard_type work_guard(io_context_->get_executor());
				is_active = true;
				curan::utils::console->warn("connection thread started running");
				io_context_->run();
				is_active = false;
				curan::utils::console->warn("connection thread stoped running");
				io_context_->stop();
			}
			catch (std::exception& e) {
				curan::utils::console->warn("connection thread stoped running due to exception");
				curan::utils::console->warn(e.what());
				io_context_->stop();
			}

		}

		bool Functor::is_running()
		{
			return is_active;
		}

		ProcessorOpenIGTLink::ProcessorOpenIGTLink(asio::io_context& io_context) : io_context_(io_context),
			socket_(io_context)
		{

		}

		ProcessorOpenIGTLink::~ProcessorOpenIGTLink()
		{
			this->close();
		}

		void ProcessorOpenIGTLink::write(igtl::MessageBase::Pointer msg)
		{
			curan::utils::console->debug("asyncronous write has been submited to the io_context");
			asio::post(io_context_,
				std::bind(&ProcessorOpenIGTLink::do_write, this, msg));
		}

		void ProcessorOpenIGTLink::close()
		{
			change_status(Status::CK_CLOSED);
			curan::utils::console->debug("asyncronous closure has been submited to the io_context");
			asio::post(io_context_,
				std::bind(&ProcessorOpenIGTLink::do_close, this));
		}

		void ProcessorOpenIGTLink::connect(const asio::ip::tcp::resolver::results_type& endpoints)
		{
			if (current_status == Status::CK_OPEN)
				return;
			change_status(Status::CK_CLOSED);
			curan::utils::console->debug("asyncronous connect has been submited to the io_context");
			asio::async_connect(socket_, endpoints,
				std::bind(&ProcessorOpenIGTLink::handle_connect,
					this, std::placeholders::_1));
		}

		ProcessorOpenIGTLink::Status ProcessorOpenIGTLink::getStatus()
		{
			return current_status;
		}

		void ProcessorOpenIGTLink::change_status(Status status)
		{
			current_status = status;
			signal_connection_status(status);
		}

		void ProcessorOpenIGTLink::handle_connect(const asio::error_code& error)
		{
			if (!error)
			{
				change_status(Status::CK_OPEN);
				message_header = igtl::MessageHeader::New();
				message_header->InitPack();
				curan::utils::console->debug("connection established and asyncronous read request has been submited to the io_context");
				asio::async_read(socket_,
					asio::buffer(message_header->GetPackPointer(), (size_t)message_header->GetPackSize()),
					std::bind(&ProcessorOpenIGTLink::handle_read_header, this,
						std::placeholders::_1));
			}
			else
			{
				curan::utils::console->error("connection error happened: {}", error.value());
				close();
			}
		}

		void ProcessorOpenIGTLink::handle_read_header(const asio::error_code& error)
		{
			if (!error)
			{
				// Deserialize the header
				message_header->Unpack();
				body_data.reserve((size_t)message_header->GetBodySizeToRead());
				asio::async_read(socket_,
					asio::buffer(body_data.data(), (size_t)message_header->GetBodySizeToRead()),
					std::bind(&ProcessorOpenIGTLink::handle_read_body, this,
						std::placeholders::_1));
				curan::utils::console->debug("asyncronous read body request has been submited to the io_context");
			}
			else
			{
				curan::utils::console->error("connection error happened: {}", error.value());
				close();
			}
		}

		void ProcessorOpenIGTLink::handle_read_body(const asio::error_code& error)
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
					std::bind(&ProcessorOpenIGTLink::handle_read_header, this,
						std::placeholders::_1));
				curan::utils::console->debug("asyncronous read header request has been submited to the io_context");
			}
			else
			{
				curan::utils::console->error("connection error happened: {}", error.value());
				close();
			}
		}

		void ProcessorOpenIGTLink::do_write(igtl::MessageBase::Pointer msg)
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
					std::bind(&ProcessorOpenIGTLink::handle_write, this,
						std::placeholders::_1));
				curan::utils::console->info("asyncronous write message request has been submited to the io_context");
			}
		}

		void ProcessorOpenIGTLink::handle_write(const asio::error_code& error)
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
						std::bind(&ProcessorOpenIGTLink::handle_write, this,
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

		void ProcessorOpenIGTLink::do_close()
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

		ChannelManager::ChannelManager()
		{
		}

		ChannelManager* ChannelManager::Get()
		{
			static ChannelManager channel_manager;
			return &channel_manager;
		}

		void ChannelManager::start_channel(std::string name, asio::io_context& ctx, error_callback callback)
		{
			auto it = openigtlink_channel_list.find(name);
			if (it == openigtlink_channel_list.end())
			{
				if (!functor.is_running())
				{
					curan::utils::Job pending_task = curan::utils::Job{ "communication functor",std::bind(&Functor::operator(), &functor) };
					curan::utils::ThreadPool* pool = curan::utils::ThreadPool::Get();
					pool->Submit(pending_task);
				}
				// there is no channel with this name so we can continue
				std::shared_ptr<ProcessorOpenIGTLink> pointer_to_processor = std::make_shared<ProcessorOpenIGTLink>(ctx);
				openigtlink_channel_list.emplace(name, pointer_to_processor);
				if (callback)
					callback(Error::SUCCESS);
				return;
			}
			else {
				// there is already a channel with this name, we need to get 
				//out but first we should check if the functor of this channel 
				//is actually running, if not we should jump start it again
				if (!functor.is_running())
				{
					curan::utils::Job pending_task = curan::utils::Job{ "communication functor",std::bind(&Functor::operator(), &functor) };
					curan::utils::ThreadPool* pool = curan::utils::ThreadPool::Get();
					pool->Submit(pending_task);
				}
				if (callback)
					callback(Error::CHANNEL_ALREADY_PRESENT);
				return;
			}
		}



		ChannelManager::Error ChannelManager::get_channel(std::string name, std::shared_ptr<ProcessorOpenIGTLink>& channel)
		{
			auto it = openigtlink_channel_list.find(name);
			if (it == openigtlink_channel_list.end())// there is no channel with this name so we need to return a nullptr
				return Error::CHANNEL_NOT_PRESENT;
			else
				channel = it->second;
			return Error::SUCCESS;
		}

		ChannelManager::Error ChannelManager::terminate_channel(std::string name)
		{
			auto it = openigtlink_channel_list.find(name);
			if (it == openigtlink_channel_list.end())// there is no channel with this name so we need to return a nullptr
			{
				return Error::CHANNEL_NOT_PRESENT;
			}
			else
			{
				openigtlink_channel_list.erase(it);
				return Error::SUCCESS;
			}
		}

		void ChannelManager::terminate_all_channels()
		{
			auto it = openigtlink_channel_list.begin();
			while (it != openigtlink_channel_list.end()) {
				it->second->close();
				++it;
			}

		}

	}
}

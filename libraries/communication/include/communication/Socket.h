#ifndef CURAN_SOCKET_HEADER_FILE_
#define CURAN_SOCKET_HEADER_FILE_

#include <asio.hpp>
#include <list>
#include <functional>
#include "utils/MemoryUtils.h"
#include <memory>
#include "ProtocolValidationHelper.h"

namespace curan
{
	namespace communication
	{

		template<typename protocol>
		struct Client;
		
		template <typename protocol>
		class Socket
		{
			static_assert(std::is_invocable_v<decltype(protocol::start), std::shared_ptr<Client<protocol>>>, "the protocol must have a static start() function that receives a templated client");
			static_assert(is_type_complete_v<typename protocol::signature>, "the protocol must have signature type function that broadcasts the the protocol messages");

			asio::ip::tcp::socket _socket;
			asio::io_context &_cxt;
			std::list<std::shared_ptr<utilities::MemoryBuffer>> to_send;

			bool is_connected = false;
			bool is_closed = false;
			std::weak_ptr<Client<protocol>> owner;
			asio::high_resolution_timer timer;

			friend class Client<protocol>;

		public:
			Socket(asio::io_context& io_context,asio::ip::tcp::socket socket) : _cxt(io_context), timer{_cxt},_socket{ std::move(socket) } ,is_connected{true}
			{

			};

			template <class _Rep, class _Period>
			Socket(asio::io_context &io_context,
				   const asio::ip::tcp::resolver::results_type &endpoints,
				   const std::chrono::duration<_Rep, _Period> &deadline,
				   std::function<void(std::error_code ec)> connection_callback) : _cxt(io_context),
																				  _socket(io_context),
																				  is_connected{false},
																				  timer{io_context} {

																				  };

			Socket(asio::io_context &io_context,
				   const asio::ip::tcp::resolver::results_type& endpoints) : 
						_cxt(io_context),
						_socket(io_context) ,
						is_connected{false},
						timer{_cxt}{}

			~Socket()
			{
				asio::error_code error;
				get_underlying_socket().shutdown(asio::socket_base::shutdown_both, error);
			};

			inline asio::ip::tcp::socket &get_underlying_socket()
			{
				return _socket;
			}

			void post()
			{
				auto lifetimekeaper = owner.lock();
				asio::post(_cxt,
						   [this, lifetimekeaper]()
						   {
							   bool shoud_write = !to_send.empty();
							   if (shoud_write && sendable())
								   do_write();
						   });
			};

			void post(std::shared_ptr<utilities::MemoryBuffer> buff)
			{
				auto lifetimekeaper = owner.lock();
				asio::post(_cxt,
						   [this, buff, lifetimekeaper]()
						   {
							   bool shoud_write = to_send.empty();
							   auto size_before = to_send.size();
							   to_send.push_back(buff);
							   // std::printf("(pointer = %llu ) before (%llu) after (%llu)\n",(size_t) this, size_before, to_send.size());
							   if (shoud_write && is_connected)
								   do_write();
						   });
			};

			void close()
			{
				if (is_closed)
					return;
				auto lifetimekeaper = owner.lock();
				asio::post(_cxt, [this, lifetimekeaper]()
						   { 
		if(get_underlying_socket().is_open()) {
			asio::error_code error;
			get_underlying_socket().shutdown(asio::socket_base::shutdown_both,error);
			is_closed = true;
			get_underlying_socket().close(error);
		} });
			}

			inline bool sendable()
			{
				return is_connected && !is_closed;
			};

		private:
			void lauch_start_after_connect(std::shared_ptr<Client<protocol>> client)
			{
				is_connected = true;
				protocol::start(client);
			}

			void trigger_start(std::weak_ptr<Client<protocol>> in_owner, const asio::ip::tcp::resolver::results_type &endpoints)
			{
				owner = in_owner;
				auto client = owner.lock();
				asio::async_connect(_socket, endpoints,
									[this, client](std::error_code ec, asio::ip::tcp::endpoint e)
									{
										if (!ec)
										{
											lauch_start_after_connect(client);
											post();
										}
									});
			}

			void trigger_start(std::weak_ptr<Client<protocol>> in_owner)
			{
				owner = in_owner;
				assert(!owner.expired());
				auto shared_version = owner.lock();
				lauch_start_after_connect(shared_version);
			}

			template <class _Rep, class _Period>
			void trigger_start(std::weak_ptr<Client<protocol>> in_owner, const asio::ip::tcp::resolver::results_type &endpoints, const std::chrono::duration<_Rep, _Period> &deadline, std::function<void(std::error_code ec)> connection_callback)
			{
				owner = in_owner;
				timer.expires_after(deadline);
				auto client = owner.lock();
				timer.async_wait([this, client, connection_callback](asio::error_code ec)
								 {
				if (is_connected)
					return;
				close();
				connection_callback(ec); });
				asio::async_connect(_socket, endpoints,
									[this, client, connection_callback](std::error_code ec, asio::ip::tcp::endpoint e)
									{
										connection_callback(ec);
										if (!ec)
										{
											lauch_start_after_connect(client);
											post();
										}
									});
			}

			void do_write()
			{
				assert(!to_send.empty());
				assert(to_send.front()->begin()->data() != nullptr);
				auto lifetimekeaper = owner.lock();
				asio::async_write(get_underlying_socket(),
								  asio::buffer(to_send.front()->begin()->data(), to_send.front()->begin()->size()), asio::transfer_all(),
								  [this, lifetimekeaper](std::error_code ec, std::size_t /*length*/)
								  {
									  if (!ec)
									  {
										  bool is_empty = to_send.empty();
										  if (!to_send.empty())
										  {
											  // std::printf("(pointer = %llu ) contained messages: %llu empty val(%d)\n",(size_t)this,to_send.size(), is_empty);
											  to_send.pop_front();
										  }
										  //
										  if (!to_send.empty() && sendable())
											  do_write();
									  }
									  else
									  {
										  close();
									  }
								  });
			};
		};
	}
}

#endif
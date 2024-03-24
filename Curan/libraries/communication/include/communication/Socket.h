#ifndef CURAN_SOCKET_HEADER_FILE_
#define CURAN_SOCKET_HEADER_FILE_

#include <asio.hpp>
#include <list>
#include <functional>
#include "Protocols.h"
#include "utils/MemoryUtils.h"

namespace curan {
	namespace communication {

		class Client;

		/*
		The socket is an abstraction of
		the underlying socket of asio.
		*/
		class Socket {
			asio::ip::tcp::socket _socket;
			asio::io_context& _cxt;
			std::list<std::shared_ptr<utilities::MemoryBuffer>> to_send;
			std::function<void(Client*)> start;
			bool is_connected = false;
			bool is_closed = false;
			Client* owner = nullptr;
			asio::high_resolution_timer timer;
			size_t pending_asyncronous_operations = 0;

		public:
			Socket(asio::io_context& io_context,
				const asio::ip::tcp::resolver::results_type& endpoints,
				callable callable, Client* owner);

			template <class _Rep, class _Period>
			Socket(asio::io_context& io_context,
						const asio::ip::tcp::resolver::results_type& endpoints,
						callable callable, 
						Client* in_owner,
						const std::chrono::duration<_Rep, _Period>& deadline,
						std::function<void(std::error_code ec)> connection_callback) : _cxt(io_context),
																		 _socket(io_context) ,
																		is_connected{false},
																		timer{io_context},
																		owner{ in_owner }
			{
				start = get_interface(callable);
				timer.expires_after(deadline);
				timer.async_wait([this,connection_callback](asio::error_code ec) {
					if(is_connected)
						return;
					close();
					connection_callback(ec);
    			});
				asio::async_connect(_socket, endpoints,
					[this,connection_callback](std::error_code ec, asio::ip::tcp::endpoint e){
						connection_callback(ec);
						if (!ec){
							is_connected = true;
							post();
							start(owner);
						}
				});
			};

			Socket(asio::io_context& io_context,
				asio::ip::tcp::socket socket,
				callable callable, Client* owner);

			~Socket();

			inline asio::ip::tcp::socket& get_underlying_socket() {
				return _socket;
			}

			void handle_connect(std::error_code ec, asio::ip::tcp::endpoint e);

			void post(std::shared_ptr<utilities::MemoryBuffer> buff);

			void post();

			void close();

			inline bool sendable() {
				return is_connected && !is_closed;
			};

	private:

			void do_write();

		};
	}
}

#endif
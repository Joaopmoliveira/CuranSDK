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

		public:
			Socket(asio::io_context& io_context,
				const asio::ip::tcp::resolver::results_type& endpoints,
				callable callable, Client* owner);

			Socket(asio::io_context& io_context,
				asio::ip::tcp::socket socket,
				callable callable, Client* owner);

			~Socket();

			inline asio::ip::tcp::socket& get_underlying_socket() {
				return _socket;
			}

			void handle_connect(std::error_code ec, asio::ip::tcp::endpoint e);

			void post(std::shared_ptr<utilities::MemoryBuffer> buff);

			void do_write();

			void close();
		};
	}
}

#endif
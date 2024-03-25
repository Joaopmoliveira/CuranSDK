#ifndef CURAN_SOCKET_HEADER_FILE_
#define CURAN_SOCKET_HEADER_FILE_

#include <asio.hpp>
#include <list>
#include <functional>
#include "Protocols.h"
#include "utils/MemoryUtils.h"
#include <memory>

namespace curan {
	namespace communication {
		
		/*
		The socket is an abstraction of
		the underlying socket of asio.
		*/
		class Socket {
			asio::ip::tcp::socket _socket;
			asio::io_context& _cxt;
			std::list<std::shared_ptr<utilities::MemoryBuffer>> to_send;
			std::function<void(std::shared_ptr<Client>)> start;
			bool is_connected = false;
			bool is_closed = false;
			std::weak_ptr<Client> owner;
			asio::high_resolution_timer timer;

			friend class Client;

		public:
			Socket(asio::io_context& io_context,const asio::ip::tcp::resolver::results_type& endpoints,callable callable);

			template <class _Rep, class _Period>
			Socket(asio::io_context& io_context,
						const asio::ip::tcp::resolver::results_type& endpoints,
						callable callable, 
						const std::chrono::duration<_Rep, _Period>& deadline,
						std::function<void(std::error_code ec)> connection_callback) : _cxt(io_context),
																		 _socket(io_context) ,
																		is_connected{false},
																		timer{io_context}
			{

			};

			Socket(asio::io_context& io_context,
				asio::ip::tcp::socket socket,
				callable callable);

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

		void lauch_start_after_connect(std::shared_ptr<Client>);

		void trigger_start(callable callable, std::weak_ptr<Client> owner, const asio::ip::tcp::resolver::results_type& endpoints);

		void trigger_start(callable callable, std::weak_ptr<Client> owner);

		template <class _Rep, class _Period>
		void trigger_start(callable callable, std::weak_ptr<Client> in_owner, const asio::ip::tcp::resolver::results_type& endpoints, const std::chrono::duration<_Rep, _Period>& deadline,std::function<void(std::error_code ec)> connection_callback) {
			owner = in_owner;
			start = get_interface(callable);
			timer.expires_after(deadline);
			auto client = owner.lock();
			timer.async_wait([this, client, connection_callback](asio::error_code ec) {
				if (is_connected)
					return;
				close();
				connection_callback(ec);
				});
			asio::async_connect(_socket, endpoints,
				[this, client, connection_callback](std::error_code ec, asio::ip::tcp::endpoint e) {
					connection_callback(ec);
					if (!ec) {
						lauch_start_after_connect(client);
						post();
					}
				});
		}

		void do_write();

		};
	}
}

#endif
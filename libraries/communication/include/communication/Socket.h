#ifndef CURAN_SOCKET_HEADER_FILE_
#define CURAN_SOCKET_HEADER_FILE_

#include <asio.hpp>
#include <list>
#include <functional>

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
			std::list<std::shared_ptr<curan::utils::memory_buffer>> to_send;
			std::function<void(Client*)> start;

		public:
			Socket(asio::io_context& io_context,
				const asio::ip::tcp::resolver::results_type& endpoints,
				callable callable, Client* owner) : _cxt(io_context),
				_socket(io_context) {
				start = std::visit<std::function<void(Client*)>, Visitor>(Visitor(), callable);
				asio::async_connect(_socket, endpoints,
					[this, owner](std::error_code ec, asio::ip::tcp::endpoint e)
					{
						if (!ec)
							start(owner);

					});
			}

			Socket(asio::io_context& io_context,
				asio::ip::tcp::socket socket,
				callable callable, Client* owner) : _cxt(io_context), _socket{ std::move(socket) } {
				start = std::visit<std::function<void(Client*)>, Visitor>(Visitor(), callable);
				start(owner);
			}

			~Socket() {
				_socket.close();
			}

			inline asio::ip::tcp::socket& get_underlying_socket() {
				return _socket;
			}

			void handle_connect(std::error_code ec, asio::ip::tcp::endpoint e) {

			}

			void post(std::shared_ptr<curan::utils::memory_buffer> buff) {
				asio::post(_cxt,
					[this, buff]()
					{
						bool write_in_progress = !to_send.empty();
						to_send.push_back(buff);
						if (!write_in_progress)
						{
							do_write();
						}
					});
			}

			void do_write()
			{
				asio::async_write(get_underlying_socket(),
					asio::buffer(to_send.front()->begin()->data(), to_send.front()->begin()->size()),
					[this](std::error_code ec, std::size_t /*length*/) {
						if (!ec) {
							to_send.pop_front();
							if (!to_send.empty())
								do_write();
						}
						else
							get_underlying_socket().close();
					});
			}

			void close() {
				asio::post(_cxt, [this]() { get_underlying_socket().close(); });
			}
		};
	}
}

#endif
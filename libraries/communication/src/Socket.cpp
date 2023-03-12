#include "communication/Socket.h"
#include <variant>

namespace curan {
	namespace communication {
		class Client;
		Socket::Socket(asio::io_context& io_context,
			const asio::ip::tcp::resolver::results_type& endpoints,
			callable callable, Client* owner) : _cxt(io_context),
			_socket(io_context) 
		{
			start = get_interface(callable);
			asio::async_connect(_socket, endpoints,
				[this, owner](std::error_code ec, asio::ip::tcp::endpoint e)
				{
					if (!ec)
						start(owner);

				});
		};

		Socket::Socket(asio::io_context& io_context,
			asio::ip::tcp::socket socket,
			callable callable, Client* owner) : _cxt(io_context), _socket{ std::move(socket) }
		{
			start = get_interface(callable);
			start(owner);
		};

		Socket::~Socket() {
			_socket.close();
		};

		void Socket::handle_connect(std::error_code ec, asio::ip::tcp::endpoint e) {

		};

		void Socket::post(std::shared_ptr<curan::utils::MemoryBuffer> buff) {
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
		};

		void Socket::do_write()
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
		};

		void Socket::close() {
			asio::post(_cxt, [this]() { get_underlying_socket().close(); });
		};
	}
}
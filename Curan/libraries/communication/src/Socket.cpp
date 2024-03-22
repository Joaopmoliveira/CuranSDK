#include "communication/Socket.h"
#include <variant>
#include "utils/Logger.h"
#include "communication/Client.h"

namespace curan {
namespace communication {

Socket::Socket(asio::io_context& io_context,
			const asio::ip::tcp::resolver::results_type& endpoints,
			callable callable, Client* owner) : _cxt(io_context),
			_socket(io_context) ,
			is_connected{false}
{
	start = get_interface(callable);
	asio::async_connect(_socket, endpoints,
		[this, owner](std::error_code ec, asio::ip::tcp::endpoint e){
			if (!ec){
				is_connected = true;
				bool shoud_write = !to_send.empty();
				do_write();
				start(owner);
			}
	});
	};

Socket::Socket(asio::io_context& io_context,
	asio::ip::tcp::socket socket,
	callable callable, Client* owner) : _cxt(io_context), _socket{ std::move(socket) } , is_connected{true}
{
	start = get_interface(callable);
	start(owner);
};

Socket::~Socket() {
	close();
};

void Socket::post(std::shared_ptr<utilities::MemoryBuffer> buff) {
	asio::post(_cxt,
		[this, buff](){
			bool shoud_write = to_send.empty();
			to_send.push_back(buff);
			if (shoud_write && is_connected)
				do_write();
		});
};

void Socket::do_write(){
	assert(to_send.size()>0);
	assert(to_send.front()->begin()->data()!=nullptr);
	asio::async_write(get_underlying_socket(),
		asio::buffer(to_send.front()->begin()->data(), to_send.front()->begin()->size()), asio::transfer_all(),
		[this](std::error_code ec, std::size_t /*length*/) {
			if (!ec) {
				if(!to_send.empty())
					to_send.pop_front();
				if (!to_send.empty())
					do_write();
				}
			else {
				close();
				utilities::cout << "failed";
			}
		});
};

void Socket::close() {
	asio::post(_cxt, [this]() { 
		if(get_underlying_socket().is_open()) {
			asio::error_code error;
			get_underlying_socket().shutdown(asio::socket_base::shutdown_both,error);
			utilities::cout << ((error) ? " shutdown requested\n" : " shutdown denied\n");
			get_underlying_socket().close(error);
			utilities::cout << ((error) ? " close requested\n" : " close denied\n");
		} 
	});
};

}
}
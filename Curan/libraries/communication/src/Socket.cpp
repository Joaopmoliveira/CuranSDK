#include "communication/Socket.h"
#include <variant>
#include "utils/Logger.h"
#include "communication/Client.h"

namespace curan {
namespace communication {

Socket::Socket(asio::io_context& io_context,
			const asio::ip::tcp::resolver::results_type& endpoints,
			callable callable) : _cxt(io_context),
			_socket(io_context) ,
			is_connected{false},
			timer{_cxt}
{

};

void Socket::trigger_start(callable callable, std::weak_ptr<Client> in_owner, const asio::ip::tcp::resolver::results_type& endpoints) {
	owner = in_owner;
	start = get_interface(callable);
	auto client = owner.lock();
	asio::async_connect(_socket, endpoints,
		[this,client](std::error_code ec, asio::ip::tcp::endpoint e) {
			if (!ec) {
				is_connected = true;
				post();
				start(client);
			}
	});
}

void Socket::trigger_start(callable callable, std::weak_ptr<Client> in_owner) {
	owner = in_owner;
	start = get_interface(callable);
	assert(!owner.expired());
	auto shared_version = owner.lock();
	start(shared_version);
}

Socket::Socket(asio::io_context& io_context,asio::ip::tcp::socket socket,callable callable) : _cxt(io_context), timer{_cxt},_socket{ std::move(socket) } ,is_connected{true}
{

};

Socket::~Socket() {
	asio::error_code error;
	get_underlying_socket().shutdown(asio::socket_base::shutdown_both, error);
};

void Socket::post(std::shared_ptr<utilities::MemoryBuffer> buff) {
	asio::post(_cxt,
		[this, buff](){
			bool shoud_write = to_send.empty();
			auto size_before = to_send.size();
			to_send.push_back(buff);
			//std::printf("(pointer = %llu ) before (%llu) after (%llu)\n",(size_t) this, size_before, to_send.size());
			if (shoud_write && is_connected)
				do_write();
		});
};

void Socket::post() {
	asio::post(_cxt,
		[this](){
			bool shoud_write = !to_send.empty();
			if (shoud_write && sendable())
				do_write();
		});
};

void Socket::do_write(){
	assert(!to_send.empty());
	assert(to_send.front()->begin()->data()!=nullptr);
	asio::async_write(get_underlying_socket(),
		asio::buffer(to_send.front()->begin()->data(), to_send.front()->begin()->size()), asio::transfer_all(),
		[this](std::error_code ec, std::size_t /*length*/) {
			if (!ec) {
				bool is_empty = to_send.empty();		
				if(!to_send.empty()){
					//std::printf("(pointer = %llu ) contained messages: %llu empty val(%d)\n",(size_t)this,to_send.size(), is_empty);
					to_send.pop_front();
				}
					//
				if (!to_send.empty() && sendable())
					do_write();
				}
			else {
				close();
				utilities::cout << "failed";
			}
		});
};

void Socket::close() {
	if (is_closed)
		return;
	asio::post(_cxt, [this]() { 
		if(get_underlying_socket().is_open()) {
			asio::error_code error;
			get_underlying_socket().shutdown(asio::socket_base::shutdown_both,error);
			is_closed = true;
			utilities::cout << ((error) ? " shutdown requested\n" : " shutdown denied\n");
			get_underlying_socket().close(error);
			utilities::cout << ((error) ? " close requested\n" : " close denied\n");
		} 
	});
};

}
}
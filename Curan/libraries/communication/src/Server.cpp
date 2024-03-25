#include "communication/Server.h"
#include "utils/Logger.h"

#include <vector>

namespace curan {
namespace communication {

Server::Server(Info& info) : _cxt{ info.io_context }, acceptor_{ _cxt, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), info.port) },connection_type{info.connection_type}  {

}

Server::Server(Info& info,std::function<void(std::error_code ec)> connection_callback) : _cxt{ info.io_context }, acceptor_{ _cxt, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), info.port) },connection_type{info.connection_type}  {
	
}

Server::~Server() {
	acceptor_.close();
}

void Server::connect(callable c) {
	if (connection_type.index() != c.index())
		throw std::runtime_error("the supplied callback is not supported against the requested interface");

	std::lock_guard<std::mutex> g{mut};
	callables.push_back(c);
	for(auto & client : list_of_clients)
		client->connect(c);
	return;
}

void Server::write(std::shared_ptr<utilities::MemoryBuffer> buffer) {
	std::lock_guard<std::mutex> g{mut};
	if (list_of_clients.size()==0)
		return;
	list_of_clients.remove_if([buffer](std::shared_ptr<Client>& client){
		if(!client->get_socket().sendable())
			return true;
		client->write(buffer);
		return false;
	}
	);			
}

void Server::accept() {
	acceptor_.async_accept(
	[this,life_time_guaranteer = shared_from_this()](std::error_code ec, asio::ip::tcp::socket socket) {
	if (!ec) {
		utilities::cout << "Server received a client";
		Client::ServerInfo info{ _cxt,connection_type,std::move(socket) };
		auto client_ptr = Client::make(info);
		std::lock_guard<std::mutex> g{mut};
		for(auto & submitted_callables : callables)
			client_ptr->connect(submitted_callables);
		list_of_clients.push_back(client_ptr);
		utilities::cout << "Server started listening for new client";
	} else {
		utilities::cout << "Server stopped listening for incoming connections\n";
	}
	accept();
	});
}

void Server::accept(std::function<bool(std::error_code ec)> connection_callback) {
	acceptor_.async_accept(
	[this,connection_callback, life_time_guaranteer = shared_from_this()](std::error_code ec, asio::ip::tcp::socket socket) {
		if (!ec) {
			utilities::cout << "Server received a client";
			Client::ServerInfo info{ _cxt,connection_type,std::move(socket) };
			auto client_ptr = Client::make(info);
			std::lock_guard<std::mutex> g{mut};
			for(auto & submitted_callables : callables)
				client_ptr->connect(submitted_callables);
			bool all_ok = connection_callback(ec);
			if (all_ok)
				list_of_clients.push_back(client_ptr);
			else
				client_ptr->get_socket().close();
			utilities::cout << "Server started listening for new client";
		} else {
			utilities::cout << "Server stopped listening for incoming connections\n";
			connection_callback(ec);
		}
		
		accept(connection_callback);
	});
}

}
}
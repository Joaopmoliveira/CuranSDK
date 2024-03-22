#include "communication/Server.h"
#include "communication/Client.h"
#include "utils/Logger.h"

#include <vector>

namespace curan {
namespace communication {

Server::Server(Info& info) : _cxt{ info.io_context }, acceptor_{ _cxt, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), info.port) },connection_type{info.connection_type}  {
	accept();
}

Server::~Server() {
	acceptor_.close();
}

std::optional<std::shared_ptr<utilities::Cancelable>> Server::connect(callable c) {
	if (connection_type.index() != c.index()){
		utilities::cout << "the supplied callback is not supported against the requested interface";
		return std::nullopt;
	}

	auto cancel = utilities::Cancelable::make_cancelable();
	combined val{ c,cancel };
	std::lock_guard<std::mutex> g{mut};
	callables.push_back(val);
	for(auto & client : list_of_clients)
		client->connect(c);
	return cancel;
}

void Server::write(std::shared_ptr<utilities::MemoryBuffer> buffer) {
	std::lock_guard<std::mutex> g{mut};
	if (list_of_clients.size()==0)
		return;
	list_of_clients.remove_if([buffer](std::shared_ptr<Client>& client){
		if(!client->get_socket().get_underlying_socket().is_open()){
			utilities::cout << "erasing client";
			return true;
		}
		client->write(buffer);
		return false;
	}
	);			
}

void Server::accept() {
	acceptor_.async_accept(
	[this](std::error_code ec, asio::ip::tcp::socket socket) {
	if (!ec) {
		utilities::cout << "Server received a client";
		Client::ServerInfo info{ _cxt,connection_type,std::move(socket) };
		auto client_ptr = std::make_shared<Client>(info);
		std::lock_guard<std::mutex> g{mut};
		for(auto & submitted_callables : callables)
			client_ptr->connect(submitted_callables.lambda);
		list_of_clients.push_back(client_ptr);
		utilities::cout << "Server started listening for new client";
	} else {
		utilities::cout << "Server stopped listening for incoming connections\n";
	}
	accept();
	});
}

}
}
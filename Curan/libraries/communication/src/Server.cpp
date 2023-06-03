#include "communication/Server.h"
#include "communication/Client.h"
#include "utils/Logger.h"

#include <vector>

namespace curan {
namespace communication {

Server::Server(Info& info) : _cxt{ info.io_context }, acceptor_{ _cxt, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), info.port) } {
	accept();
}

Server::~Server() {
	acceptor_.close();
}

std::optional<std::shared_ptr<utilities::Cancelable>> Server::connect(callable c) {
	if (connection_type.index() != c.index())
		return std::nullopt;
	auto cancel = utilities::Cancelable::make_cancelable();
	combined val{ c,cancel };
	callables.push_back(val);
	return cancel;
}

void Server::write(std::shared_ptr<utilities::MemoryBuffer> buffer) {
	if (list_of_clients.size()==0)
		utilities::cout << "No client to write";
	list_of_clients.remove_if(
		[&buffer](std::shared_ptr<Client>& client) {
			if (!client->get_socket().get_underlying_socket().is_open())
				return true;
			utilities::cout << "wrote to client";
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
		list_of_clients.push_back(client_ptr);
	}
	accept();
	});
	utilities::cout << "Server started listening for new client";
}

}
}
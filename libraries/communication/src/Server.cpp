#include "communication/Server.h"
#include "communication/Client.h"

namespace curan {
	namespace communication {
		Server::Server(Info& info) : _cxt{ info.io_context }, acceptor_{ _cxt,info.get_endpoint() } {
				accept();
			}

			Server::~Server() {
				acceptor_.close();
			}

			std::optional<std::shared_ptr<utils::Cancelable>> Server::connect(callable c) {
				if (connection_type.index() != c.index())
					return std::nullopt;
				auto cancel = utils::Cancelable::make_cancelable();
				combined val{ c,cancel };
				callables.push_back(val);
				return cancel;
			}

			void Server::write(std::shared_ptr<curan::utils::MemoryBuffer> buffer) {
				std::cout << "Writing to all clients\n";
				for (auto& client : list_of_clients)
					client->write(buffer);
			}

			void Server::accept() {
				acceptor_.async_accept(
					[this](std::error_code ec, asio::ip::tcp::socket socket) {
						if (!ec) {
							std::cout << "Server received a client\n";
							Client::ServerInfo info{ _cxt,connection_type,std::move(socket) };
							auto client_ptr = std::make_shared<Client>(info);
							list_of_clients.push_back(std::move(client_ptr));
						}
						accept();
					});
				std::cout << "Server started listening for clients\n";
			}
	}
}
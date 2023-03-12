#ifndef CURAN_SERVER_HEADER_FILE_
#define CURAN_SERVER_HEADER_FILE_

#include <asio.hpp>
#include <list>
#include <functional>
#include <memory>

namespace curan {
	namespace communication {
		class Client;

		class Server {
			asio::io_context& _cxt;
			asio::ip::tcp::acceptor acceptor_;

			std::list<std::shared_ptr<Client>> list_of_clients;

			struct combined {
				callable lambda;
				std::shared_ptr<cancelable> canceled;

				combined(callable lambda, std::shared_ptr<cancelable> canceled) : lambda{ lambda }, canceled{ canceled } {}
			};

			std::vector<combined> callables;
			callable connection_type;

		public:
			struct Info {
				asio::io_context& io_context;
				callable connection_type;
				short port;
				Info(asio::io_context& io_context, callable connection_type, short port) :io_context{ io_context }, connection_type{ connection_type }, port{ port }, endpoint{ asio::ip::tcp::v4(),port } {

				}

				asio::ip::tcp::endpoint get_endpoint() {
					return endpoint;
				}
			private:
				asio::ip::tcp::endpoint endpoint;

			};

			Server(Info& info) : _cxt{ info.io_context }, acceptor_{ _cxt,info.get_endpoint() } {
				accept();
			}

			~Server() {
				acceptor_.close();
			}

			[[nodiscard]] std::optional<std::shared_ptr<cancelable>> connect(callable c) {
				if (connection_type.index() != c.index())
					return std::nullopt;
				auto cancel = cancelable::make_cancelable();
				combined val{ c,cancel };
				callables.push_back(val);
				return cancel;
			}

			void write(std::shared_ptr<curan::utils::memory_buffer> buffer) {
				std::cout << "Writing to all clients\n";
				for (auto& client : list_of_clients)
					client->write(buffer);
			}

		private:

			void accept() {
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
		};
	}
}

#endif
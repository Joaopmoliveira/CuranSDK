#ifndef CURAN_SERVER_HEADER_FILE_
#define CURAN_SERVER_HEADER_FILE_

#include <asio.hpp>
#include <list>
#include <functional>
#include <memory>
#include "utils/MemoryUtils.h"
#include "Protocols.h"
#include <optional>
#include "Client.h"

namespace curan {
	namespace communication {
		class Client;

		class Server {
			asio::io_context& _cxt;
			asio::ip::tcp::acceptor acceptor_;
			std::mutex mut;
			std::list<std::shared_ptr<Client>> list_of_clients;

			std::vector<callable> callables;
			callable connection_type;

		public:
			struct Info {
				asio::io_context& io_context;
				callable connection_type;
				unsigned short port;
				Info(asio::io_context& io_context, callable connection_type, unsigned short port) :io_context{ io_context }, connection_type{ connection_type }, port{ port }, endpoint{ asio::ip::tcp::v4(),port } {

				}

				asio::ip::tcp::endpoint get_endpoint() {
					return endpoint;
				}
			private:
				asio::ip::tcp::endpoint endpoint;

			};

			Server(Info& info);

			Server(Info& info,std::function<void(std::error_code ec)> connection_callback);

			~Server();

			void connect(callable c);

			inline size_t number_of_clients(){
				std::lock_guard<std::mutex> g{mut};
				return list_of_clients.size();
			}

			inline void cancel(){
				std::lock_guard<std::mutex> g{mut};
				for (auto& clients : list_of_clients) {
					clients->get_socket().close();
				}
			};

			void write(std::shared_ptr<utilities::MemoryBuffer> buffer);

			inline asio::io_context& get_context(){
				return _cxt;
			}

		private:

			void accept();

			void accept(std::function<void(std::error_code ec)> connection_callback);
		};
	}
}

#endif
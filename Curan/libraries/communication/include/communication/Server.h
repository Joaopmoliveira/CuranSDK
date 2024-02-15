#ifndef CURAN_SERVER_HEADER_FILE_
#define CURAN_SERVER_HEADER_FILE_

#include <asio.hpp>
#include <list>
#include <functional>
#include <memory>
#include "utils/Cancelable.h"
#include "utils/MemoryUtils.h"
#include "Protocols.h"
#include <optional>

namespace curan {
	namespace communication {
		class Client;

		class Server {
			asio::io_context& _cxt;
			asio::ip::tcp::acceptor acceptor_;
			std::mutex mut;
			std::list<std::shared_ptr<Client>> list_of_clients;

			struct combined {
				callable lambda;
				std::shared_ptr<utilities::Cancelable> canceled;

				combined(callable lambda, std::shared_ptr<utilities::Cancelable> canceled) : lambda{ lambda }, canceled{ canceled } {}
			};

			std::vector<combined> callables;
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

			~Server();

			[[nodiscard]] std::optional<std::shared_ptr<utilities::Cancelable>> connect(callable c);

			inline size_t number_of_clients(){
				std::lock_guard<std::mutex> g{mut};
				return list_of_clients.size();
			}

			inline void cancel(){
				std::lock_guard<std::mutex> g{mut};
				list_of_clients = std::list<std::shared_ptr<Client>>{};
			};

			void write(std::shared_ptr<utilities::MemoryBuffer> buffer);

			inline asio::io_context& get_context(){
				return _cxt;
			}

		private:

			void accept();
		};
	}
}

#endif
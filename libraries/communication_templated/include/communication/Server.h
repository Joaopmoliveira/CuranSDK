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
		template<typename Protocol>
			class ServerMine : public std::enable_shared_from_this<ServerMine> {
		public:
    static_assert(std::is_invocable_v<decltype(T::start),std::shared_ptr<Client<T>>>, "the protocol must have a static start() function that receives a templated client");
    static_assert(is_type_complete_v<typename T::signature>, "the protocol must have signature type function that broadcasts the the protocol messages");

			struct Info {
				asio::io_context& io_context;
				unsigned short port;
				Info(asio::io_context& io_context, callable connection_type, unsigned short port) :io_context{ io_context }, connection_type{ connection_type }, port{ port }, endpoint{ asio::ip::tcp::v4(),port } {

				}

				asio::ip::tcp::endpoint get_endpoint() {
					return endpoint;
				}
			private:
				asio::ip::tcp::endpoint endpoint;

			};
		private:
			asio::io_context& _cxt;
			asio::ip::tcp::acceptor acceptor_;
			std::mutex mut;
			std::list<std::shared_ptr<ClientMine<Protocol>>> list_of_clients;

			std::vector<Protocol::signature> callables;

			Server(Info& info);

			Server(Info& info, std::function<bool(std::error_code ec)> connection_callback);

		public:

			template<typename Protocol>
			static inline std::shared_ptr<Server<Protocol>> make(Info& info) {
				std::shared_ptr<Server<Protocol>> server = std::shared_ptr<Server<Protocol>>(new Server<Protocol>{info});
				server->accept();
				return server;
			}

			/*
			The connection callback can be used to refuse incoming connections.
			The method should return a boolean value which indicates if the client 
			should be added to the list of internal clients of the server. If return false
			the client is canceled.
			*/
			static inline std::shared_ptr<Server<Protocol>> make(Info& info, std::function<bool(std::error_code ec)> connection_callback) {
				std::shared_ptr<Server<Protocol>> server = std::shared_ptr<Server<Protocol>>(new Server<Protocol>{ info , connection_callback });
				server->accept(connection_callback);
				return server;
			}

			~Server();

			void connect(Protocol::signature c);

			inline size_t number_of_clients(){
				std::lock_guard<std::mutex> g{mut};
				return list_of_clients.size();
			}

			inline void cancel(){
				std::lock_guard<std::mutex> g{mut};
				for (auto& clients : list_of_clients) {
					clients->get_socket().close();
				}
				list_of_clients.clear();
			};

			inline void close(){
				cancel();
				acceptor_.cancel();
			}

			void write(std::shared_ptr<utilities::MemoryBuffer> buffer);

			inline asio::io_context& get_context(){
				return _cxt;
			}

		private:

			void accept();

			void accept(std::function<bool(std::error_code ec)> connection_callback);
		};


		class Server : public std::enable_shared_from_this<Server> {
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
		private:
			asio::io_context& _cxt;
			asio::ip::tcp::acceptor acceptor_;
			std::mutex mut;
			std::list<std::shared_ptr<Client>> list_of_clients;

			std::vector<callable> callables;
			callable connection_type;

			Server(Info& info);

			Server(Info& info, std::function<bool(std::error_code ec)> connection_callback);

		public:

			static inline std::shared_ptr<Server> make(Info& info) {
				std::shared_ptr<Server> server = std::shared_ptr<Server>(new Server{info});
				server->accept();
				return server;
			}

			/*
			The connection callback can be used to refuse incoming connections.
			The method should return a boolean value which indicates if the client 
			should be added to the list of internal clients of the server. If return false
			the client is canceled.
			*/
			static inline std::shared_ptr<Server> make(Info& info, std::function<bool(std::error_code ec)> connection_callback) {
				std::shared_ptr<Server> server = std::shared_ptr<Server>(new Server{ info , connection_callback });
				server->accept(connection_callback);
				return server;
			}

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
				list_of_clients.clear();
			};

			inline void close(){
				cancel();
				acceptor_.cancel();
			}

			void write(std::shared_ptr<utilities::MemoryBuffer> buffer);

			inline asio::io_context& get_context(){
				return _cxt;
			}

		private:

			void accept();

			void accept(std::function<bool(std::error_code ec)> connection_callback);
		};
	}
}

#endif
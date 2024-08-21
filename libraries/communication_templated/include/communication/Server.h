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

		template<typename protocol>
		class Server : public std::enable_shared_from_this<Server> {
		    static_assert(std::is_invocable_v<decltype(protocol::start),std::shared_ptr<Client<protocol>>>, "the protocol must have a static start() function that receives a templated client");
    		static_assert(is_type_complete_v<typename protocol::signature>, "the protocol must have signature type function that broadcasts the the protocol messages");

		public:
			struct Info {
				asio::io_context& io_context;
				unsigned short port;
				Info(asio::io_context& io_context, unsigned short port) :io_context{ io_context }, connection_type{ connection_type }, port{ port }, endpoint{ asio::ip::tcp::v4(),port } {

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

			Server(Info& info) : _cxt{ info.io_context }, acceptor_{ _cxt, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), info.port) } {

			}

			Server(Info& info,std::function<bool(std::error_code ec)> connection_callback) : _cxt{ info.io_context }, acceptor_{ _cxt, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), info.port) }  {
	
			}

		public:

			static inline std::shared_ptr<Server<protocol>> make(asio::io_context& io_context,unsigned short port) {
				std::shared_ptr<Server<protocol>> server = std::shared_ptr<Server<protocol>>(new Server<protocol>{io_context,port});
				server->accept();
				return server;
			}

			/*
			The connection callback can be used to refuse incoming connections.
			The method should return a boolean value which indicates if the client 
			should be added to the list of internal clients of the server. If return false
			the client is canceled.
			*/
			static inline std::shared_ptr<Server<protocol>> make(asio::io_context& io_context,unsigned short port, std::function<bool(std::error_code ec)> connection_callback) {
				std::shared_ptr<Server<protocol>> server = std::shared_ptr<Server<protocol>>(new Server{ io_context , port , connection_callback });
				server->accept(connection_callback);
				return server;
			}

			~Server() {
	acceptor_.close();
}

			void connect(callable c) {
	if (connection_type.index() != c.index())
		throw std::runtime_error("the supplied callback is not supported against the requested interface");

	std::lock_guard<std::mutex> g{mut};
	callables.push_back(c);
	for(auto & client : list_of_clients)
		client->connect(c);
	return;
}

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

			void write(std::shared_ptr<utilities::MemoryBuffer> buffer) {
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

			inline asio::io_context& get_context(){
				return _cxt;
			}

		private:

			void accept() {
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

			void accept(std::function<bool(std::error_code ec)> connection_callback) {
	acceptor_.async_accept(
	[this,connection_callback, life_time_guaranteer = shared_from_this()](std::error_code ec, asio::ip::tcp::socket socket) {
		if (!ec) {
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
		} else {
			connection_callback(ec);
		}
		
		accept(connection_callback);
	});
}
		};
	}
}

#endif
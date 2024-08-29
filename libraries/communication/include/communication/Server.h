#ifndef CURAN_SERVER_HEADER_FILE_
#define CURAN_SERVER_HEADER_FILE_

#include <asio.hpp>
#include <list>
#include <functional>
#include <memory>
#include "utils/MemoryUtils.h"
#include <optional>
#include "Client.h"
#include "ProtocolValidationHelper.h"

namespace curan
{
	namespace communication
	{

		template <typename protocol>
		class Server : public std::enable_shared_from_this<Server<protocol>>
		{
			static_assert(std::is_invocable_v<decltype(protocol::start), std::shared_ptr<Client<protocol>>>, "the protocol must have a static start() function that receives a templated client");
			static_assert(is_type_complete_v<typename protocol::signature>, "the protocol must have signature type function that broadcasts the the protocol messages");

		public:

		private:
			asio::io_context &_cxt;
			asio::ip::tcp::acceptor acceptor_;
			std::mutex mut;
			std::list<std::shared_ptr<Client<protocol>>> list_of_clients;

			std::vector<typename protocol::signature> callables;

			Server(asio::io_context &io_context, unsigned short port) : _cxt{io_context}, acceptor_{_cxt, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)}
			{
			}

			Server(asio::io_context &io_context, unsigned short port, std::function<bool(std::error_code ec)> connection_callback) : _cxt{io_context}, acceptor_{_cxt, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)}
			{
			}

		public:
			static inline std::shared_ptr<Server<protocol>> make(asio::io_context &io_context, unsigned short port)
			{
				std::shared_ptr<Server<protocol>> server = std::shared_ptr<Server<protocol>>(new Server<protocol>{io_context, port});
				server->accept();
				return server;
			}

			/*
			The connection callback can be used to refuse incoming connections.
			The method should return a boolean value which indicates if the client
			should be added to the list of internal clients of the server. If return false
			the client is canceled.
			*/
			static inline std::shared_ptr<Server<protocol>> make(asio::io_context &io_context, unsigned short port, std::function<bool(std::error_code ec)> connection_callback)
			{
				std::shared_ptr<Server<protocol>> server = std::shared_ptr<Server<protocol>>(new Server{io_context, port, connection_callback});
				server->accept(connection_callback);
				return server;
			}

			~Server()
			{
				acceptor_.close();
			}

			std::shared_ptr<Server<protocol>> copy(){
				return this->shared_from_this();
			}

			void connect(typename protocol::signature c)
			{
				std::lock_guard<std::mutex> g{mut};
				callables.push_back(c);
				for (auto &client : list_of_clients)
					client->connect(c);
				return;
			}

			inline size_t number_of_clients()
			{
				std::lock_guard<std::mutex> g{mut};
				return list_of_clients.size();
			}

			inline void cancel()
			{
				std::lock_guard<std::mutex> g{mut};
				for (auto &clients : list_of_clients)
					clients->get_socket().close();
				list_of_clients.clear();
			};

			inline void close()
			{
				cancel();
				acceptor_.cancel();
			}

			void write(std::shared_ptr<utilities::MemoryBuffer> buffer)
			{
				std::lock_guard<std::mutex> g{mut};
				if (list_of_clients.size() == 0)
					return;
				list_of_clients.remove_if([buffer](std::shared_ptr<Client<protocol>> &client)
										  {
		if(!client->get_socket().sendable())
			return true;
		client->write(buffer);
		return false; });
			}

			inline asio::io_context &get_context()
			{
				return _cxt;
			}

		private:
			void accept()
			{
				acceptor_.async_accept(
					[this, life_time_guaranteer = copy()](std::error_code ec, asio::ip::tcp::socket socket)
					{
						if (!ec)
						{
							auto client_ptr = Client<protocol>::make(_cxt, std::move(socket));
							std::lock_guard<std::mutex> g{mut};
							for (auto &submitted_callables : callables)
								client_ptr->connect(submitted_callables);
							list_of_clients.push_back(client_ptr);
							
						}
						else
						{
							throw std::runtime_error("cannot wait for client");
						}
						accept();
					});
			}

			void accept(std::function<bool(std::error_code ec)> connection_callback)
			{
				acceptor_.async_accept(
					[this, connection_callback, life_time_guaranteer = copy()](std::error_code ec, asio::ip::tcp::socket socket)
					{
						if (!ec)
						{
							auto client_ptr = Client<protocol>::make(_cxt, std::move(socket));
							std::lock_guard<std::mutex> g{mut};
							for (auto &submitted_callables : callables)
								client_ptr->connect(submitted_callables);
							bool all_ok = connection_callback(ec);
							if (all_ok)
								list_of_clients.push_back(client_ptr);
							else
								client_ptr->get_socket().close();
						}
						else
						{
							connection_callback(ec);
						}

						accept(connection_callback);
					});
			}
		};
	}
}

#endif
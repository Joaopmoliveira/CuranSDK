#ifndef CURAN_CLIENT_HEADER_FILE_
#define CURAN_CLIENT_HEADER_FILE_

#include <memory>
#include <asio.hpp>
#include "Socket.h"
#include "utils/Overloading.h"
#include <utility>
#include <optional>

namespace curan {
	namespace communication {
		
		template<typename protocol>
		class Client : public std::enable_shared_from_this<Client> {
		    static_assert(std::is_invocable_v<decltype(protocol::start),std::shared_ptr<Client<protocol>>>, "the protocol must have a static start() function that receives a templated client");
    		static_assert(is_type_complete_v<typename protocol::signature>, "the protocol must have signature type function that broadcasts the the protocol messages");

		public:
			struct Info {
				asio::io_context& io_context;
				asio::ip::tcp::resolver::results_type endpoints;
				Info(asio::io_context& io_context) :io_context{ io_context }{}
			};

			struct ServerInfo {
				asio::io_context& io_context;
				asio::ip::tcp::socket socket;
			};

		private:
			asio::io_context& _cxt;
			curan::communication::Socket socket;

			std::vector<callable> callables;

			std::mutex mut;

			Client(Info& info) : _cxt{ info.io_context },
socket{ _cxt,info.endpoints,info.connection_type},
	connection_type{ info.connection_type } {
};

			template <class _Rep, class _Period>
			Client(Info& info, const std::chrono::duration<_Rep, _Period>& deadline, std::function<void(std::error_code ec)> connection_callback) :
				_cxt{ info.io_context },
				socket{ _cxt,info.endpoints,info.connection_type,deadline,connection_callback },
				connection_type{ info.connection_type } {};

			Client(ServerInfo& info) : _cxt{ info.io_context },
	socket{ _cxt,std::move(info.socket),info.connection_type },
	connection_type{ info.connection_type } {
};

		public:

			~Client(){
				
			}

			static inline std::shared_ptr<Client> make(Info& info) {
				// this is a bad practice, in effect we have a multistage contructor of the socket client
				std::shared_ptr<Client> client = std::shared_ptr<Client>(new Client{info});
				client->socket.trigger_start(info.connection_type,client->weak_from_this(),info.endpoints);
				return client;
			}

			template <class _Rep, class _Period>
			static inline std::shared_ptr<Client> make(Info& info, const std::chrono::duration<_Rep, _Period>& deadline, std::function<void(std::error_code ec)> connection_callback) {
				// this is a bad practice, in effect we have a multistage contructor of the socket client
				std::shared_ptr<Client> client = std::shared_ptr<Client>(new Client{ info,deadline,connection_callback });
				client->socket.trigger_start(info.connection_type, client->weak_from_this(), info.endpoints,deadline, connection_callback);
				return client;
			}

			static inline std::shared_ptr<Client> make(ServerInfo& info) {
				// this is a bad practice, in effect we have a multistage contructor of the socket client
				std::shared_ptr<Client> client = std::shared_ptr<Client>(new Client{ info });
				client->socket.trigger_start(info.connection_type, client->weak_from_this());
				return client;
			}

			inline std::shared_ptr<Client> copy() {
				return shared_from_this();
			}

			void connect(callable c) {
	if (connection_type.index() != c.index())
		throw std::runtime_error("the connected interface does not match the client interface");
	std::lock_guard<std::mutex> g{mut};
	callables.push_back(std::move(c));
	return;
};

			void write(std::shared_ptr<utilities::MemoryBuffer> buffer) {
	std::lock_guard<std::mutex> g{mut};
	socket.post(std::move(buffer));
};

			inline curan::communication::Socket& get_socket() {
				return socket;
			}	

			template<class T, class ... Args>
			void transverse_callables(Args&& ... args) {
				std::lock_guard<std::mutex> g{mut};
				for (auto& listener : callables) {
					if (std::holds_alternative<T>(listener)) {
						auto localinterpretation = std::get<T>(listener);
						localinterpretation(std::forward<decltype(args)>(args)...);
					}
				}

			}
		};
	}
}

#endif
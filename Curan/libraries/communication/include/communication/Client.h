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
		/*
		The client class has two constructors, one which is called
		by the user, because its an handcrafted client and the other
		is the constructor created by the server. Each client must
		be associated with a protocol at compile time. When forwarding this
		protocol, the arguments are prespecified. More on other examples.
		*/
		class Client {
			asio::io_context& _cxt;
			Socket socket;

			std::vector<callable> callables;
			callable connection_type;	

			std::mutex mut;

		public:

			struct Info {
				asio::io_context& io_context;
				callable connection_type;
				asio::ip::tcp::resolver::results_type endpoints;
				Info(asio::io_context& io_context, callable connection_type) :io_context{ io_context }, connection_type{ connection_type } {}
			};

			struct ServerInfo {
				asio::io_context& io_context;
				callable connection_type;
				asio::ip::tcp::socket socket;
			};

			Client(Info& info);

			template <class _Rep, class _Period>
			Client(Info& info,const std::chrono::duration<_Rep, _Period>& deadline,std::function<void(std::error_code ec)> connection_callback) : 
					 _cxt{ info.io_context },
					socket{ _cxt,info.endpoints,info.connection_type,this,deadline,connection_callback},
					connection_type{ info.connection_type } {};

			Client(ServerInfo& info);

			~Client();

			void connect(callable c);

			void write(std::shared_ptr<curan::utilities::MemoryBuffer> buffer);

			inline Socket& get_socket() {
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
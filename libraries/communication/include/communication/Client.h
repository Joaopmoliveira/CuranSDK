#ifndef CURAN_CLIENT_HEADER_FILE_
#define CURAN_CLIENT_HEADER_FILE_

#include <memory>
#include <asio.hpp>
#include "Socket.h"
#include "utils/Cancelable.h"

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

			struct combined {
				callable lambda;
				std::shared_ptr<utils::Cancelable> canceled;
				combined(callable lambda, std::shared_ptr<utils::Cancelable> canceled) : lambda{ lambda }, canceled{ canceled } {}
			};

			std::vector<combined> callables;
			callable connection_type;

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

			Client(ServerInfo& info);

			[[nodiscard]] std::optional<std::shared_ptr<utils::Cancelable>> connect(callable c);

			void write(std::shared_ptr<curan::utils::MemoryBuffer> buffer);

			inline Socket& get_socket() {
				return socket;
			}

			template<class T, class ... Args>
			void transverse_callables(Args&&... args) {
				for (auto& listener : callables) {
					if (listener.canceled->operator()()) {
						auto localinterpretation = std::get<T>(listener.lambda);
						localinterpretation(std::forward<decltype(args)>(args)...);
					}
				}
			}
		};
}
}

#endif
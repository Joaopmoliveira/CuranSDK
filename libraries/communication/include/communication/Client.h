#ifndef CURAN_CLIENT_HEADER_FILE_
#define CURAN_CLIENT_HEADER_FILE_

#include <memory>
#include <asio.hpp>
#include "Socket.h"
#include "utils/Overloading.h"
#include <utility>
#include <optional>
#include "ProtocolValidationHelper.h"

namespace curan
{
	namespace communication
	{
		template <typename protocol>
		class Client : public std::enable_shared_from_this<Client<protocol>>
		{
			static_assert(std::is_invocable_v<decltype(protocol::start), std::shared_ptr<Client<protocol>>>, "the protocol must have a static start() function that receives a templated client");
			static_assert(is_type_complete_v<typename protocol::signature>, "the protocol must have signature type function that broadcasts the the protocol messages");

		public:


		private:
			asio::io_context &_cxt;
			curan::communication::Socket<protocol> socket;

			std::vector<typename protocol::signature> callables;

			std::mutex mut;

			Client(asio::io_context &io_context,asio::ip::tcp::resolver::results_type endpoints) : _cxt{io_context} , socket{ _cxt,endpoints} {};

			template <class _Rep, class _Period>
			Client(asio::io_context &io_context,asio::ip::tcp::resolver::results_type endpoints, const std::chrono::duration<_Rep, _Period> &deadline, std::function<void(std::error_code ec)> connection_callback) : _cxt{io_context},
																																					socket{io_context, endpoints, deadline, connection_callback}
																																				{};

			Client(asio::io_context &io_context,asio::ip::tcp::socket socket) : _cxt{io_context},
									   socket{io_context, std::move(socket)}
									   {
									   };

		public:
			~Client()
			{
			}

			static inline std::shared_ptr<Client<protocol>> make(asio::io_context &io_context,asio::ip::tcp::resolver::results_type endpoints)
			{
				// this is a bad practice, in effect we have a multistage contructor of the socket client
				std::shared_ptr<Client<protocol>> client = std::shared_ptr<Client<protocol>>(new Client<protocol>{io_context,endpoints});
				client->socket.trigger_start(client->weak_from_this(),endpoints);
				return client;
			}

			template <class _Rep, class _Period>
			static inline std::shared_ptr<Client<protocol>> make(asio::io_context &io_context,asio::ip::tcp::resolver::results_type endpoints, const std::chrono::duration<_Rep, _Period> &deadline, std::function<void(std::error_code ec)> connection_callback)
			{
				// this is a bad practice, in effect we have a multistage contructor of the socket client
				std::shared_ptr<Client<protocol>> client = std::shared_ptr<Client<protocol>>(new Client<protocol>{io_context,endpoints, deadline, connection_callback});
				client->socket.trigger_start(client->weak_from_this(), endpoints, deadline, connection_callback);
				return client;
			}

			static inline std::shared_ptr<Client> make(asio::io_context &io_context,asio::ip::tcp::socket socket)
			{
				// this is a bad practice, in effect we have a multistage contructor of the socket client
				std::shared_ptr<Client<protocol>> client = std::shared_ptr<Client<protocol>>(new Client<protocol>{io_context,std::move(socket)});
				client->socket.trigger_start(client->weak_from_this());
				return client;
			}

			inline std::shared_ptr<Client<protocol>> copy()
			{
				return this->shared_from_this();
			}

			void connect(typename protocol::signature c)
			{
				std::lock_guard<std::mutex> g{mut};
				callables.push_back(std::move(c));
				return;
			};

			void write(std::shared_ptr<utilities::MemoryBuffer> buffer)
			{
				std::lock_guard<std::mutex> g{mut};
				socket.post(std::move(buffer));
			};

			inline curan::communication::Socket<protocol> &get_socket()
			{
				return socket;
			}

			template <class... Args>
			void transverse_callables(Args &&...args)
			{
				std::lock_guard<std::mutex> g{mut};
				for (auto &listener : callables)
					listener(std::forward<decltype(args)>(args)...);
			}
		};
	}
}

#endif
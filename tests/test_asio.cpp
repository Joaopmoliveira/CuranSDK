#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#include <windows.h>
#include <asio.hpp>
#include <variant>
#include <map>
#include "sigslot\signal.hpp"
#include "utils\ThreadSafeQueue.h"
#include "utils\Logger.h"

#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlPositionMessage.h"
#include "igtlImageMessage.h"
#include "igtlClientSocket.h"
#include "igtlStatusMessage.h"

#if OpenIGTLink_PROTOCOL_VERSION >= 2
#include "igtlSensorMessage.h"
#include "igtlPointMessage.h"
#include "igtlTrajectoryMessage.h"
#include "igtlStringMessage.h"
#include "igtlTrackingDataMessage.h"
#include "igtlQuaternionTrackingDataMessage.h"
#include "igtlCapabilityMessage.h"
#endif

#include "utils/MemoryUtils.h"

/*
We have three abstractions, the classes that deal with user code,
the classes that deal with protocol implementations, which must 
be developed for specific routines with special logic and the inbetween
which must deal with all the messy details about asio and so forth. 
We achieve this by creating servers and clients, (the classes) that deal
with developers, provide interfaces for listener patterns and so forth.
We have an abstraction of a socket which contains some ASIO logic, and then 
we have the namespace protocols which defines the routines on how we 
read messages from sockets and pass them along to our client/server API.
*/

/*
The socket is an abstraction of
the underlying socket of asio.
*/
class Socket {
	asio::ip::tcp::socket _socket;
	asio::io_context& _cxt;
	std::list<std::shared_ptr<curan::utils::memory_buffer>> to_send;
public:
	Socket(asio::io_context& io_context,
		const asio::ip::tcp::resolver::results_type& endpoints) : _cxt(io_context),
		_socket(io_context) {
		asio::async_connect(_socket, endpoints,
			[this](std::error_code ec, asio::ip::tcp::endpoint e)
			{
				handle_connect(ec, e);
			});
	}

	Socket(asio::io_context& io_context) : _cxt(io_context), _socket{ io_context } {

	}

	~Socket() {
		_socket.close();
	}

	inline asio::ip::tcp::socket& get_underlying_socket() {
		return _socket;
	}

	void handle_connect(std::error_code ec, asio::ip::tcp::endpoint e) {

	}

	void post(std::shared_ptr<curan::utils::memory_buffer> buff) {
		asio::post(_cxt,
			[this, buff]()
			{
				bool write_in_progress = !to_send.empty();
				to_send.push_back(std::move(buff));
				if (!write_in_progress)
				{
					do_write();
				}
			});
	}

	void do_write()
	{
		asio::async_write(get_underlying_socket(),
			asio::buffer(to_send.front()->begin()->data(), to_send.front()->begin()->size()),
			[this](std::error_code ec, std::size_t /*length*/){
				if (!ec){
					to_send.pop_front();
					if (!to_send.empty())
						do_write();
				}
				else
					get_underlying_socket().close();
			});
	}

	void close(){
		asio::post(_cxt, [this]() { get_underlying_socket().close(); });
	}
};

/*
Interface for the openigtlink protocol.
*/
using callable_openigtlink = std::function<void(size_t, std::error_code, igtl::MessageBase::Pointer)>;

/*
This is the most important point, we create a 
variant which contains the signature of the
callable methods.
*/
using callable = std::variant<callable_openigtlink>;

/*
You can query and request for a connection to be 
terminated at any moments notice.
*/
class cancelable : std::enable_shared_from_this<cancelable> {
	std::mutex mut;
	bool is_cancelled;

	cancelable() : is_cancelled{ false } {

	}

public:

	[[nodiscard]] static std::shared_ptr<cancelable> make_cancelable() {
		return std::shared_ptr<cancelable>(new cancelable());
	}

	void cancel() {
		std::lock_guard<std::mutex> g(mut);
		is_cancelled = 0;
	}

	[[nodiscard]] bool operator() () {
		std::lock_guard<std::mutex> g(mut);
		return is_cancelled;
	}
};

class Client {
	asio::io_context& _cxt;
	Socket socket;
	
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

		Info(asio::io_context& io_context, callable connection_type) :io_context{ io_context }, connection_type{ connection_type } {}
	};

	Client(Info& info) : _cxt{ info.io_context }, socket{ _cxt }, connection_type{info.connection_type} {

	}

	[[nodiscard]] std::optional<std::shared_ptr<cancelable>> connect(callable c) {
		if (connection_type.index() != c.index())
			return std::nullopt;
		auto cancel = cancelable::make_cancelable();
		combined val{ c,cancel };
		callables.push_back(val);
		return cancel;
	}

	void write(std::shared_ptr<curan::utils::memory_buffer> buffer){
		socket.post(std::move(buffer));
	}

	Socket& get_socket() {
		return socket;
	}
};

class Server {
	asio::io_context& _cxt;
	Socket socket;
	std::vector<Client> list_of_clients;
	
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

		Info(asio::io_context& io_context, callable connection_type) :io_context{ io_context }, connection_type{ connection_type } {}
	};

	Server(Info& info) : _cxt{ info.io_context }, socket{ _cxt } {

	}

	[[nodiscard]] std::optional<std::shared_ptr<cancelable>> connect(callable c) {
		if (connection_type.index() != c.index())
			return std::nullopt;
		auto cancel = cancelable::make_cancelable();
		combined val{c,cancel };
		callables.push_back(val);
		return cancel;
	}

	void write(std::shared_ptr<curan::utils::memory_buffer> buffer) {
		socket.post(std::move(buffer));
	}
};

namespace protocols {
	namespace igtlink {
		namespace implementation {
			struct carry_on_luggage {
				igtl::MessageBase::Pointer message_to_receive;
				igtl::MessageBase::Pointer header_to_receive;
				Client* owner = nullptr;

				carry_on_luggage(Client* supplied_owner) {
					header_to_receive->AllocatePack();
					owner = supplied_owner;
				}
			};

			void read_header(carry_on_luggage val) {
				asio::async_read(val.owner->get_socket().get_underlying_socket(),
					asio::buffer(val.header_to_receive->GetBufferPointer(), val.header_to_receive->GetBufferSize()),
					[val](std::error_code ec, std::size_t len)
					{
						if (!ec && val.header_to_receive->Unpack())
							read_body(std::move(val));
						else
							val.owner->get_socket().get_underlying_socket().close();
					});
			}

			void read_body(carry_on_luggage val) {
				val.message_to_receive->SetMessageHeader(val.header_to_receive);
				val.message_to_receive->AllocatePack();
				asio::async_read(val.owner->get_socket().get_underlying_socket(),
					asio::buffer(val.message_to_receive->GetBufferPointer(), val.message_to_receive->GetBufferSize()),
					[val](std::error_code ec, std::size_t len)
					{
						if (!ec && val.header_to_receive->Unpack())
							read_header(std::move(val));
						else
							val.owner->get_socket().get_underlying_socket().close();
					});
			}
		}

		void start(Client* client_pointer) {
			implementation::carry_on_luggage val{ client_pointer };
			val.owner = client_pointer;
			implementation::read_header(std::move(val));
		}


	};
}

/*
Launch a server thread which waits for a 
connection and talks through the openIGTlink
protocol
*/
void foo(asio::io_context& cxt) {
	callable_openigtlink igtlink_interface;
	Server::Info construction{ cxt,igtlink_interface };
	Server server{ construction };
	for (size_t message_counter = 0; message_counter < 20; ++message_counter) {
		
	}
}

/*
Launch a client which connects itself to 
the server running inside foo.
*/
void bar(asio::io_context& cxt) {
	callable_openigtlink igtlink_interface;
	Client::Info construction{ cxt,igtlink_interface };
	Client client{ construction };
}

int main() {
	asio::io_context io_context;

	return 0;
}
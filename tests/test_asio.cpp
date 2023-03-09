#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#include <windows.h>
#include <asio.hpp>
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

namespace protocols {
	using Protocol = std::function<int(void)>;
	namespace igtlink {
		void start(std::shared_ptr<curan::utils::memory_buffer>) {

			//read header of protocol

			asio::async_read(socket_,
				asio::buffer(read_msg_.data(), chat_message::header_length),
				[this](std::error_code ec, std::size_t /*length*/)
				{
					if (!ec && read_msg_.decode_header())
					{
						do_read_body();
					}
					else
					{
						socket_.close();
					}
				});
		}

		void end() {

			//read body of protocol

			if (!error)
			{
				asio::async_read(socket_,
					asio::buffer(read_msg_.data(), chat_message::header_length),
					boost::bind(&chat_client::handle_read_header, this,
						asio::placeholders::error));
			}
			else
			{
				do_close();
			}
		}
	};
}

/*
The socket is an abstraction of
the underlying socket of asio. 
*/
class Socket {
	asio::ip::tcp::socket _socket;
	asio::io_context& _cxt;

public:
	Socket(asio::io_context& io_context,
		const asio::ip::tcp::resolver::results_type& endpoints) : _cxt(io_context),
		_socket(io_context) {
		asio::async_connect(_socket, endpoints,
			[this](std::error_code ec, asio::ip::tcp::endpoint e)
			{
				handle_connect(ec,e);
			});
	}

	~Socket() {
	
	}

	void handle_connect(std::error_code ec, asio::ip::tcp::endpoint e) {
	
	}
	void close()
	{
		asio::post(_cxt, [this]() { _socket.close(); });
	}
};

class Client {
	asio::io_context& _cxt;
	Socket socket;

	Client(asio::io_context& io_context) : _cxt{ io_context } : {

	}
};

class Server {
	asio::io_context& _cxt;
	Socket socket;
	std::vector<Client> list_of_clients;

	Server(asio::io_context& io_context) : _cxt{ io_context } : {

	}
};

/*
Launch a server thread which waits for a 
connection and talks through the openIGTlink
protocol
*/
void foo() {

}

/*
Launch a client which connects itself to 
the server running inside foo.
*/
void bar() {

}

int main() {


	return 0;
}


/*
This example shows how the constructors of the servers and the clients
submit themselfs to the server managers and the client managers, such that we can
keep track of the current open connections. This allows us to display on the
UI information about the current status of the application.
Another important concept in these examples is the actual protocol
implementation is separated from the client socket. This means that to acess
the internals of the socket you need to cast the pointer of the socket into
the protocol you desire and from there you need to use the api of that particular
protocol implementation. This allows us to use more than one protocol for communicating
with the outside world.
*/

/*

asio::io_context* get_io_context() {
	static asio::io_context cxt;
	return &cxt;
}

enum class ProtocolImplmentation {
	OPEN_IGT_LINK,
	NONE
};
*/

/*
You can implement the state machine of asio as you wish, but you need to suply two important
functions, how to handle a connection and how to handle a
*/
/*
struct ProtocolInfo {
	ProtocolImplmentation implementation = ProtocolImplmentation::NONE;
};
*/
/*
All protocols should have a callable signal
which emits the message just received to all
the listeners.
*/
/*

class Session : public std::enable_shared_from_this<Session> {
protected:

	size_t identifier = 0;
	asio::ip::tcp::socket _socket;
	asio::io_context& _cxt;

	ProtocolImplmentation underlying_protocol = ProtocolImplmentation::NONE;

	virtual void async_handle_connect(const asio::error_code& error, asio::ip::tcp::endpoint const& endpoint) {

	};

public:

	Session(asio::io_context& cxt, asio::ip::tcp::socket&& socket, ProtocolImplmentation implem) : _cxt{ cxt }, _socket{ std::move(socket) }, underlying_protocol{ implem }{

	}

	Session(asio::io_context& cxt, const asio::ip::tcp::resolver::results_type& results, ProtocolImplmentation implem) : _cxt{ cxt }, _socket(cxt), underlying_protocol{ implem }
	{

	};

	virtual ~Session() {
	}

	virtual void close() {

	};

	inline ProtocolImplmentation protocol() {
		return underlying_protocol;
	}

	inline size_t identification() {
		return identifier;
	}
};

class ProtoOpenIGTLink : public Session {


	void async_handle_header(const asio::error_code& error) {
		if (error) {	
			curan::utils::console->info("handle async header: error-"+ error.message());
			close();

			return;
		}
		message_pointer = igtl::MessageBase::New();
		message_pointer->SetMessageHeader(message_header);
		message_pointer->AllocatePack();
		asio::async_read(_socket,
			asio::buffer(message_header->GetPackPointer(), (size_t)message_header->GetBodySizeToRead()),
			std::bind(&ProtoOpenIGTLink::async_handle_body, getptr(),
				std::placeholders::_1));
	}

	void async_handle_body(const asio::error_code& error) {
		if (error) {
			curan::utils::console->info("handle async body: error-" + error.message());
			close();
			return;
		}
		callable(message_pointer);
		asio::async_read(_socket,
			asio::buffer(message_header->GetPackPointer(), (size_t)message_header->GetPackSize()),
			std::bind(&ProtoOpenIGTLink::async_handle_header, getptr(),
				std::placeholders::_1));
	}

	void async_handle_write(const asio::error_code& error)
	{
		if (error) {
			curan::utils::console->info("handle async write: error-" + error.message());
			close();
			return;
		}

		std::lock_guard<std::mutex> g(mut);
		list_of_messages_to_send.erase(list_of_messages_to_send.begin());
		bool is_empty = list_of_messages_to_send.empty();
		if (!is_empty){ //if we still have more messages to continue sending, we post work to the queue
			asio::const_buffer buffer{ (*list_of_messages_to_send.begin())->GetBufferPointer(),(size_t)(*list_of_messages_to_send.begin())->GetBufferSize() };
			asio::async_write(_socket,
				buffer,
				std::bind(&ProtoOpenIGTLink::async_handle_write, getptr(),
					std::placeholders::_1));
		}
	}

	igtl::MessageHeader::Pointer message_header;
	igtl::MessageBase::Pointer message_pointer;
	std::mutex mut;
	std::list<igtl::MessageHeader::Pointer> list_of_messages_to_send;

	void async_handle_connect(const asio::error_code& error, asio::ip::tcp::endpoint const& endpoint) override {
		if (error) {
			curan::utils::console->info("handle connect: error-" + error.message());
			close();
			return;
		}
		auto pt = getptr();
		asio::async_read(_socket,
			asio::buffer(message_header->GetPackPointer(), (size_t)message_header->GetPackSize()),
			std::bind(&ProtoOpenIGTLink::async_handle_header, getptr(),std::placeholders::_1));
	};

	ProtoOpenIGTLink(asio::io_context& cxt, const asio::ip::tcp::resolver::results_type& results) : Session{ cxt,results,ProtocolImplmentation::OPEN_IGT_LINK } {
		curan::utils::console->info("created a client");
		message_header = igtl::MessageHeader::New();
		message_header->InitPack();
	}


	ProtoOpenIGTLink(asio::io_context& cxt, asio::ip::tcp::socket&& socket_) : Session{ cxt,std::move(socket_),ProtocolImplmentation::OPEN_IGT_LINK } {
		curan::utils::console->info("created a client");
		message_header = igtl::MessageHeader::New();
		message_header->InitPack();
	}

public:

	static std::shared_ptr<ProtoOpenIGTLink> make_shared(asio::io_context& cxt, asio::ip::tcp::socket&& socket_) {
		auto session = std::shared_ptr<ProtoOpenIGTLink>(new ProtoOpenIGTLink(cxt, std::move(socket_)));
		asio::async_read(session->_socket,
			asio::buffer(session->message_header->GetPackPointer(), (size_t)session->message_header->GetPackSize()),
			std::bind(&ProtoOpenIGTLink::async_handle_header, session,
				std::placeholders::_1));
		return session;
	}

	static std::shared_ptr<ProtoOpenIGTLink> make_shared(asio::io_context& cxt, const asio::ip::tcp::resolver::results_type& results) {
		auto session = std::shared_ptr<ProtoOpenIGTLink>(new ProtoOpenIGTLink(cxt, results));
		asio::async_connect(session->_socket, results,
			std::bind(&ProtoOpenIGTLink::async_handle_connect,
				session, std::placeholders::_1, std::placeholders::_2));
		return session;
	}

	std::shared_ptr<ProtoOpenIGTLink> getptr() {
		return std::static_pointer_cast<ProtoOpenIGTLink>(shared_from_this());
	}

	sigslot::signal<igtl::MessageBase::Pointer> callable;

	~ProtoOpenIGTLink() override {
		close();
	}

	void async_write(igtl::MessageBase::Pointer msg)
	{
		std::lock_guard<std::mutex> g(mut);
		bool is_empty = list_of_messages_to_send.empty();
		list_of_messages_to_send.push_back(msg);
		if (is_empty) { //if the previous size is zero, then no message is currently being processed
			std::list<igtl::MessageBase::Pointer>::iterator it = list_of_messages_to_send.begin();
			asio::const_buffer buffer{ (*list_of_messages_to_send.begin())->GetBufferPointer(),(size_t)(*list_of_messages_to_send.begin())->GetBufferSize() };
			asio::async_write(_socket,
				buffer,
				std::bind(&ProtoOpenIGTLink::async_handle_write, this,
					std::placeholders::_1));
		}
	}

	void close() override {
		_socket.close();
		curan::utils::console->info("closed the session and the associated socket");
	}
};
*/
/*
The session manager cannot be instantiated by the user
since its supposed to always be queried by the get_session_manager
function. This guarantees that we only have a single object
of this class.
*/
/*
class SessionManager {
	std::map<std::string, std::shared_ptr<Session>> sessions;

	SessionManager();

public:
	static SessionManager* get_manager() {
		static SessionManager manager;
		return &manager;
	}
};

class Server;
*/
/*
The server manager cannot be instantiated by the user
since its supposed to always be queried by the get_server_manager
function. This guarantees that we only have a single object
of this class.
*/
/*
class ServerManager {
	std::mutex mut;
	std::map<size_t, std::shared_ptr<Server>> servers;

	ServerManager() {
	}

	friend class Server;

	bool add_server(std::shared_ptr<Server> server) {
		std::lock_guard<std::mutex> g(mut);
		static size_t local = 0;
		++local;
		auto const pair = servers.try_emplace(local, server);
		return pair.second;
	};
public:

	static ServerManager* get_manager() {
		static ServerManager manager;
		return &manager;
	}


};

class Server : public std::enable_shared_from_this<Server>
{
	size_t identifier;
	bool is_closed = true;
	ProtocolImplmentation implementation;

	Server(asio::io_context& io_context, short port, ProtocolInfo& info)
		: io_context_(io_context), acceptor_(io_context, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)),
		socket_(io_context), implementation{info.implementation}
	{
		curan::utils::console->info("the server has benn lauched");
		do_accept();
	}
public:

	sigslot::signal<std::shared_ptr<Session>> callable;

	~Server() {
		stop_server();
	}

	[[nodiscard]] static std::shared_ptr<Server> make_shared(asio::io_context& io_context, short port, ProtocolInfo& info) {
		ServerManager* manager = ServerManager::get_manager();
		auto server = std::shared_ptr<Server>(new Server(io_context, port, info));
		if (manager->add_server(server))
			return server;
		return nullptr;
	}

	[[nodiscard]] std::shared_ptr<Server> shared_copy() {
		return shared_from_this();
	}

	[[nodiscard]] asio::ip::tcp::endpoint get_endpoint() {
		return acceptor_.local_endpoint();
	}

private:

	std::shared_ptr<Server> getptr() {
		return shared_from_this();
	}

	void do_accept()
	{
		acceptor_.async_accept(socket_,
			[this](std::error_code ec)
			{
				curan::utils::console->info("received a client request");
				if (ec) {
					stop_server();
					return;
				}
				switch (implementation) {
				case ProtocolImplmentation::OPEN_IGT_LINK:
				{
					auto ptr = ProtoOpenIGTLink::make_shared(io_context_, std::move(socket_));
					callable(std::dynamic_pointer_cast<Session>(ptr));
				}
				break;
				default:
					throw std::exception("no protocol was specified, therefore cannot create the correct server");
					break;
				}
				do_accept();
			});
	}

	void stop_server() {
		if (acceptor_.is_open())
			acceptor_.close();
		callable.disconnect_all();
		curan::utils::console->info("closed the server and the associated socket");
	}



	asio::ip::tcp::acceptor acceptor_;
	asio::ip::tcp::socket socket_;
	asio::io_context& io_context_;

};
*/

/*
function that deals messages received from the clients connected to the
*/
/*
void function2(igtl::MessageBase::Pointer message) {
	curan::utils::console->info("received a message from the accepted client socket");
}
*/
/*
function that deals with received connections
*/
/*
void function1(std::shared_ptr<Session> received_session) {
	curan::utils::console->info("received a from the callable session");

	switch (received_session->protocol()) {
	case ProtocolImplmentation::OPEN_IGT_LINK:
	{
		curan::utils::console->info("trying to lauch signal");
		std::shared_ptr<ProtoOpenIGTLink> converted = std::dynamic_pointer_cast<ProtoOpenIGTLink>(received_session);
		converted->callable.connect(function2);
		curan::utils::console->info("lauched signal");
	}
	break;
	default:

		break;
	}
}
*/

//------------------------------------------------------------
// Function to generate random matrix.
/*
void GetRandomTestMatrix(igtl::Matrix4x4& matrix)
{
	float position[3];
	float orientation[4];

	// random position
	static float phi = 0.0;
	position[0] = 50.0 * cos(phi);
	position[1] = 50.0 * sin(phi);
	position[2] = 50.0 * cos(phi);
	phi = phi + 0.2;

	// random orientation
	static float theta = 0.0;
	orientation[0] = 0.0;
	orientation[1] = 0.6666666666 * cos(theta);
	orientation[2] = 0.577350269189626;
	orientation[3] = 0.6666666666 * sin(theta);
	theta = theta + 0.1;

	//igtl::Matrix4x4 matrix;
	igtl::QuaternionToMatrix(orientation, matrix);

	matrix[0][3] = position[0];
	matrix[1][3] = position[1];
	matrix[2][3] = position[2];
}


int main()
{
	try {
		auto communication_blocking_thread = []() {
			try {
				curan::utils::console->info("lauching implicit strand");
				asio::executor_work_guard<asio::io_context::executor_type> word_guard{ get_io_context()->get_executor() };
				size_t number_of_iters = 0;
				while (!get_io_context()->stopped()) {
					number_of_iters++;
					get_io_context()->run_one();
					curan::utils::console->info("ran %f times",number_of_iters);
				}
				curan::utils::console->info("terminated strand");
			}
			catch (std::exception& e) {
				curan::utils::console->info("communication thread:"+std::string(e.what()));
			}
		};

		ProtocolInfo info{ ProtocolImplmentation::OPEN_IGT_LINK };
		short port = 50000;
		auto serv = Server::make_shared(*get_io_context(), port, info);
		serv->callable.connect(function1);

		//now we lauch the thread which actually contains the number we are interested in
		std::thread communication_thread{ communication_blocking_thread };

		asio::error_code error;
		asio::ip::tcp::resolver resolver(*get_io_context());
		asio::ip::tcp::resolver::results_type results = resolver.resolve("localhost", std::to_string(port), error);
		
		for (const asio::ip::tcp::endpoint& endpoint : results)
		{
			std::cout << endpoint << "\n";
		}
		
		//now we can create a session which we will read into 
		auto sess = ProtoOpenIGTLink::make_shared(*get_io_context(), results);

		igtl::TransformMessage::Pointer transMsg;
		transMsg = igtl::TransformMessage::New();
		transMsg->SetDeviceName("Tracker");

		igtl::TimeStamp::Pointer ts;
		ts = igtl::TimeStamp::New();
		igtl::Matrix4x4 matrix;
		GetRandomTestMatrix(matrix);
		ts->GetTime();
		transMsg->SetMatrix(matrix);
		transMsg->SetTimeStamp(ts);
		transMsg->Pack();
		igtl::MessageBase::Pointer temp = igtl::MessageBase::New();
		std::cout << temp->Copy(transMsg);
		std::string s = transMsg->GetMessageType();
		curan::utils::console->info(s);
		sess->async_write(temp);
		get_io_context()->stop();
		communication_thread.join();
	}
	catch (std::exception& e) {
		curan::utils::console->info("main: "+std::string(e.what()));
		return 1;
	}
	return 0;
}

*/
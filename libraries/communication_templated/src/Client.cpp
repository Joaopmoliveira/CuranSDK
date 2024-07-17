#include "communication/Client.h"

namespace curan {
namespace communication {
		
Client::Client(Info& info) : _cxt{ info.io_context },
socket{ _cxt,info.endpoints,info.connection_type},
	connection_type{ info.connection_type } {
};

Client::Client(ServerInfo& info) : _cxt{ info.io_context },
	socket{ _cxt,std::move(info.socket),info.connection_type },
	connection_type{ info.connection_type } {
};

Client::~Client(){
}

void Client::connect(callable c) {
	if (connection_type.index() != c.index())
		throw std::runtime_error("the connected interface does not match the client interface");
	std::lock_guard<std::mutex> g{mut};
	callables.push_back(std::move(c));
	return;
};

void Client::write(std::shared_ptr<utilities::MemoryBuffer> buffer) {
	std::lock_guard<std::mutex> g{mut};
	socket.post(std::move(buffer));
};

}
}
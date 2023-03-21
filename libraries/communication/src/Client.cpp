#include "communication/Client.h"

namespace curan {
namespace communication {
		
Client::Client(Info& info) : _cxt{ info.io_context },
	socket{ _cxt,info.endpoints,info.connection_type,this },
	connection_type{ info.connection_type } {
};

Client::Client(ServerInfo& info) : _cxt{ info.io_context },
	socket{ _cxt,std::move(info.socket),info.connection_type,this },
	connection_type{ info.connection_type } {
};

std::optional<std::shared_ptr<utils::Cancelable>> Client::connect(callable c) {
	if (connection_type.index() != c.index())
		return std::nullopt;
	auto cancel = utils::Cancelable::make_cancelable();
	combined val{ c,cancel };
	callables.push_back(std::move(val));
	return cancel;
};

void Client::write(std::shared_ptr<curan::utils::MemoryBuffer> buffer) {
	socket.post(std::move(buffer));
};

}
}
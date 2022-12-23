#ifndef CURAN_OPENIGTLINKPROTOCOL_HEADER_FILE_
#define CURAN_OPENIGTLINKPROTOCOL_HEADER_FILE_
#include "Protocol.h"
#include "utils/Callable.h"

namespace curan {
	namespace communication {

		/*
		The protocol class is the parent class of all protocol implementations.
		For example, assume that you wish to implement a specific protocol, like
		openigtlink. To do this your class should derive from the protocol
		implementation, after which your protocol implementation will be called with the correct
		asio context.
		*/
		class OpenIGTLinkProtocol : public Protocol , utils::Callable<int> {
			void async_send() = 0;
			void async_receive() = 0;
		};
	}
}

#endif
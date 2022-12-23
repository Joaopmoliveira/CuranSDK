#ifndef CURAN_PROTOCOL_HEADER_FILE_
#define CURAN_PROTOCOL_HEADER_FILE_
#include <asio.hpp>
#include "utils/TheadSafeQueue.h"
#include "utils/Callable.h"

namespace curan {
	namespace communication {

		enum ProtocolImplementation {
			OPEN_IGT_LINK,
		};

		/*
		The protocol class is the parent class of all protocol implementations.
		For example, assume that you wish to implement a specific protocol, like
		openigtlink. To do this your class should derive from the protocol
		implementation, after which your protocol implementation will be called with the correct
		asio context.
		*/
		class Protocol{
			ProtocolImplementation implementation;

			Protocol(ProtocolImplementation implementation);
			void async_send() = 0;
			void async_receive() = 0;

			inline ProtocolImplementation getimplementation() {
				return implementation;
			};
		};
	}
}

#endif
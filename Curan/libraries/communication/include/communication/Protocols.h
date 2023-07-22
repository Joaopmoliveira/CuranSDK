#ifndef CURAN_PROTOCOLS_HEADER_FILE_
#define CURAN_PROTOCOLS_HEADER_FILE_

#include <asio.hpp>
#include <list>
#include <functional>
#include <memory>
#include <variant>

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

#include "customprotocols/FRIContent.h"

namespace curan {
	namespace communication {
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

		class Client;

		/*
		Interface for the openigtlink protocol.
		*/
		using interface_igtl = std::function<void(const size_t&, const std::error_code&, igtl::MessageBase::Pointer)>;
		using interface_fri = std::function<void(const size_t&, const std::error_code&, FRIMessage)>;
		using interface_empty = std::function<void(void)>;

		/*
		This is the most important point, we create a
		variant which contains the signature of the
		callable methods.
		*/
		using callable = std::variant<interface_empty,interface_igtl,interface_fri>;

		/*
		This function selects which protocol we want to communicate with depending
		on which type is contained inside the variant. 
		*/
		std::function<void(Client*)> get_interface(callable callable_type);
	}
}

#endif



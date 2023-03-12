#ifndef CURAN_MEMORY_UTILS_HEADER_FILE_
#define CURAN_MEMORY_UTILS_HEADER_FILE_

#include <asio.hpp>

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

namespace curan {
	namespace communication {
		namespace protocols {
			namespace igtlink {
				enum status {
					OK = 0,
					FAILED_TO_UNPACK_HEADER,
					FAILED_TO_UNPACK_BODY
				};

				class Client;

				namespace implementation {

					struct IgtlinkClientConnection {
						igtl::MessageBase::Pointer message_to_receive;
						igtl::MessageBase::Pointer header_to_receive;
						Client* owner;
						IgtlinkClientConnection(Client* supplied_owner);
					};

					void read_header_first_time(IgtlinkClientConnection val);
					void read_body(IgtlinkClientConnection val, std::error_code ec);
					void read_header(IgtlinkClientConnection val, std::error_code ec);
				}

				void start(Client* client_pointer);
			};
		}
	}
}


#endif 
#ifndef CURAN_PROTOIGTL_HEADER_FILE_
#define CURAN_PROTOIGTL_HEADER_FILE_

#include <asio.hpp>

#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlPositionMessage.h"
#include "igtlImageMessage.h"
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


		template<typename protocol>
		class Client;

		namespace protocols {

			struct igtlink;

		namespace igtlinkimplementation {
				enum status {
					OK = 0,
					FAILED_TO_UNPACK_HEADER,
					FAILED_TO_UNPACK_BODY
				};

				struct IgtlinkClientConnection {
						igtl::MessageBase::Pointer message_to_receive;
						igtl::MessageHeader::Pointer header_to_receive;
						std::shared_ptr<Client<igtlink>> owner;
						IgtlinkClientConnection(std::shared_ptr<Client<igtlink>> supplied_owner);
				};
		}

			struct igtlink{
				using signature = std::function<void(const size_t&, const std::error_code&, igtl::MessageBase::Pointer)>;

				static void read_header_first_time(igtlinkimplementation::IgtlinkClientConnection val);
				static void read_body(igtlinkimplementation::IgtlinkClientConnection val, std::error_code ec);
				static void read_header(igtlinkimplementation::IgtlinkClientConnection val, std::error_code ec);
				static void start(std::shared_ptr<Client<igtlink>> client_pointer);
			};
		}
	}
}


#endif 
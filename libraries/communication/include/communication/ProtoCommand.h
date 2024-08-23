#ifndef CURAN_PROTOFRI_HEADER_FILE_
#define CURAN_PROTOFRI_HEADER_FILE_

#include <asio.hpp>
#include "ProtocolValidationHelper.h"
#include "customprotocols/RobotCommand.h"

namespace curan {
	namespace communication {

		template<typename protocol>
		class Client;

		namespace protocols {

		struct robotcommand;

		namespace robotcommandmplementation {
				enum status {
					OK = 0,
					FAILED_TO_UNPACK_HEADER,
					FAILED_TO_UNPACK_BODY
				};

				struct RobotClientConnection {
					std::shared_ptr<RobotCommand> message = nullptr;
					std::shared_ptr<Client<robotcommand>> owner;
					RobotClientConnection(std::shared_ptr<Client<robotcommand>> supplied_owner);
				};
		}

		struct robotcommand{
   	 		public:
			using signature = std::function<void(const size_t&, const std::error_code&, std::shared_ptr<RobotCommand>)>;
			
			static void start(std::shared_ptr<Client<robotcommand>> client);
			void static read_header_first_time(robotcommandmplementation::RobotClientConnection val);
			void static read_body(robotcommandmplementation::RobotClientConnection val, std::error_code ec);
			void static read_header(robotcommandmplementation::RobotClientConnection val, std::error_code ec);
		};

		}
	}
}


#endif 
#ifndef CURAN_PROTOFRI_HEADER_FILE_
#define CURAN_PROTOFRI_HEADER_FILE_

#include <asio.hpp>
#include "ProtocolValidationHelper.h"
#include "customprotocols/FRIContent.h"

namespace curan {
	namespace communication {

		template<typename protocol>
		class Client;

		namespace protocols {

		struct fri;

		namespace friimplementation {
				enum status {
					OK = 0,
					FAILED_TO_UNPACK_HEADER,
					FAILED_TO_UNPACK_BODY
				};

				struct FRIClientConnection {
					std::shared_ptr<FRIMessage> message = nullptr;
					std::shared_ptr<Client<fri>> owner;
					FRIClientConnection(std::shared_ptr<Client<fri>> supplied_owner);
				};
		}

		struct fri{
   	 		public:
			using signature = std::function<void(const size_t&, const std::error_code&, std::shared_ptr<FRIMessage>)>;
			
			static void start(std::shared_ptr<Client<fri>> client);
			void static read_header_first_time(friimplementation::FRIClientConnection val);
			void static read_body(friimplementation::FRIClientConnection val, std::error_code ec);
			void static read_header(friimplementation::FRIClientConnection val, std::error_code ec);
		};

		}
	}
}


#endif 
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

		struct frimessage{
				enum status {
					OK = 0,
					FAILED_TO_UNPACK_HEADER,
					FAILED_TO_UNPACK_BODY
				};

				struct FRIClientConnection {
					std::shared_ptr<FRIMessage> message = nullptr;
					std::shared_ptr<Client<frimessage>> owner;
					FRIClientConnection(std::shared_ptr<Client<frimessage>> supplied_owner);
				};

    		using signature = std::function<void(const size_t&, const std::error_code&, std::string_view value)>;

			void static start(std::shared_ptr<Client<frimessage>> client);
			void static read_header_first_time(FRIClientConnection val);
			void static read_body(FRIClientConnection val, std::error_code ec);
			void static read_header(FRIClientConnection val, std::error_code ec);
		};
		}
	}
}


#endif 
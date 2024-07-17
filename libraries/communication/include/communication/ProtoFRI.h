#ifndef CURAN_PROTOFRI_HEADER_FILE_
#define CURAN_PROTOFRI_HEADER_FILE_

#include <asio.hpp>

#include "customprotocols/FRIContent.h"

namespace curan {
	namespace communication {

		class Client;


		

		namespace protocols {
			namespace frimessage {
				enum status {
					OK = 0,
					FAILED_TO_UNPACK_HEADER,
					FAILED_TO_UNPACK_BODY
				};

				namespace implementation {

					struct FRIClientConnection {
						std::shared_ptr<FRIMessage> message = nullptr;
						std::shared_ptr<Client> owner;
						FRIClientConnection(std::shared_ptr<Client> supplied_owner);
					};

					void read_header_first_time(FRIClientConnection val);
					void read_body(FRIClientConnection val, std::error_code ec);
					void read_header(FRIClientConnection val, std::error_code ec);
				}

				void start(std::shared_ptr<Client> client_pointer);
			};
		}
	}
}


#endif 
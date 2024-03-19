#ifndef CURAN_PROTO_PROC_HANDLER_HEADER_FILE_
#define CURAN_PROTO_PROC_HANDLER_HEADER_FILE_

#include <asio.hpp>

#include "customprotocols/ProcessHandler.h"

namespace curan {
	namespace communication {

		class Client;

		namespace protocols {
			namespace proc_handler_message {
                
				enum status {
					OK = 0,
					FAILED_TO_UNPACK_HEADER,
					FAILED_TO_UNPACK_BODY
				};

				namespace implementation {

					struct HandlerClientConnection {
						std::shared_ptr<ProcessHandler> message = nullptr;
						Client* owner;
						HandlerClientConnection(Client* supplied_owner);
					};

					void read_header_first_time(HandlerClientConnection val);
					void read_body(HandlerClientConnection val, std::error_code ec);
					void read_header(HandlerClientConnection val, std::error_code ec);
				}

				void start(Client* client_pointer);
			};
		}
	}
}


#endif 
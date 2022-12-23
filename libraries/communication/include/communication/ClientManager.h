#ifndef CURAN_SERVERMANAGER_HEADER_FILE_
#define CURAN_SERVERMANAGER_HEADER_FILE_
#include <map>
#include "Protocol.h"
#include "Client.h"

namespace curan {
	namespace communication {

		/*
		Because conceptually a server does different things from a
		client we separate the two entities. The server manager contains a list of
		all the current servers which are active.
		*/
		class ClientManager {
			std::map<size_t, Client> server_list;

			ClientManager();
		public:
			static ClientManager* getmanager();

			void create_client(std::shared_ptr<Protocol> protocol_implementation);
		};
	}
}

#endif
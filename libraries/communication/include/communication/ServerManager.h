#ifndef CURAN_SERVERMANAGER_HEADER_FILE_
#define CURAN_SERVERMANAGER_HEADER_FILE_
#include <map>
#include "Protocol.h"

namespace curan {
	namespace communication {

		/*
		Because conceptually a server does different things from a
		client we separate the two entities. The server manager contains a list of
		all the current servers which are active.
		*/
		class ServerManager {
			std::map<size_t, Server> server_list;

			ServerManager();
		public:
			static ServerManager* getmanager();

			void create_server(std::shared_ptr<Protocol> protocol_implementation);
		};
	}
}

#endif


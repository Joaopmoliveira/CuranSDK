#ifndef CURAN_CLIENT_HEADER_FILE_
#define CURAN_CLIENT_HEADER_FILE_

#include "Protocol.h"

namespace curan {
	namespace communication {
		/*
		A client will have a particular protocol which it points to.
		The protocol is a thing which is given to  
		*/
		class Client {
			std::shared_ptr<Protocol> prot = nullptr;
		};
	}
}

#endif
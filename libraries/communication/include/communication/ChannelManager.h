#ifndef CURAN_CHANNELMANAGER_HEADER_FILE_
#define CURAN_CHANNELMANAGER_HEADER_FILE_

#include "ProcessorOIGTL.h"
#include "Functor.h"
#include <string>
#include <mutex>
#include <map>

namespace curan{
    namespace communication{
        		/*
		The channel manager contains channels that can be connected to
		different objects. These objects can submit themselfs in order
		to be notified.

		Each object which wishes to be warned should request a channel
		which is compatible with the language that they understand. To
		increase the maximum velocity that the application runs the
		burden of selecting the type of message is imposed on the listener.
		*/
		class ChannelManager
		{

		public:

			/*
			Enumeration of possible errors returned in the Ck module
			*/
			enum Error {
				SUCCESS = 0,
				CHANNEL_NOT_PRESENT,
				CHANNEL_ALREADY_PRESENT,
			};

			/*
			The error callback function should be submited by the user
			to be called at a latter point in time.
			*/
			using error_callback = std::function<void(Error)>;
			
			static ChannelManager* Get();
			void start_channel(std::string name, asio::io_context& ctx, error_callback callback = error_callback());
			Error get_channel(std::string name, std::shared_ptr<ProcessorOIGTL>& channel);
			Error terminate_channel(std::string name);
			void terminate_all_channels();

		private:
			ChannelManager();
			std::mutex mut;
			Functor functor;
			std::map<std::string, std::shared_ptr<ProcessorOIGTL>> openigtlink_channel_list;
		};
    }
}

#endif
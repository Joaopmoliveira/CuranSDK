#include "communication/ChannelManager.h"

#include "utils/TheadPool.h"
#include "communication/ProcessorOIGTL.h"

namespace curan{
    namespace communication{
        ChannelManager::ChannelManager()
		{
		}

		ChannelManager* ChannelManager::Get()
		{
			static ChannelManager channel_manager;
			return &channel_manager;
		}

		void ChannelManager::start_channel(std::string name, asio::io_context& ctx, error_callback callback)
		{
			auto it = openigtlink_channel_list.find(name);
			if (it == openigtlink_channel_list.end())
			{
				if (!functor.is_running())
				{
					curan::utils::Job pending_task = curan::utils::Job{ "communication functor",std::bind(&Functor::operator(), &functor) };
					curan::utils::ThreadPool* pool = curan::utils::ThreadPool::Get();
					pool->Submit(pending_task);
				}
				// there is no channel with this name so we can continue
				std::shared_ptr<ProcessorOIGTL> pointer_to_processor = std::make_shared<ProcessorOIGTL>(ctx);
				openigtlink_channel_list.emplace(name, pointer_to_processor);
				if (callback)
					callback(Error::SUCCESS);
				return;
			}
			else {
				// there is already a channel with this name, we need to get 
				//out but first we should check if the functor of this channel 
				//is actually running, if not we should jump start it again
				if (!functor.is_running())
				{
					curan::utils::Job pending_task = curan::utils::Job{ "communication functor",std::bind(&Functor::operator(), &functor) };
					curan::utils::ThreadPool* pool = curan::utils::ThreadPool::Get();
					pool->Submit(pending_task);
				}
				if (callback)
					callback(Error::CHANNEL_ALREADY_PRESENT);
				return;
			}
		}



		ChannelManager::Error ChannelManager::get_channel(std::string name, std::shared_ptr<ProcessorOIGTL>& channel)
		{
			auto it = openigtlink_channel_list.find(name);
			if (it == openigtlink_channel_list.end())// there is no channel with this name so we need to return a nullptr
				return Error::CHANNEL_NOT_PRESENT;
			else
				channel = it->second;
			return Error::SUCCESS;
		}

		ChannelManager::Error ChannelManager::terminate_channel(std::string name)
		{
			auto it = openigtlink_channel_list.find(name);
			if (it == openigtlink_channel_list.end())// there is no channel with this name so we need to return a nullptr
			{
				return Error::CHANNEL_NOT_PRESENT;
			}
			else
			{
				openigtlink_channel_list.erase(it);
				return Error::SUCCESS;
			}
		}

		void ChannelManager::terminate_all_channels()
		{
			auto it = openigtlink_channel_list.begin();
			while (it != openigtlink_channel_list.end()) {
				it->second->close();
				++it;
			}

		}
    }
}
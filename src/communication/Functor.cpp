#include "Functor.h"

namespace curan{
    namespace communication{
       Functor::Functor()
		{
			GetIOContext(&io_context_);
		}

		void Functor::operator()() {
			try {
				using work_guard_type = asio::executor_work_guard<asio::io_context::executor_type>;
				work_guard_type work_guard(io_context_->get_executor());
				is_active = true;
				curan::utils::console->warn("connection thread started running");
				io_context_->run();
				is_active = false;
				curan::utils::console->warn("connection thread stoped running");
				io_context_->stop();
			}
			catch (std::exception& e) {
				curan::utils::console->warn("connection thread stoped running due to exception");
				curan::utils::console->warn(e.what());
				io_context_->stop();
			}

		}

		bool Functor::is_running()
		{
			return is_active;
		} 
    }
}
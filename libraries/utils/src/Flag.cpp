#include "utils/Flag.h"

namespace curan{
    namespace utils{
        void Flag::set()
		{
			std::lock_guard g(mutex_);
			flag_ = true;
			cond_var_.notify_all();
		}

		void Flag::clear()
		{
			std::lock_guard g(mutex_);
			flag_ = false;
		}

		void Flag::wait()
		{
			std::unique_lock lock(mutex_);
			cond_var_.wait(lock, [this]() { return flag_; });
		}
    }
}
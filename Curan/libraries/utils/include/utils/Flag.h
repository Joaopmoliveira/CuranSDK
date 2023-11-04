#ifndef CURAN_FLAG_HEADER_FILE_
#define CURAN_FLAG_HEADER_FILE_

#include <mutex>
#include <memory>
#include <condition_variable>

namespace curan {
	namespace utilities {
		/*
	*	Thread safe flag which can be used to wait
	*   for a given event from multiple threads. When
	*	created the boolean value is set to false. All
	*	the methods are protected by a mutex, thus
	*	avoiding race conditions.
	*/
		class Flag
		{
		public:

			Flag() : flag_{ false } {}

			Flag(bool var) : flag_{ var } {}

			Flag(const Flag&) = delete;

			Flag& operator=(const Flag&) = delete;

			/*
			Activates the flag, i.e. the boolean value is set to true
			*/
			inline void set(bool val){
				std::lock_guard g(mutex_);
				flag_ = val;
				cond_var_.notify_all();
			}

			/*
			Waits for the boolean value to be turned to true by some thread.
			*/
			inline void wait(){
				std::unique_lock lock(mutex_);
				cond_var_.wait(lock, [this]() { return flag_; });
			}

			/*
			Return the current value of the underlying flag
			*/
			inline bool value() {
				std::lock_guard<std::mutex> g{mutex_};
				return flag_;
			}

		private:
			bool flag_;
			std::mutex mutex_;
			std::condition_variable cond_var_;
		};

	}
}

#endif
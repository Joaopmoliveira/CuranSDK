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
		class Flag : std::enable_shared_from_this<Flag>
		{
			Flag() : flag_{ false } {}

		public:
			
			/*
			This guarantees that the user cannot violate the shared flag, and given that be finition 
			a shared flag is shared across threads then we need a smart pointer to handle memory allocation
			*/
			static std::shared_ptr<Flag> make_shared_flag();

			/*
			Return a copy of this shared flag with a common incrementer and 
			common undelrying pointer
			*/
			std::shared_ptr<Flag> makecommoncopy();

			/*
			Activates the flag, i.e. the boolean value is set to true
			*/
			void set();

			/*
			Deactivates the flag, i.e. the boolean value is set to false
			*/
			void clear();

			/*
			Waits for the boolean value to be turned to true by some thread.
			*/
			void wait();

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
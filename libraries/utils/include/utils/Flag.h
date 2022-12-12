#ifndef CURAN_FLAG_HEADER_FILE_
#define CURAN_FLAG_HEADER_FILE_

#include <mutex>

namespace curan {
	namespace utils {
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

		private:
			bool flag_;
			std::mutex mutex_;
			std::condition_variable cond_var_;
		};

	}
}

#endif
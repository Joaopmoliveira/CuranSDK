#ifndef CURAN_THREADPOOL_HEADER_FILE_
#define CURAN_THREADPOOL_HEADER_FILE_

#include <mutex>
#include <vector>
#include "ThreadSafeQueue.h"
#include "Job.h"

namespace curan {
	namespace utils {
		/*
		The DkThreadPool class should be a unique entity in
		the application. Internally it contains a number of
		threads (this number is platform and machine specific) that are
		in a blocked state until tasks are submited to the pool.
		The class contains a numbering mechanism that can
		be queried for the number of thread which are currently
		working and the number of tasks which are in queue
		to be run.
		*/
		class ThreadPool
		{
		private:
			bool stopped = false;
			mutable std::mutex mut;
			std::vector<std::thread> pool;
			ThreadSafeQueue<Job> job_queue;
			int number_of_tasks_executing = 0;
			int number_of_pending_tasks = 0;
			ThreadPool();
			~ThreadPool();
			void infinite_loop();

		public:
			static ThreadPool* Get();
			void GetNumberTasks(int& tasks_executing, int& tasks_in_queue);
			void Submit(Job task);
			void Shutdown();
		};
	}
}

#endif
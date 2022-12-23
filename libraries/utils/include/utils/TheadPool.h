#ifndef CURAN_THREADPOOL_HEADER_FILE_
#define CURAN_THREADPOOL_HEADER_FILE_

#include <mutex>
#include <vector>
#include "ThreadSafeQueue.h"
#include "Job.h"

namespace curan {
	namespace utils {
		/*
		A thread pool contains an array of threads which are waiting for
		jobs to be supplied. Because this thread pool is 
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
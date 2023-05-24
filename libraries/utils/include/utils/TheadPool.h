#ifndef CURAN_THREADPOOL_HEADER_FILE_
#define CURAN_THREADPOOL_HEADER_FILE_

#include "SafeQueue.h"
#include <mutex>
#include <vector>
#include "Job.h"

namespace curan {
	namespace utilities {
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
			SafeQueue<Job> job_queue;
			int number_of_tasks_executing = 0;
			int number_of_pending_tasks = 0;
			void infinite_loop();

		public:

			ThreadPool(int number_of_threads);
			~ThreadPool();

			void get_number_tasks(int& tasks_executing, int& tasks_in_queue);
			void submit(Job task);
			void shutdown();
		};

		extern std::unique_ptr<ThreadPool> pool;

		void initialize_thread_pool(int number_of_threads);

		void terminate_thread_pool();
}
}

#endif
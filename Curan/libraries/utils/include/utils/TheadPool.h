#ifndef CURAN_THREADPOOL_HEADER_FILE_
#define CURAN_THREADPOOL_HEADER_FILE_

#include "SafeQueue.h"
#include <mutex>
#include <vector>
#include "Job.h"
#include <memory>

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
			std::mutex mut;
			std::vector<std::thread> pool;
			SafeQueue<Job> job_queue;
			int number_of_tasks_executing = 0;
			int number_of_pending_tasks = 0;
			void infinite_loop();
			ThreadPool(size_t number_of_threads);
		public:
			~ThreadPool();
			static std::shared_ptr<ThreadPool> create(size_t num_of_threads);
			void get_number_tasks(int& tasks_executing, int& tasks_in_queue);
			void submit(Job task);
			void shutdown();
			inline size_t size(){
				std::lock_guard<std::mutex> g{mut};
				return pool.size();
			}
		};
}
}

#endif
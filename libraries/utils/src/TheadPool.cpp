#include "utils/TheadPool.h"

namespace curan{
    namespace utils{
        ThreadPool::ThreadPool()
		{
			int num_threads = std::thread::hardware_concurrency();
			num_threads /= 2.0;
			if (num_threads < 1)
				num_threads = 1;
			for (int ii = 0; ii < num_threads; ii++)
				pool.push_back(std::thread(&ThreadPool::infinite_loop, this));
		}

		ThreadPool::~ThreadPool()
		{
			if (!stopped)
				Shutdown();
		}

		void ThreadPool::infinite_loop()
		{
			Job job;
			while (true)
			{
				if (!job_queue.wait_and_pop(job))
					break;
				{
					std::lock_guard<std::mutex> lk(mut);
					++number_of_tasks_executing;
					--number_of_pending_tasks;
				}
				job.function_to_execute();
				{
					std::lock_guard<std::mutex> lk(mut);
					--number_of_tasks_executing;
				}
			}
		}

		ThreadPool* ThreadPool::Get()
		{
			static ThreadPool resource{};
			return &resource;
		}

		void ThreadPool::GetNumberTasks(int& tasks_executing, int& tasks_in_queue)
		{
			std::lock_guard<std::mutex> lk(mut);
			tasks_executing = number_of_tasks_executing;
			tasks_in_queue = number_of_pending_tasks;
		}

		void ThreadPool::Submit(Job task)
		{
			std::lock_guard<std::mutex> lk(mut);
			++number_of_pending_tasks;
			job_queue.push(task);
		}

		void ThreadPool::Shutdown()
		{
			std::lock_guard<std::mutex> lk(mut);
			job_queue.invalidate();

			for (std::thread& every_thread : pool)
				every_thread.join();

			pool.clear();
			stopped = true; // use this flag in destructor, if not set, call shutdown() 
		}
    }
}
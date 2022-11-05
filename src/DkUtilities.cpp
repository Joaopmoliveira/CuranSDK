#include "DkUtilities.h"

namespace curan
{
	namespace utils {
		std::shared_ptr<spdlog::logger>console = spdlog::stdout_color_mt("logger");

		ThreadPool::ThreadPool()
		{
			int num_threads = std::thread::hardware_concurrency();
			num_threads /= 2;
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
			while (true)
			{
				Job job = job_queue.wait_and_pop();
				if (job_queue.is_invalid())
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
			job_queue.invalidate();

			for (std::thread& every_thread : pool)
				every_thread.join();

			pool.clear();
			stopped = true; // use this flag in destructor, if not set, call shutdown() 
		}

		Job::Job(std::string descript, std::function<void(void)> funct) : description{ descript }, function_to_execute{ funct }
		{}

		Job::Job()
		{
		}

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
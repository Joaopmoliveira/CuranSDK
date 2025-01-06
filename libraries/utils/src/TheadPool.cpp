#include "utils/TheadPool.h"
#include <iostream>
#include <memory>

namespace curan{
namespace utilities{

ThreadPool::ThreadPool(size_t number_of_threads, ThreadPoolDestructionBehavior in_behavior) : stopped{false} , behavior{in_behavior}
{
	for (int ii = 0; ii < number_of_threads; ii++)
		pool.push_back(std::thread(&ThreadPool::infinite_loop, this));
}

void ThreadPool::shutdown()
{
	{
		std::lock_guard<std::mutex> lk(mut);
		if(stopped)
			return;
		stopped = true;
	}
	internal_shutdown();
}

ThreadPool::~ThreadPool()
{
	bool call_internal_shutshown = false;
	{
		std::lock_guard<std::mutex> lk(mut);
		if(!stopped){
			stopped = true;
			call_internal_shutshown = true;
		}
	}
	if(call_internal_shutshown) internal_shutdown();
}

void ThreadPool::internal_shutdown()
{
	switch(behavior){
		case ThreadPoolDestructionBehavior::RETURN_AS_FAST_AS_POSSIBLE:
		{
			job_queue.invalidate();
			for (std::thread& every_thread : pool)
				every_thread.join();
			pool.clear();
		}
		break;
		case ThreadPoolDestructionBehavior::TERMINATE_ALL_PENDING_TASKS:
		default:
		{
			std::cout << "waiting for threads...\n";
			for (std::thread& every_thread : pool)
				every_thread.join();
			pool.clear();
		}
		break;
	}

}

void ThreadPool::infinite_loop()
{
	std::optional<Job> job;
	while (true){
		job = job_queue.wait_and_pop();

		if(job_queue.invalid()){
			return;
		}
			
		if (!job)
			continue;

		{
			std::lock_guard<std::mutex> lk(mut);
			++number_of_tasks_executing;
			--number_of_pending_tasks;
		}
		(*job)();
		{
			std::lock_guard<std::mutex> lk(mut);
			--number_of_tasks_executing;
		}

		{
			std::lock_guard<std::mutex> lk(mut);
			if (stopped && number_of_pending_tasks==0)
				if(job_queue.valid())
					job_queue.invalidate();
		}

	}
}

void ThreadPool::get_number_tasks(int& tasks_executing, int& tasks_in_queue)
{
	std::lock_guard<std::mutex> lk(mut);
	tasks_executing = number_of_tasks_executing;
	tasks_in_queue = number_of_pending_tasks;
}

void ThreadPool::submit(Job task)
{
	std::lock_guard<std::mutex> lk(mut);
	if(stopped)
		return;
	++number_of_pending_tasks;
	job_queue.push(task);
}

std::shared_ptr<curan::utilities::ThreadPool> ThreadPool::create(size_t num_of_threads,ThreadPoolDestructionBehavior in_behavior){
	return std::shared_ptr<curan::utilities::ThreadPool>(new ThreadPool{num_of_threads,in_behavior});
}

}
}
#include "utils/TheadPool.h"
#include <iostream>
#include <memory>

namespace curan{
namespace utilities{

ThreadPool::ThreadPool(size_t number_of_threads) : stopped{false}
{
	for (int ii = 0; ii < number_of_threads; ii++)
		pool.push_back(std::thread(&ThreadPool::infinite_loop, this));
}

ThreadPool::~ThreadPool()
{
	std::lock_guard<std::mutex> lk(mut);
	std::cout << "destroying thread pool\n";
	if (!stopped){
		stopped = true;
		internal_shutdown();
	}

}

void ThreadPool::infinite_loop()
{
	std::optional<Job> job;
	while (true){
		job = job_queue.wait_and_pop();
		if (!job)
			continue;
		
		if(job_queue.is_invalid())
			return;
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
	++number_of_pending_tasks;
	job_queue.push(task);
}

void ThreadPool::internal_shutdown()
{
	job_queue.invalidate();
	for (std::thread& every_thread : pool)
		every_thread.join();
	pool.clear();
	stopped = true; // use this flag in destructor, if not set, call shutdown() 
}

void ThreadPool::shutdown()
{
	std::lock_guard<std::mutex> lk(mut);
	if(stopped)
		return;
	internal_shutdown();
}

std::shared_ptr<curan::utilities::ThreadPool> ThreadPool::create(size_t num_of_threads){
	return std::shared_ptr<curan::utilities::ThreadPool>(new ThreadPool{num_of_threads});
}

}
}
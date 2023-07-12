#include "utils/TheadPool.h"
#include <memory>

namespace curan{
namespace utilities{

ThreadPool::ThreadPool(size_t number_of_threads)
{
	for (int ii = 0; ii < number_of_threads; ii++)
		pool.push_back(std::thread(&ThreadPool::infinite_loop, this));
}

ThreadPool::~ThreadPool()
{
	std::lock_guard<std::mutex> lk(mut);
	if (!stopped)
		shutdown();
}

void ThreadPool::infinite_loop()
{
	Job job;
	while (true){
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

void ThreadPool::shutdown()
{
	std::lock_guard<std::mutex> lk(mut);
	job_queue.invalidate();
	for (std::thread& every_thread : pool)
		every_thread.join();
	pool.clear();
	stopped = true; // use this flag in destructor, if not set, call shutdown() 
}

std::shared_ptr<curan::utilities::ThreadPool> ThreadPool::create(size_t num_of_threads){
	return std::shared_ptr<curan::utilities::ThreadPool>(new ThreadPool{num_of_threads});
}

}
}
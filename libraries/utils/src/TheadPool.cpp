#include "utils/TheadPool.h"

namespace curan{
namespace utils{

std::unique_ptr<ThreadPool> pool = nullptr;

void initialize_thread_pool(int number_of_threads) {
	pool = std::make_unique<ThreadPool>(number_of_threads);
}

void terminate_thread_pool(){
	pool.reset();
}

ThreadPool::ThreadPool(int number_of_threads)
{
	for (int ii = 0; ii < number_of_threads; ii++)
		pool.push_back(std::thread(&ThreadPool::infinite_loop, this));
}

ThreadPool::~ThreadPool()
{
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

}
}
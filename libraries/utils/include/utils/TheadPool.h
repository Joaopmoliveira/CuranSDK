#ifndef CURAN_THREADPOOL_HEADER_FILE_
#define CURAN_THREADPOOL_HEADER_FILE_

#include "SafeQueue.h"
#include <mutex>
#include <vector>
#include "Job.h"
#include <memory>
#include <thread>

namespace curan {
namespace utilities {

/*
A thread pool is an incredible usefull entity. Internally 
N threads are running. All blocked waiting for a Job to be
submited so that the thread pool can execute work. This allows 
the computer to avoid running in idle cpu cycles. 

The destructor of the class takes care of calling .join() on each internal 
thread. A typical use case of the class goes as follows;

int main(){
	auto thread_pool = ThreadPool::create(2);
	thread_pool->submit({"first job to me executed",[](){ std::cout << "printing myself right now" << std::endl; }});
	thread_pool->submit({"first job to me executed",[](){ std::cout << "printing myself right now" << std::endl; }});
	thread_pool->submit({"first job to me executed",[](){ std::cout << "printing myself right now" << std::endl; }});
	thread_pool->submit({"first job to me executed",[](){ std::cout << "printing myself right now" << std::endl; }});
	thread_pool->submit({"first job to me executed",[](){ std::cout << "printing myself right now" << std::endl; }});
	return 0;
}

Notice that in this previous example because 
we only have two available threads, there is more work than workers, 
what happens is that the work remains in the interal queue
of the threadpool until the other older work has been processed.
*/

class ThreadPool{
public:

~ThreadPool();
ThreadPool(const ThreadPool& other) =  delete;

static std::shared_ptr<ThreadPool> create(size_t num_of_threads);

void get_number_tasks(int& tasks_executing, int& tasks_in_queue);

void submit(Job task);

void shutdown();

inline size_t size(){
	std::lock_guard<std::mutex> g{mut};
	return pool.size();
}

private:

void infinite_loop();

ThreadPool(size_t number_of_threads);

void internal_shutdown();

bool stopped = false;
std::mutex mut;
std::vector<std::thread> pool;
SafeQueue<Job> job_queue;
int number_of_tasks_executing = 0;
int number_of_pending_tasks = 0;
};

}
}

#endif
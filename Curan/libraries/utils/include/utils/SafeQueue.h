#ifndef CURAN_THREADSAFEQUEUE_HEADER_FILE_
#define CURAN_THREADSAFEQUEUE_HEADER_FILE_

#include <mutex>
#include <queue>
#include <condition_variable>
#include <optional>

namespace curan {
namespace utilities {

/*
The SafeQueue is a pivotal class inside Curan. It allows us
to share information between threads with safety guarantees. 

** I have not tested all requirements of the templated type**

A simple use case of this class is having two thread, a producer 
and a consumer and waiting without wasting idle cpu time, until 
the producer has generated an object for us to process. 

Consider the following

void foo(SafeQueue<double>& queue){
	if(auto value = queue.wait_and_pop(); value)
		std::cout << "value: " << *value;
	else
		std::cout << "no signal received"
}

int main(){
	SafeQueue<double> queue;
	std::thread oth{[&](){foo(queue);}};

	
	std::this_thread::sleep_for(std::chrono::seconds(1000));

	oth.join();
	return 0;
}

Inside curan we use this class to receive
assyncrounous signals from the operating system
to then use these signals for drawing operations.
*/

template<typename T>
class SafeQueue {

public:

SafeQueue() {}
SafeQueue(SafeQueue const& other) = delete;

void push(const T& new_value) {
	std::lock_guard<std::mutex> lk(mut);
	data_queue.push(new_value);
	data_cond.notify_one();
}

void clear(){
	std::lock_guard<std::mutex> lk(mut);
	data_queue = std::queue<T>{};
}

[[nodiscard]] std::optional<T> wait_and_pop() {
	std::unique_lock<std::mutex> lk(mut);
	data_cond.wait(lk, [this] {return (!data_queue.empty() || invalid); });
	if (invalid || data_queue.empty())
		return std::nullopt;
	auto value = data_queue.front();
	data_queue.pop();
	return value;
}

[[nodiscard]] std::optional<T> try_pop() {
	std::lock_guard<std::mutex> lk(mut);
	if (data_queue.empty())
		return std::nullopt;
	auto value = data_queue.front();
	data_queue.pop();
	return value;
}

[[nodiscard]] std::optional<T> front() {
	std::lock_guard<std::mutex> lk(mut);
	if (data_queue.empty())
		return std::nullopt;
	auto value = data_queue.front();
	return value;
}

[[nodiscard]] bool empty(){
	std::lock_guard<std::mutex> lk(mut);
	return data_queue.empty();
}

[[nodiscard]] size_t size() {
	std::lock_guard<std::mutex> lk(mut);
	return data_queue.size();
}

void invalidate() {
	std::lock_guard<std::mutex> lk(mut);
	if (!invalid) {
		invalid = true;
		data_cond.notify_all();
	}
}

[[nodiscard]] bool is_invalid() {
	std::lock_guard<std::mutex> lk(mut);
	return invalid;
}

private:

bool invalid = false;
std::mutex mut;
std::queue<T> data_queue;
std::condition_variable data_cond;

};

}
}

#endif



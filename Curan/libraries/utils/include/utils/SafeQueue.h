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


*/

template<typename T>
class SafeQueue {

public:

SafeQueue() {}

SafeQueue(SafeQueue const& other) {
	std::scoped_lock lck{other.mut , mut};
	data_queue = other.data_queue;
}

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



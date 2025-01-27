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
	queue.push(5.0);

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
	std::lock_guard<std::mutex> lk(m_mut);
	if(m_invalid)
		return;
	m_data_queue.push(new_value);
	m_data_cond.notify_one();
}

void emplace(auto&&... args){
	std::lock_guard<std::mutex> lk(m_mut);
	if(m_invalid)
		return;
	m_data_queue.emplace(std::forward<decltype(args)>(args)...);
	m_data_cond.notify_one();
}

void clear(){
	std::lock_guard<std::mutex> lk(m_mut);
	m_data_queue = std::queue<T>{};
}

[[nodiscard]] std::optional<T> wait_and_pop() {
	std::unique_lock<std::mutex> lk(m_mut);
	if (m_invalid)
		return std::nullopt;
	m_data_cond.wait(lk, [this] {return (!m_data_queue.empty() || m_invalid); });
	if (m_invalid ||m_data_queue.empty())
		return std::nullopt;
	auto value = m_data_queue.front();
	m_data_queue.pop();
	return value;
}

template< class Rep, class Period>
[[nodiscard]] std::optional<T> wait_and_pop(const std::chrono::duration<Rep, Period>& rel_time) {
	std::unique_lock<std::mutex> lk(m_mut);
	if (m_invalid)
		return std::nullopt;
	bool cond = m_data_cond.wait_for(lk,rel_time, [this] {return (!m_data_queue.empty() || m_invalid); });
	if (m_invalid || m_data_queue.empty())
		return std::nullopt;
	auto value = m_data_queue.front();
	m_data_queue.pop();
	return value;
}

[[nodiscard]] std::optional<T> try_pop() {
	std::lock_guard<std::mutex> lk(m_mut);
	if (m_data_queue.empty() || m_invalid)
		return std::nullopt;
	auto value = m_data_queue.front();
	m_data_queue.pop();
	return value;
}

[[nodiscard]] std::optional<T> front() {
	std::lock_guard<std::mutex> lk(m_mut);
	if (m_data_queue.empty() || m_invalid)
		return std::nullopt;
	auto value = m_data_queue.front();
	return value;
}

[[nodiscard]] std::optional<T> back() {
	std::lock_guard<std::mutex> lk(m_mut);
	if (m_data_queue.empty() || m_invalid)
		return std::nullopt;
	auto value = m_data_queue.back();
	return value;
}

[[nodiscard]] bool empty(){
	std::lock_guard<std::mutex> lk(m_mut);
	return m_data_queue.empty();
}

[[nodiscard]] size_t size() {
	std::lock_guard<std::mutex> lk(m_mut);
	return m_data_queue.size();
}

void invalidate() {
	std::lock_guard<std::mutex> lk(m_mut);
	if (!m_invalid) {
		m_invalid = true;
		m_data_cond.notify_all();
	}
}

[[nodiscard]] bool invalid() {
	std::lock_guard<std::mutex> lk(m_mut);
	return m_invalid;
}

[[nodirscard]] bool  valid(){
	std::lock_guard<std::mutex> lk(m_mut);
	return !m_invalid;
}

[[nodiscard]] std::queue<T> request_current_queue_copy(){
	std::lock_guard<std::mutex> lk(m_mut);
	return m_data_queue;
}

private:

bool m_invalid = false;
std::mutex m_mut;
std::queue<T> m_data_queue;
std::condition_variable m_data_cond;

};

}
}

#endif



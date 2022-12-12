#ifndef CURAN_THREADSAFEQUEUE_HEADER_FILE_
#define CURAN_THREADSAFEQUEUE_HEADER_FILE_

#include <mutex>
#include <queue>
#include "Logger.h"

namespace curan {
	namespace utils {
		/*
		Contained type must be default constructable
		*/
		template<typename contained>
		class ThreadSafeQueue {
		private:
			bool invalid = false;
			std::mutex mut;
			std::queue<contained> data_queue;
			std::condition_variable data_cond;
		public:
			ThreadSafeQueue() {

			}
			ThreadSafeQueue(ThreadSafeQueue const& other) {
				std::lock_guard<std::mutex> lc(other.mut);
				std::lock_guard<std::mutex> lk(mut);
				data_queue = other.data_queue;
			}

			void push(contained new_value) {
				std::lock_guard<std::mutex> lk(mut);
				data_queue.push(new_value);
				data_cond.notify_one();
			}

			bool wait_and_pop(contained& value) {
				std::unique_lock<std::mutex> lk(mut);
				data_cond.wait(lk, [this] {return (!data_queue.empty() || invalid); });
				if (invalid || data_queue.empty()) {
					value = contained();
					return false;
				}
				value = data_queue.front();
				data_queue.pop();
				return true;
			}

			contained wait_and_pop() {
				std::unique_lock<std::mutex> lk(mut);
				data_cond.wait(lk, [this] {return ((!data_queue.empty()) || invalid); });
				if (invalid || data_queue.empty())
					return contained();
				contained res(data_queue.front());
				data_queue.pop();
				return res;
			};

			bool try_pop(contained& value) {
				std::lock_guard<std::mutex> lk(mut);
				if (data_queue.empty())
					return false;
				value = data_queue.front();
				data_queue.pop();
				return true;
			}
			contained try_pop() {
				std::lock_guard<std::mutex> lk(mut);
				if (data_queue.empty()) {
					return contained();
				}
				contained res(data_queue.front());
				data_queue.pop();
				return res;
			}

			bool try_front(contained& value) {
				std::lock_guard<std::mutex> lk(mut);
				if (data_queue.empty())
					return false;
				value = data_queue.front();
				return true;
			}
			contained try_front() {
				std::lock_guard<std::mutex> lk(mut);
				if (data_queue.empty()) {
					return contained();
				}
				contained res(data_queue.front());
				return res;
			}

			bool empty() const {
				std::lock_guard<std::mutex> lk(mut);
				return data_queue.empty();
			}
			int size() {
				std::lock_guard<std::mutex> lk(mut);
				return data_queue.size();
			}
			void invalidate() {
				{
					std::lock_guard<std::mutex> lk(mut);
					invalid = true;
				}
				data_cond.notify_all();
			}
			bool is_invalid() {
				console->info("trying to lock id of thread {}", std::this_thread::get_id());
				std::lock_guard<std::mutex> lk(mut);
				return invalid;
			}
		};
	}
}

#endif



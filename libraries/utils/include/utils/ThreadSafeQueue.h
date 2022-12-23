#ifndef CURAN_THREADSAFEQUEUE_HEADER_FILE_
#define CURAN_THREADSAFEQUEUE_HEADER_FILE_

#include <mutex>
#include <queue>
#include "Logger.h"

namespace curan {
	namespace utils {
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

			[[nodiscard]] bool wait_and_pop(contained& value) {
				std::unique_lock<std::mutex> lk(mut);
				data_cond.wait(lk, [this] {return (!data_queue.empty() || invalid); });
				if (invalid || data_queue.empty()) {
					return false;
				}
				value = data_queue.front();
				data_queue.pop();
				return true;
			}

			[[nodiscard]] bool try_pop(contained& value) {
				std::lock_guard<std::mutex> lk(mut);
				if (data_queue.empty())
					return false;
				value = data_queue.front();
				data_queue.pop();
				return true;
			}

			[[nodiscard]] bool try_front(contained& value) {
				std::lock_guard<std::mutex> lk(mut);
				if (data_queue.empty())
					return false;
				value = data_queue.front();
				return true;
			}

			[[nodiscard]] bool empty(){
				std::lock_guard<std::mutex> lk(mut);
				return data_queue.empty();
			}

			[[nodiscard]] int size() {
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

			[[nodiscard]] bool is_invalid() {
				std::lock_guard<std::mutex> lk(mut);
				return invalid;
			}
		};
	}
}

#endif



#include <mutex>
#include <queue>

namespace curan {
	namespace utils {
		/*
contained type must be default constructable
*/
		template<typename contained>
		class ThreadSafeQueue {
		private:
			bool invalid = false;
			mutable std::mutex mut;
			std::queue<contained> data_queue;
			std::condition_variable data_cond;
		public:
			ThreadSafeQueue();
			ThreadSafeQueue(ThreadSafeQueue const& other);

			void push(contained new_value);

			void wait_and_pop(contained& value);
			contained wait_and_pop();

			bool try_pop(contained& value);
			contained try_pop();

			bool try_front(contained& value);
			contained try_front();

			bool empty() const;
			int size();
			void invalidate();
			bool is_invalid();
		};

		template<typename contained>
		inline ThreadSafeQueue<contained>::ThreadSafeQueue()
		{

		}

		template<typename contained>
		inline ThreadSafeQueue<contained>::ThreadSafeQueue(ThreadSafeQueue<contained> const& other)
		{
			std::lock_guard<std::mutex> lc(other.mut);
			std::lock_guard<std::mutex> lk(mut);
			data_queue = other.data_queue;
		}

		template<typename contained>
		inline void ThreadSafeQueue<contained>::push(contained new_value)
		{
			std::lock_guard<std::mutex> lk(mut);
			data_queue.push(new_value);
			data_cond.notify_one();
		}

		template<typename contained>
		inline void ThreadSafeQueue<contained>::wait_and_pop(contained& value)
		{
			std::unique_lock<std::mutex> lk(mut);
			data_cond.wait(lk, [this] {return (!data_queue.empty() || invalid); });
			if (invalid) {
				value = contained();
				return;
			}
			value = data_queue.front();
			data_queue.pop();
		}

		template<typename contained>
		inline void ThreadSafeQueue<contained>::invalidate()
		{
			std::unique_lock<std::mutex> lk(mut);
			invalid = true;
			data_cond.notify_all();
		}

		template<typename contained>
		inline bool ThreadSafeQueue<contained>::is_invalid()
		{
			std::unique_lock<std::mutex> lk(mut);
			return invalid;
		}

		template<typename contained>
		inline contained ThreadSafeQueue<contained>::wait_and_pop()
		{
			std::unique_lock<std::mutex> lk(mut);
			data_cond.wait(lk, [this] {return ((!data_queue.empty()) || invalid); });
			if (invalid)
				return contained();
			contained res(data_queue.front());
			data_queue.pop();
			return res;
		}

		template<typename contained>
		inline bool ThreadSafeQueue<contained>::try_pop(contained& value)
		{
			std::lock_guard<std::mutex> lk(mut);
			if (data_queue.empty())
				return false;
			value = data_queue.front();
			data_queue.pop();
			return true;
		}

		template<typename contained>
		inline contained ThreadSafeQueue<contained>::try_pop()
		{
			std::lock_guard<std::mutex> lk(mut);
			if (data_queue.empty()) {
				return contained();
			}
			contained res(data_queue.front());
			data_queue.pop();
			return res;
		}


		template<typename contained>
		inline bool ThreadSafeQueue<contained>::try_front(contained& value)
		{
			std::lock_guard<std::mutex> lk(mut);
			if (data_queue.empty())
				return false;
			value = data_queue.front();
			return true;
		}

		template<typename contained>
		inline contained ThreadSafeQueue<contained>::try_front()
		{
			std::lock_guard<std::mutex> lk(mut);
			if (data_queue.empty()) {
				return contained();
			}
			contained res(data_queue.front());
			return res;
		}


		template<typename contained>
		inline bool ThreadSafeQueue<contained>::empty() const
		{
			std::lock_guard<std::mutex> lk(mut);
			return data_queue.empty();
		}

		template<typename contained>
		inline int ThreadSafeQueue<contained>::size()
		{
			std::lock_guard<std::mutex> lk(mut);
			return data_queue.size();
		}
	}
}





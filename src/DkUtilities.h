#ifndef DkUtilities_h_DEFINED
#define DkUtilities_h_DEFINED

#include <mutex>
#include <queue> 

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h" // must be included
#include "spdlog/sinks/stdout_color_sinks.h"

#include "stb_image.h"
#include <map>
#include <filesystem>
#include <sstream>
#include <string>


namespace curan {
	namespace utils {
		extern std::shared_ptr<spdlog::logger> console;

		static unsigned long long current_identifier_number = 0;

		struct message {
			void* content;
			unsigned long long identifier;
		};

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


		/*
		A pending task is a request made by the user which
		currently sits in queue which must be terminated.
		It has a description associated with the task to finish.
		*/
		struct Job {
			/*
			* Function that actually gets executed by the thread pool of the application
			*/
			std::function<void(void)> function_to_execute;
			/*
			* A small description of the task that should be executed. This is used to
			* provide feedback to the user on the operatorions currently in queue.
			*/
			std::string description;

			Job(std::string descript, std::function<void(void)> funct);

			Job();
		};

		/*
		The DkThreadPool class should be a unique entity in
		the application. Internally it contains a number of
		threads (this number is platform and machine specific) that are
		in a blocked state until tasks are submited to the pool.
		The class contains a numbering mechanism that can
		be queried for the number of thread which are currently
		working and the number of tasks which are in queue
		to be run.
		*/
		class ThreadPool
		{
		private:
			bool stopped = false;
			mutable std::mutex mut;
			std::vector<std::thread> pool;
			ThreadSafeQueue<Job> job_queue;
			int number_of_tasks_executing = 0;
			int number_of_pending_tasks = 0;
			ThreadPool();
			~ThreadPool();
			void infinite_loop();

		public:
			static ThreadPool* Get();
			void GetNumberTasks(int& tasks_executing, int& tasks_in_queue);
			void Submit(Job task);
			void Shutdown();
		};


		template <typename T>
		std::string to_string_with_precision(const T a_value, const int n = 6)
		{
			std::ostringstream out;
			out.precision(n);
			out << std::fixed << a_value;
			return out.str();
		}

		/*
		*	Thread safe flag which can be used to wait
		*   for a given event from multiple threads. When 
		*	created the boolean value is set to false. All
		*	the methods are protected by a mutex, thus 
		*	avoiding race conditions.
		*/
		class Flag
		{
		public:
			Flag() : flag_{ false } {}

			/*
			Activates the flag, i.e. the boolean value is set to true
			*/
			void set();

			/*
			Deactivates the flag, i.e. the boolean value is set to false
			*/
			void clear();

			/*
			Waits for the boolean value to be turned to true by some thread. 
			*/
			void wait();

		private:
			bool flag_;
			std::mutex mutex_;
			std::condition_variable cond_var_;
		};

		/*
		* 
		* 
		* 
		* 
		* 
		* 
		*/
		class Coordinates
		{

		};
	}
}
#endif
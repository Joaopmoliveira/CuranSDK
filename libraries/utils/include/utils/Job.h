#include <string>
#include <functional>

namespace curan {
	namespace utils {
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
	}
}
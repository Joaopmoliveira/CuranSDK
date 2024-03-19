#ifndef CURAN_JOB_HEADER_FILE_
#define CURAN_JOB_HEADER_FILE_

#include <string>
#include <functional>

namespace curan {
namespace utilities {

/*
A pending task is a request made by the user which
currently sits in queue which must be terminated.
It has a description associated with the task to finish.
*/
class Job {
public:
Job(std::string descript, std::function<void(void)> funct);

Job();

inline void operator() () const {
	if(function_to_execute)
		function_to_execute();
}

inline std::string description() const {
	return _description;
}

private:

std::function<void(void)> function_to_execute;
std::string _description;

};

}
}

#endif
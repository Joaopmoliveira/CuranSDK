#ifndef CURAN_JOB_HEADER_FILE_
#define CURAN_JOB_HEADER_FILE_

#include <string>
#include <functional>

namespace curan {
namespace utilities {

/*
A Job is basically a pointer to a function to be executed 
at a latter point in time. Because we capture a functor
you can capture the necessary variables with a lambda
and from there you can execute the job by calling the 
operator()

The job has a description so that we can query a queue for 
pending jobs, jobs being executed, etc.. This allows us to 
provide feedback to a user about which jobs are running. 

The job class is mostly a class to be used in conjuction 
with the ThreadPool class. Consider the following example 
(not representative of how this class was designed
 to be used because we don't use the ThreadPool class)

int main(){

	std::list<curan::utilities::Job> list_of_jobs;
	int i = 10;
	list_of_jobs.emplace_back("first job",[=](){std::cout << "the captured value is: " << i << std::endl;});
	list_of_jobs.emplace_back("second job",[&](){std::cout << "the captured reference is: " << i << std::endl;});
	i = 20;
	for(auto& job : list_of_jobs)
		job();
};

The snippet of code should print the following

the captured value is: 10
the captured reference is: 20

If we wanted we could query the description of the pending jobs as 


int main(){

	std::list<curan::utilities::Job> list_of_jobs;
	int i = 10;
	list_of_jobs.emplace_back("first job",[=](){std::cout << "the captured value is: " << i << std::endl;});
	list_of_jobs.emplace_back("second job",[&](){std::cout << "the captured reference is: " << i << std::endl;});
	i = 20;
	for(auto& job : list_of_jobs)
		std::cout << "job in queue has the description :" << job.description() << std::endl;
};

This little class is expressive in name and associates a description to future work

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
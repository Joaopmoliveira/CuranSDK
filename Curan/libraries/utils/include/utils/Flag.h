#ifndef CURAN_FLAG_HEADER_FILE_
#define CURAN_FLAG_HEADER_FILE_

#include <mutex>
#include <memory>
#include <condition_variable>

namespace curan {
namespace utilities {

/*
Thread safe flag which can be used to wait
for a given event from multiple threads. When
created the boolean value is set to false.


Consider the following scenario. We have a function foo
that is running in another thread, this function should only
print to the output stream once the flag is true. The flag class
allows us to wait for events to be triggerred

% This function is as follows

void foo(Flag& f_flag){
	f_flag.wait();
	std::cout << "An event was detected!\n";
}

% we first define our flag and then we lauch the other 
% thread. 

int main(){
	curan::utilities::Flag f_flag;
	std::thread other_thread{[&](){foo(f_flag);}};

% now that we have lauched the other thread
% we can do our heavy activities without worriyng about 
% the other tasks

	for(size_t i = 0; i< 100000 ; ++i){
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

% now that we have finished our activity we can trigger the boolean flag 

	f_flag.set(true);

% which will wake up the wait() method in the foo function
% once the foo function prints the thread returns and we can join again
% with the thread

	other_thread.join();
	return 0;
}

*/

class Flag
{
public:

Flag() : flag_{ false } {}

Flag(bool var) : flag_{ var } {}

Flag(const Flag&) = delete;

Flag& operator=(const Flag&) = delete;


inline void set(bool val){
	std::lock_guard g(mutex_);
	flag_ = val;
	cond_var_.notify_all();
}


inline void wait(){
	std::unique_lock lock(mutex_);
	cond_var_.wait(lock, [this]() { return flag_; });
}

inline bool value() {
	std::lock_guard<std::mutex> g{mutex_};
	return flag_;
}

private:

bool flag_;
std::mutex mutex_;
std::condition_variable cond_var_;
};

}
}

#endif
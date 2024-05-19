#ifndef CURAN_LOCKABLE_HEADER_FILE_
#define CURAN_LOCKABLE_HEADER_FILE_

#include <mutex>

namespace curan {
namespace utilities {

/*
The Lockable class is supposed to be the parent 
of any class that needs to lock its mutex. A simple example 
of the use case of this class is

class NeedsToLock : public Lockable{
	double super_dangerous_number = 0.0;
public:
	void update_number(double num){
		std::lock_guard<std::mutex> g{get_mutex()};
		super_dangerous_number = num;
	}

	double get_number(){
		std::lock_guard<std::mutex> g{get_mutex()};
		return super_dangerous_number;
	}
}

Now this class is safe to be used across threads because we lock access to the
variable both for reading and writing

*/

class Lockable {
	std::mutex mut;

public:

std::mutex& get_mutex() {
	return mut;
}

};

}
}

#endif
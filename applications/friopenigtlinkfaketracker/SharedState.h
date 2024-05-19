#ifndef SHARED_ROBOT_STATE_HEADER_H
#define SHARED_ROBOT_STATE_HEADER_H

#include <array>
#include <memory>
#include <atomic>

/*
This is a class which wraps the atomic behavior we desired. 
We create a shared pointer so that this blob of memory
can be accessed from multiple threads. Whenver the control
thread receives the robot state we update the internal 
of this class with a write call. (This is atomic)
When we want to read from said class and do post processing, 
then we can make a temporary copy with a read call (this is atomic)
and the entire state is syncronized.
The memory order guarantees that we dont wait for longer
than necessary and we allow the compiler to change the 
order of our code however much he decides its necessary to optimize
the code.
To transmit information about the status of the application
the class has a method which can query if this thread should die
or not. 
*/

struct State{
    std::array<double,7> joint_config;
    std::array<double,7> external_torques;
    std::array<double,7> measured_torques;
};

struct SharedState {
    std::atomic<State> robot_state;
};

#endif
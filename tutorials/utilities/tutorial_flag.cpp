/*
Consider the following scenario, you have two threads, which both run simultaneously. You want to pass values from
one thread to the other as fast as possible. First lets include the necessary header files for this experiment.

Firt we include <iostream> to be able to print to the output stream
*/

#include <iostream>

/*
We include <thread> to be able to launch threads from our application
*/
#include <thread>

/*
We include <random> to be able to generate random numbers from our application
*/
#include <random>

/*
Lastly we include the Flag from curan
*/
#include "utils/Flag.h"

/*
Now each thread will run one function called foo and bar. We pass the safe queue by reference to both functions 
*/
int foo(curan::utilities::Flag& queue);
int bar(curan::utilities::Flag& queue);

/*
we make this variable atomic to make sure that both threads manipulate the same variable in memory and not a cached value
*/
std::atomic<bool> variable_to_keep_threads_running = true;

/*
Internally function bar will apply a filter to the passed type, in this case a double, while foo gets values from a sensor, which
in this example will be simulated through a random number generator
*/

int foo(curan::utilities::Flag& to_tring_flag_bar,curan::utilities::Flag& to_wait_flag_foo){
    for(size_t i = 0; i < 10 ; ++i){
        to_tring_flag_bar.trig();
        to_wait_flag_foo.wait();
        std::cout << "push to screen" << std::endl;
    }
    std::cout << "stopping foo" << std::endl;
    return 0;
}

int bar(curan::utilities::Flag& to_wait_flag_bar,curan::utilities::Flag& to_trig_flag_foo){
    for(size_t i = 0; i < 10 ; ++i){
        to_wait_flag_bar.wait();
        std::cout << "priting to screen" << std::endl;
        to_trig_flag_foo.trig();
    }
    std::cout << "stopping bar" << std::endl;
    return 0;
}


int main(){
    curan::utilities::Flag flag_bar;
    curan::utilities::Flag flag_foo;
    std::thread foo_thread{[&](){foo(flag_bar,flag_foo);}};
    std::thread bar_thread{[&](){bar(flag_bar,flag_foo);}};

    std::this_thread::sleep_for(std::chrono::seconds(2));
    variable_to_keep_threads_running = false;
    bar_thread.join();
    foo_thread.join();
    std::cout << "stopping threads" << std::endl;
    return 0;
}
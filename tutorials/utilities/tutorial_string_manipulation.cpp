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
Lastly we include the safequeue from curan
*/
#include "utils/SafeQueue.h"

/*
Now each thread will run one function called foo and bar. We pass the safe queue by reference to both functions 
*/
int foo(curan::utilities::SafeQueue<double>& queue);
int bar(curan::utilities::SafeQueue<double>& queue);

/*
we make this variable atomic to make sure that both threads manipulate the same variable in memory and not a cached value
*/
std::atomic<bool> variable_to_keep_threads_running = true;

/*
Internally function bar will apply a filter to the passed type, in this case a double, while foo gets values from a sensor, which
in this example will be simulated through a random number generator
*/

int foo(curan::utilities::SafeQueue<double>& queue){
    
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(1.0, 2.0);

    while(variable_to_keep_threads_running){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        queue.push(dis(gen));
    }
}

int bar(curan::utilities::SafeQueue<double>& queue){
    double filter_value = 0.0;
    while(variable_to_keep_threads_running){
        auto value = queue.wait_and_pop();
        if(!value)
            continue;
        filter_value = 0.9*filter_value+0.1**value;
        std::printf("sensor: %.4f filtered: %.4f\n",*value,filter_value);
    }
}

/*
The safe queue guarantees that acess to the internal queue is always protected by a mutex, thus we cannot put a value and read it simultaneously. This would be undefined behavior
*/

int main(){
    curan::utilities::SafeQueue<double> queue;

    std::thread foo_thread{[&](){foo(queue);}};
    std::thread bar_thread{[&](){bar(queue);}};

    std::this_thread::sleep_for(std::chrono::seconds(2));
    variable_to_keep_threads_running = false;
    bar_thread.join();
    foo_thread.join();
    
}
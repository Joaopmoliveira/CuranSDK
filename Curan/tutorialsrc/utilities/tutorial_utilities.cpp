// stupid solution

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include "utils/SafeQueue.h"

int foo(std::atomic<bool>& flag_to_signal,double* value);
int bar(std::atomic<bool>& flag_to_signal,double* value);

int foo(curan::utilities::SafeQueue<double>& queue);
int bar(curan::utilities::SafeQueue<double>& queue);

int not_the_best_solution(){
    std::atomic<bool> flag = true;
    double value = 1.0;
    auto callable_foo = [&flag,&value](){
        foo(flag,&value);
    };
    std::thread foo_thread{callable_foo};
    bar(flag,&value);
    foo_thread.join();
    return 0;
};

int the_curan_solution(){
    curan::utilities::SafeQueue<double> queue;
    queue.push(1.0);
    auto callable_foo = [&queue](){
        foo(queue);
    };
    std::thread foo_thread{callable_foo};
    bar(queue);
    foo_thread.join();
    return 0;
}

int main(){
    return the_curan_solution();
};

int foo(std::atomic<bool>& flag_to_signal,double* value){
    char input;
    while(true){
        std::cin >> input; //this call blocks until input is provided
        switch(input){
            case 'a': //agressive -> gain 3
            {
                flag_to_signal.store(true);
                *value = 3;
            }
            break;
            case 's': //smooth -> gain 1
            {
                flag_to_signal.store(true);
                *value = 1;
            }
            break;
            case 'x': //stop control
            {
                flag_to_signal.store(true);
                *value = 0;
                return 0;
            }
            break;
            default: //do nothing
            break;
        }
    }
};

int foo(curan::utilities::SafeQueue<double>& queue){
    char input;
    while(true){
        std::cin >> input; //this call blocks until input is provided
        switch(input){
            case 'a': //agressive -> gain 3
            {
                queue.push(3.0);
            }
            break;
            case 's': //smooth -> gain 1
            {
                queue.push(1.0);
            }
            break;
            case 'x': //stop control
            {
                queue.push(0.0);
                return 0;
            }
            break;
            default: //do nothing
            break;
        }
    }
};

int bar(std::atomic<bool>& flag_to_signal,double* value){
    while(true){
        if(flag_to_signal){
            flag_to_signal = false;
            std::cout << "gain: " << *value << std::endl;
            if(*value==0){ //x was pressed we need to stop
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    };
    return 1;
};

int bar(curan::utilities::SafeQueue<double>& queue){
    while(true){
        double val = 0.0;
        if(queue.wait_and_pop(val) && val== 0){
            std::cout << "gain: " << val << std::endl;
            break;
        }
        std::cout << "gain: " << val << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    };
    return 1;
};

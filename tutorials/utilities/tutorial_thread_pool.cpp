/*
Consider the following scenario, you want to parallelize a set of tasks. But because this parallelization 
happens every so ofter, creating and destroyng a thread is an expensive task. Therefore we must preallocaate 
all the threads we wish to use in the future, and then find a mecanism to post asyncrounously work into this 
thread pool. 
*/

#include <iostream>

/*
We include <random> to be able to generate random numbers from our application
*/
#include <random>

/*
Lastly we include the TheadPool from curan
*/
#include "utils/TheadPool.h"

/*
Our goal is to execute a number of tasks, in this case to process a string and count the number of characters 'a'
 in parallel with distinct behaviors:

First we write the function that generates random strings (code from) 
https://stackoverflow.com/questions/440133/how-do-i-create-a-random-alpha-numeric-string-in-c
*/

std::string random_string( size_t length )
{
    auto randchar = []() -> char
    {
        const char charset[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[ rand() % max_index ];
    };
    std::string str(length,0);
    std::generate_n( str.begin(), length, randchar );
    return str;
}

/*
The function we execute in parallel is the following
*/

int count_number_of_equal_chars(const std::string& s){
    int n = 0;
    std::for_each(s.begin(),s.end(),[&](char cc){ if(cc=='a')   ++n;});
    return n;
}

/*
Now we have the first function that submits some amount of work and then returns as quicly as possible
*/

int return_as_quicly_as_possible(){
    std::cout << "starting evaluation:\n";
    constexpr size_t number_of_strings = 400;
    std::vector<std::string> random_strings;
    random_strings.reserve(number_of_strings);
    for(size_t i = 0; i< number_of_strings; ++i)
        random_strings.push_back(random_string(40));
    auto pool = curan::utilities::ThreadPool::create(4,curan::utilities::RETURN_AS_FAST_AS_POSSIBLE);
    for(const auto& str : random_strings)
        pool->submit(curan::utilities::Job{"job to solve",[&](){  std::cout << "string (" << str << ") number of \'a\' chars : " << count_number_of_equal_chars(str) << std::endl;}});
    //when we arrive here we ignore all pending work and return 
    return 0;
}

/*
Now we have the first function that submits some amount of work and then returns only when all work is finished
*/

int return_once_all_work_is_finished(){
    std::cout << "starting evaluation:\n";
    constexpr size_t number_of_strings = 400;
    std::vector<std::string> random_strings;
    random_strings.reserve(number_of_strings);
    for(size_t i = 0; i< number_of_strings; ++i)
        random_strings.push_back(random_string(40));
    auto pool = curan::utilities::ThreadPool::create(4,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
    for(const auto& str : random_strings)
        pool->submit(curan::utilities::Job{"job to solve",[&](){  std::cout << "string (" << str << ") number of \'a\' chars : " << count_number_of_equal_chars(str) << std::endl;}});
    //when we arrive here the destructor of ThreadPool blocks until all pending work has been solved
    return 0;
}

int main(){
    return_as_quicly_as_possible();
    return_once_all_work_is_finished();
    return 0;
}
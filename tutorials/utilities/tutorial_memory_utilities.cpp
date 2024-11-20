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
#include "utils/MemoryUtils.h"

/*
Its important to note that the memory related manipulation of data is not as efficient as a dedicated program designed from scratch for a particular 
purpose. If you require absolute control over memory allocation, you need to avoid these classes. 

The main purpose of the memory utilities is to take data from others APIs and allow it to be propagated inside curan
as a simple memory buffer. Fundamentaly we use the memory buffers from ASIO because its the main library that uses this concept, although
other libraries also sporadically use these concepts. 
*/

/*
Consider that you are using an API that implements its own smart pointers. You want to send this blob of memory through a
Client or Server. 
*/

int main(){

    return 0;
}
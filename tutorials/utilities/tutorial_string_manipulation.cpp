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
#include "utils/StringManipulation.h"


int main(){
    double a = 4321.123456789123456789;
    std::cout << "double with precision 1: " << curan::utilities::to_string_with_precision(a,1) << std::endl;
    std::cout << "double with precision 2: " << curan::utilities::to_string_with_precision(a,2)<< std::endl;
    std::cout << "double with precision 3: " << curan::utilities::to_string_with_precision(a,3)<< std::endl;
    std::cout << "double with precision 4: " << curan::utilities::to_string_with_precision(a,4)<< std::endl;
    std::cout << "double with precision 5: " << curan::utilities::to_string_with_precision(a,5)<< std::endl;
    std::cout << "double with precision 6: " << curan::utilities::to_string_with_precision(a,6)<< std::endl;
    std::cout << "double with precision 7: " << curan::utilities::to_string_with_precision(a,7)<< std::endl;
    std::cout << "double with precision 8: " << curan::utilities::to_string_with_precision(a,8)<< std::endl;
    std::cout << "double with precision 9: " << curan::utilities::to_string_with_precision(a,9)<< std::endl;
    std::cout << "double with precision 10: " << curan::utilities::to_string_with_precision(a,10)<< std::endl;
    return 0;
}
#include "utils/CircularBuffer.h"
#include <iostream>

int main(){
    curan::utilities::CircularBuffer<double> buffer{4};

    auto cprint = [&](){
        for(const auto & val : buffer)
            std::cout << val << " ";
        std::cout << std::endl;
    };

    buffer.put(10.0);
    cprint();
    buffer.put(20.0); 
    cprint();
    buffer.put(30.0);
    cprint();
    buffer.put(40.0);
    cprint();
    buffer.put(50.0);
    cprint();
    buffer.put(60.0);
    cprint();
    return 0;
}
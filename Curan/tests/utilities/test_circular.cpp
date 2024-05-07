#include "utils/CircularBuffer.h"
#include <iostream>
#include <array>

int main(){
    curan::utilities::CircularBuffer<double> buffer{4};

    auto printlayout = [&](){
        std::cout << "->raw: ";
        for(const auto & val : buffer)
            std::cout << val << " ";
        std::cout << std::endl;
    };

    auto printlinearlayout = [&](){
        std::cout << "->linear: ";
        for(const auto & val : buffer.linear_view())
            std::cout << val << " ";
        std::cout << std::endl;
    };

    std::array<double,10> r;
    size_t i = 0;
    for(auto& v : r){
        v = i;
        ++i;
    }

    for(auto v : r){
        std::cout << "\n=========\n";
        buffer.put(std::move(v));
        printlayout();
        printlinearlayout();
    }
    return 0;
}
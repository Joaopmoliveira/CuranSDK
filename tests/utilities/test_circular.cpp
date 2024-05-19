#include "utils/CircularBuffer.h"
#include <iostream>
#include <array>

int main(){
    curan::utilities::CircularBuffer<double> buffer{4};

    buffer.operate([](double& po){ po = -1.0;});

    std::cout << "linear view size: " << buffer.linear_view().size() << "\n";

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
    double i = 0;
    for(auto& v : r){
        v = i;
        ++i;
    }

    for(auto v : r){
        std::cout << "\n=========\n";
        std::cout << "header before : " << buffer.head();
        buffer.put(std::move(v));
        std::cout << " header after : " << buffer.head();
        printlayout();
        printlinearlayout();
    }
    return 0;
}
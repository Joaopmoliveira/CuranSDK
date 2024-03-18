#include <thread>
#include <chrono>
#include <iostream>

int main(){
    std::cout << "in\n";
    for(size_t i = 0; i < 10; ++i){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "running...\n";
    } 
    std::cout << "out\n";
    return 0;
}
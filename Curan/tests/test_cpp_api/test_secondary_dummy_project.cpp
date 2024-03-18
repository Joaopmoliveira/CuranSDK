#include <thread>
#include <chrono>
#include <iostream>

int main(){
    std::cout << "in\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    std::cout << "out\n";
    return 0;
}
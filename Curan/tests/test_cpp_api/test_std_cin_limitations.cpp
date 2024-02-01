#include <iostream>
#include <thread>
#include <print>

int main(){

    std::cout << "going to sleep\n";
    std::thread t{[](){
        std::cout << "entering the long wait\n";
        std::string s;
        std::cin >> s;
        std::cout << "exception thrown\n";
    }};
    t.detach();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    return 0;
}
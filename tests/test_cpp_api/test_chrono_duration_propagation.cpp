#include <iostream>
#include <chrono>
#include <thread>

struct A{
    std::chrono::nanoseconds duration;
    
    template <class _Rep, class _Period>
    A(const std::chrono::duration<_Rep, _Period>& _Rel_time){
        duration = std::chrono::duration_cast<std::chrono::nanoseconds>(_Rel_time);
    }
};


void boo(){
    std::cout << "hello\n";
}

int main(){
    A a{std::chrono::microseconds(1)};
    std::cout << "duration in " << std::chrono::duration_cast<std::chrono::nanoseconds>(a.duration).count();
    return 0;
}
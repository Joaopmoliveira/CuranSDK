#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include "utils/DateManipulation.h"

int main(){
    try{
        std::cout << curan::utilities::formated_date<std::chrono::system_clock>(std::chrono::system_clock::time_point(std::chrono::system_clock::duration(0))) << std::endl;

        std::cout << curan::utilities::formated_date<std::chrono::system_clock>(std::chrono::system_clock::time_point(std::chrono::system_clock::duration(std::chrono::seconds(1000000000)))) << std::endl;

        std::cout << curan::utilities::formated_date<std::chrono::system_clock>(std::chrono::system_clock::time_point(std::chrono::system_clock::duration(1000000000))) << std::endl;

    } catch (std::runtime_error &e){
        std::cout << "exception: " << e.what() << std::endl;
        return 2;
    } catch (...) {
        std::cout << "generic failure" << std::endl;
        return 1;
    }
    return 0;
}
#include "utils/StringManipulation.h"
#include <iostream>
#include <array>

int main(){
    std::cout << "the transformed string is: " << curan::utilities::to_string_with_precision(123456789012.142857142857142849212692681248881854116916656494140625,3) << std::endl;
    return 0;
}
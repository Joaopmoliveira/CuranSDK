
#include <iostream>

void foo(double in){
    in += 1.0;
}

void bar(double* in){
    *in += 1.0;
}

void zeta(double& in){
    in += 1.0;
}

int main(){
    double value = 1.0;
    std::printf("value is %f",value);
    foo(value);
    std::printf("value is %f",value);
    double* adress_of_value = &value;
    bar(adress_of_value);
    std::printf("value is %f",value);
    zeta(value);
    std::printf("value is %f",value);
    return 0;
}
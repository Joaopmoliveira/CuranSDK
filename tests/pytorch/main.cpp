#include <torch/torch.h>
#include <iostream>

int main() {
    std::cout << "hi mark" << std::endl;
    
try{
    torch::Tensor tensor = torch::eye(3);
    std::cout << tensor << std::endl;
    return 0;
} catch(std::runtime_error& e){
    std::cout << "exception: " << e.what() << std::endl;
    return 1;
} catch(...){
     std::cout << "unhandled exception: " << std::endl;
     return 2;
}

}
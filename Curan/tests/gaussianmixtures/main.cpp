#include "gaussianmixtures/GMR.h"
#include <fstream>
#include <iostream>

constexpr size_t in_size = 2;
constexpr size_t out_size = 2;

int main(){
    curan::gaussian::GMR<in_size,out_size> model;
    std::ifstream modelfile{CURAN_COPIED_RESOURCE_PATH"/gaussianmixtures_testing/mymodel.txt"};
    modelfile >> model;
    for(size_t i = 0; i< model.components(); ++i){
        std::printf("\n==============\nComponent %ld\n",i);
        std::printf("==============\n-Prior %f\n",model.priork[i]);
        std::printf("\n==============\n-Mu \n==============\n");
        std::cout << model.muk[i];
        std::printf("\n==============\n-Sigma\n==============\n");
        std::cout << model.invSigmak[i].inverse(); 
        std::printf("\n==============\n-Ak\n==============\n");
        std::cout << model.Ak[i];    
        std::printf("\n==============\n-bk\n==============\n");
        std::cout << model.bk[i];   
    }

    std::ifstream modelfile{CURAN_COPIED_RESOURCE_PATH"/gaussianmixtures_testing/mymodel.txt"};

    Eigen::Matrix<double,in_size,100> inputs = Eigen::Matrix<double,in_size,100>::Random();
    for(const auto& in : inputs.colwise())
        std::cout << "Input\n" << in << "\nOutput\n" << model.likeliest(in) << "\n";
    return 0;
}
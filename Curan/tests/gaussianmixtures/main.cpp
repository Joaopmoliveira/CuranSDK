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
        std::printf("Component %ld\n",i);
        std::printf("\tPrior %ld\n",model.priork[i]);
        std::printf("\tMu \n");
        std::cout << model.muk[i];
        std::printf("\tSigma\n");
        std::cout << model.invSigmak[i].inverse(); 
        std::printf("\tAk\n");
        std::cout << model.Ak[i];    
        std::printf("\tbk\n");
        std::cout << model.bk[i];   
    }
    return 0;
}
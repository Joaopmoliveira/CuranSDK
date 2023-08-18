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

    std::ifstream testfile{CURAN_COPIED_RESOURCE_PATH"/gaussianmixtures_testing/testmymodel.txt"};
    nlohmann::json testing = nlohmann::json::parse(testfile);
    size_t number_of_tests = testing["nTests"];
    double total_error = 0.0;
    double max_error = -100000.0;
    int iter = -1;
    for(size_t it = 0; it < number_of_tests ; ++it){
        nlohmann::json test =  testing["test"+std::to_string(it+1)];
        std::stringstream s;
        std::string input = test["input"];
        s << input;
        auto InputMat = curan::utilities::convert_matrix(s);
        std::string output = test["output"];
        s = std::stringstream{};
        s << output;
        auto ExpectedOutputMat = curan::utilities::convert_matrix(s);
        auto ConcreteOutput = model.likeliest(InputMat);
        std::cout << "\nInput\n" << InputMat << "\nCpp Output\n" << ConcreteOutput << "\nReal Output\n" << ExpectedOutputMat << "\n";
        double local_error = (ExpectedOutputMat-ConcreteOutput).norm();
        total_error += local_error;
        std::cout << "Error: " << local_error << "\n";
        if(local_error > max_error){
            std::cout << "Max found " << it;
            iter = it;
            max_error = local_error;
        }
    }
    std::printf("\nTotal Error : %f\nAverage Error: %f\nMax Error: %f\n",total_error,total_error/number_of_tests,max_error);
    std::printf("\niteration of maximum %d \n",iter);
    return 0;
}
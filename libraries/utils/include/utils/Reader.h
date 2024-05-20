#ifndef CURAN_READER_HEADER_FILE_
#define CURAN_READER_HEADER_FILE_

#include "Eigen/Dense"
#include <sstream>

namespace curan {
namespace utilities {

/*
This is a simple utility function to convert from a stringstream into an Eigen Matrix, which 
is useful when interacting between different programs or between Python<--->C++ or Matlab<--->C++

the function only converts strings into 2D matrices of any size. Optionally you can provide the
delimiter which signals the mechanics of the function that you are changing lines 

Consider the following 

int main(){
    std::stringstream s = "0.01 0.01 0.01; 0.01 0.01 0.1 : 1.0 1.0 1.0"
    Eigen::MatrixXd matrix = convert_matrix(std::stringstream& data,';');
    std::cout << matrix << std::endl;
}
*/

Eigen::MatrixXd convert_matrix(std::stringstream& data, char var = ' ');

}
}

#endif
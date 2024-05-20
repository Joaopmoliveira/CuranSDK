#ifndef CURAN_CONVERT_ARRAY_TO_EIGEN_
#define CURAN_CONVERT_ARRAY_TO_EIGEN_

#include <Eigen/Dense>

namespace curan {
namespace robotic {

template<typename numeric_type,size_t rows>
Eigen::Matrix<numeric_type,rows,1> convert(const std::array<numeric_type,rows>& value){
    Eigen::Matrix<numeric_type,rows,1> converted;
    for(size_t row = 0; row < rows; ++row)
        converted[row] = value[row];
    return converted;
}

template<typename numeric_type,size_t rows,size_t cols>
Eigen::Matrix<numeric_type,rows*cols,1> convert(const std::array<std::array<numeric_type,rows>,cols>& value){
    Eigen::Matrix<numeric_type,rows*cols,1> converted;
    size_t linear_index = 0;
    for(size_t col = 0; col < cols; ++col)
        for(size_t row = 0; row < rows; ++row){
            converted[linear_index] = value[col][row];
            ++linear_index;
        }   
    return converted;
}

template<typename numeric_type,size_t rows>
std::array<numeric_type,rows> convert(const Eigen::Matrix<numeric_type,rows,1>& value){
    std::array<numeric_type,rows> converted;
    for(size_t row = 0; row < rows; ++row)
        converted[row] = value[row];
    return converted;
}


}
}

#endif
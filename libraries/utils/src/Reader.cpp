#include "utils/Reader.h"

namespace curan {
namespace utilities {

Eigen::MatrixXd convert_matrix(std::stringstream& data, char separation_char)
{
    // the matrix entries are stored in this variable row-wise. For example if we have the matrix:
    // M=[a b c 
    //    d e f]
    // the entries are stored as matrixEntries=[a,b,c,d,e,f], that is the variable "matrixEntries" is a row vector
    // later on, this vector is mapped into the Eigen matrix format
    std::vector<double> matrixEntries;
 
    // this variable is used to store the row of the matrix that contains commas 
    std::string matrixRowString;
 
    // this variable is used to store the matrix entry;
    std::string matrixEntry;
 
    // this variable is used to track the number of rows
    int matrixRowNumber = 0;
    size_t first_num_cols = 0;
    bool should_initialize = true;
    size_t previous_num_cols = 0;
    auto tmp = data.str();
    std::string cpy;
    std::unique_copy(tmp.begin(), tmp.end(), std::back_insert_iterator<std::string>(cpy),
                    [](char &x, char &y) {
                        return iswspace(x) && iswspace(y);
                    });
    data.str(cpy);
    
    while(!data.eof()){
        getline(data, matrixRowString);
        if(matrixRowString.size()<1)
            continue;
        size_t num_cols = 0;
        std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.
        while (!matrixRowStringStream.eof()) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
        {
            getline(matrixRowStringStream, matrixEntry, separation_char);
            if(!matrixEntry.size()){
                continue;
            }
            matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
            ++num_cols;
        }
        if(should_initialize){
            previous_num_cols = num_cols;
            should_initialize = false;
        }
        if(previous_num_cols!=num_cols){
            throw std::runtime_error("inconsistent number of cols");
        }
            
        matrixRowNumber++; //update the row numbers
    } 

    // here we convet the vector variable into the matrix and return the resulting object, 
    // note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
    if(!matrixRowNumber)
        throw std::runtime_error("no rows read");
    size_t rows = matrixRowNumber;
    size_t cols = matrixEntries.size() / matrixRowNumber;
    if(rows*cols!=matrixEntries.size())
        throw std::runtime_error("incorrect number of rows and cols");
    return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

}
}
#ifndef CURAN_READER_HEADER_FILE_
#define CURAN_READER_HEADER_FILE_

#include "Eigen/Dense"
#include <sstream>

namespace curan {
namespace utilities {

Eigen::MatrixXd convert_matrix(std::stringstream& data, char var = ' ');

}
}

#endif
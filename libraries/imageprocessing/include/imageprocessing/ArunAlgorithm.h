#ifndef CURAN_ARUN_ALGORITHM_HEADER_FILE_
#define CURAN_ARUN_ALGORITHM_HEADER_FILE_

#include <Eigen/Dense>

namespace curan{
namespace image{

Eigen::Matrix<double,3,Eigen::Dynamic> kmeans(const  Eigen::Matrix<double,3,Eigen::Dynamic>& points , size_t nclusters);

std::tuple<Eigen::Matrix<double,4,4>,double> arun_estimate(const Eigen::Matrix<double,3,Eigen::Dynamic>& fixed_point_cloud, const Eigen::Matrix<double,3,Eigen::Dynamic>& moving_point_cloud);

std::array<std::tuple<Eigen::Matrix<double, 4, 4>, double>, 6> extract_potential_solutions(const Eigen::Matrix<double, 3, Eigen::Dynamic> &fixed_cloud,const  Eigen::Matrix<double, 3, Eigen::Dynamic> &moving_cloud,size_t number_of_clusters);

}
}

#endif
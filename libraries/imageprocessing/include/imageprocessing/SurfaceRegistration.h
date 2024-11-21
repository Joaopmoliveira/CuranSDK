#ifndef CURAN_SURFACE_REGISTRATION_HEADER_FILE_
#define CURAN_SURFACE_REGISTRATION_HEADER_FILE_

#include <Eigen/Dense>
#include <tuple>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "ArunAlgorithm.h"

namespace curan{
namespace image{

enum Units{
    METERS,
    MILLIMETERS,
    CENTIMETERS
};

Eigen::Matrix<double,4,4> solve_registration_problem(Units point_cloud_units,const std::tuple<pcl::PointCloud<pcl::PointXYZ>::ConstPtr,Eigen::Matrix<double,3,Eigen::Dynamic>>& fixed_point_cloud,const std::tuple<pcl::PointCloud<pcl::PointXYZ>::ConstPtr,Eigen::Matrix<double,3,Eigen::Dynamic>> moving_point_cloud, size_t num_clusters);

}
}

#endif
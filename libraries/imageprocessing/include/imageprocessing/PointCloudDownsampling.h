#ifndef CURAN_POINTCLOUD_DOWNSAMPLING_HEADER_FILE_
#define CURAN_POINTCLOUD_DOWNSAMPLING_HEADER_FILE_

#include <Eigen/Dense>
#include <tuple>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

namespace curan{
namespace image{

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr,Eigen::Matrix<double, 3, Eigen::Dynamic>> convertEigenToPCLPointCloudWithSpacingVoxelGrid(const Eigen::Matrix<double, 3, Eigen::Dynamic> &eigenPoints, double spacing);

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr,Eigen::Matrix<double, 3, Eigen::Dynamic>> convertEigenToPCLPointCloudWithSpacingUniform(const Eigen::Matrix<double, 3, Eigen::Dynamic> &eigenPoints, double spacing);

}
}

#endif
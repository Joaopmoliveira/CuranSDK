
#include "imageprocessing/PointCloudDownsampling.h"

namespace curan {
namespace image {

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr,Eigen::Matrix<double, 3, Eigen::Dynamic>> convertEigenToPCLPointCloudWithSpacingVoxelGrid(const Eigen::Matrix<double, 3, Eigen::Dynamic> &eigenPoints, double spacing)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_intermediate(new pcl::PointCloud<pcl::PointXYZ>());

    cloud_intermediate->width = eigenPoints.cols();
    cloud_intermediate->height = 1;
    cloud_intermediate->points.resize(eigenPoints.cols());
    for (size_t i = 0; i < eigenPoints.cols(); ++i)
    {
        cloud_intermediate->points[i].x = eigenPoints(0, i);
        cloud_intermediate->points[i].y = eigenPoints(1, i);
        cloud_intermediate->points[i].z = eigenPoints(2, i);
    }

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_intermediate);
    sor.setLeafSize(spacing, spacing, spacing);
    sor.filter(*cloudout);

    Eigen::Matrix<double, 3, Eigen::Dynamic> outEigenPoints = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3,cloudout->width);
    for (size_t i = 0; i < cloudout->width; ++i) {
        outEigenPoints(0, i) = cloudout->points[i].x;
        outEigenPoints(1, i) = cloudout->points[i].y;
        outEigenPoints(2, i) = cloudout->points[i].z;
    }


    return {cloudout,outEigenPoints};
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr,Eigen::Matrix<double, 3, Eigen::Dynamic>> convertEigenToPCLPointCloudWithSpacingUniform(const Eigen::Matrix<double, 3, Eigen::Dynamic> &eigenPoints, double spacing)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_just_points(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_just_points->width = eigenPoints.cols();
    cloud_just_points->height = 1;
    cloud_just_points->points.resize(eigenPoints.cols());
    for (size_t i = 0; i < eigenPoints.cols(); ++i)
    {
        cloud_just_points->points[i].x = eigenPoints(0, i);
        cloud_just_points->points[i].y = eigenPoints(1, i);
        cloud_just_points->points[i].z = eigenPoints(2, i);
    }

    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(cloud_just_points);
    uniform_sampling.setRadiusSearch(spacing);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud;
    uniform_sampling.filter(*downsampled_cloud);

    Eigen::Matrix<double, 3, Eigen::Dynamic> outEigenPoints = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3,downsampled_cloud->width);
    for (size_t i = 0; i < downsampled_cloud->width; ++i) {
        outEigenPoints(0, i) = downsampled_cloud->points[i].x;
        outEigenPoints(1, i) = downsampled_cloud->points[i].y;
        outEigenPoints(2, i) = downsampled_cloud->points[i].z;
    }

    return {downsampled_cloud,outEigenPoints};
}

}
}
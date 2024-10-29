#include "utils/Reader.h"
#include <Eigen/Dense>
#include <iostream>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

void

print4x4Matrix (const Eigen::Matrix4d & matrix)

{

  printf ("Rotation matrix :\n");

  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));

  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));

  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));

  printf ("Translation vector :\n");

  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));

}

Eigen::Matrix<double, 3, Eigen::Dynamic> read_point_cloud(const std::string &path)
{
    std::vector<Eigen::Matrix<double, 3, 1>> vectors_in_file;
    std::ifstream input{path};
    if (!input.is_open())
        throw std::runtime_error("failure to read point cloud");
    std::stringstream s;
    s << input.rdbuf();
    input.close();
    auto points_in_file = curan::utilities::convert_matrix(s, ',');

    std::printf("size of point cloud: (%d %d)\n", (int)points_in_file.rows(),
                (int)points_in_file.cols());

    if (points_in_file.rows() != 3)
    {
        throw std::runtime_error("the point cloud has an incorrect size");
    }
    Eigen::Matrix<double, 3, Eigen::Dynamic> point_cloud_eigen = points_in_file;
    return point_cloud_eigen;
}

void write_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, const std::string& path)
{
    // Create an Eigen matrix to hold the data
    Eigen::Matrix<double, 3, Eigen::Dynamic> eigen_matrix(3, point_cloud->size());

    // Fill the Eigen matrix with point cloud data
    for (size_t i = 0; i < point_cloud->size(); ++i)
    {
        eigen_matrix(0, i) = static_cast<double>(point_cloud->points[i].x);
        eigen_matrix(1, i) = static_cast<double>(point_cloud->points[i].y);
        eigen_matrix(2, i) = static_cast<double>(point_cloud->points[i].z);
    }

    std::ofstream out{path};
    if(!out.is_open())
        throw std::runtime_error("failure to open output file");

    out << eigen_matrix;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr convertEigenToPCLPointCloudWithSpacingVoxelGrid(const Eigen::Matrix<double, 3, Eigen::Dynamic> &eigenPoints,double spacing)
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

    return cloudout;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertEigenToPCLPointCloudWithSpacingUniform(const Eigen::Matrix<double, 3, Eigen::Dynamic> &eigenPoints,double spacing)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_just_points( new pcl::PointCloud<pcl::PointXYZ>());
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud( new pcl::PointCloud<pcl::PointXYZ>());
    uniform_sampling.filter(*downsampled_cloud);

    return downsampled_cloud;
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertEigenToPCLPointCloudWithSpacingVoxelGrid(read_point_cloud(CURAN_COPIED_RESOURCE_PATH"/sentinels/sentinel_front_cross.txt"),4);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_uni = convertEigenToPCLPointCloudWithSpacingUniform(read_point_cloud(CURAN_COPIED_RESOURCE_PATH"/sentinels/sentinel_front_cross.txt"),4);
    write_point_cloud(cloud,CURAN_COPIED_RESOURCE_PATH"/cross_with_voxel_grid.txt");
    write_point_cloud(cloud_uni,CURAN_COPIED_RESOURCE_PATH"/cross_with_uniform_grid.txt");

    int iterations = 100; // Default number of ICP iterations

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(cloud);
    icp.setInputTarget(cloud_uni);

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;

    icp.align(aligned_cloud);

    if (icp.hasConverged())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        std::cout << "\nICP transformation " << iterations<< " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix(transformation_matrix);
    }
    else
    {
        PCL_ERROR("\nICP has not converged.\n");
        return (-1);
    }

    return 0;
}
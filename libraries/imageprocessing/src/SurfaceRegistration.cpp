#include "imageprocessing/SurfaceRegistration.h"

namespace curan{
namespace image{


Eigen::Matrix<double,4,4> solve_registration_problem(Units point_cloud_units,const std::tuple<pcl::PointCloud<pcl::PointXYZ>::ConstPtr,Eigen::Matrix<double,3,Eigen::Dynamic>>& fixed_point_cloud,const std::tuple<pcl::PointCloud<pcl::PointXYZ>::ConstPtr,Eigen::Matrix<double,3,Eigen::Dynamic>> moving_point_cloud, size_t num_clusters){
    const auto& [plc_compt_fixed_cloud,eigen_fixed_cloud] = fixed_point_cloud;
    const auto& [plc_compt_moving_cloud,eigen_moving_cloud] = moving_point_cloud;
    auto ordered_solutions = extract_potential_solutions(eigen_fixed_cloud, eigen_moving_cloud, num_clusters);

    Eigen::Matrix<double,4,4> T_to_centroid_moving = Eigen::Matrix<double,4,4>::Identity();

    Eigen::Matrix<double, 3, 27> angle_array_around_estimated_solution;
    angle_array_around_estimated_solution << -10*0.01745329252, -10*0.01745329252, -10*0.01745329252, 0, 0, 0, 10*0.01745329252, 10*0.01745329252, 10*0.01745329252,
      -10*0.01745329252, -10*0.01745329252, -10*0.01745329252, 0, 0, 0, 10*0.01745329252, 10*0.01745329252, 10*0.01745329252, -10*0.01745329252, -10*0.01745329252, -10*0.01745329252, 0, 0, 0, 10*0.01745329252, 10*0.01745329252, 10*0.01745329252,
      -10*0.01745329252, 0, 10*0.01745329252, -10*0.01745329252, 0, 10*0.01745329252, -10*0.01745329252, 0, 10*0.01745329252, -10*0.01745329252, 0, 10*0.01745329252, -10*0.01745329252, 0, 10*0.01745329252, -10*0.01745329252, 0, 10*0.01745329252,
      -10*0.01745329252, 0, 10*0.01745329252, -10*0.01745329252, 0, 10*0.01745329252, -10*0.01745329252, 0, 10*0.01745329252, -10*0.01745329252, -10*0.01745329252, -10*0.01745329252, -10*0.01745329252, -10*0.01745329252, -10*0.01745329252, -10*0.01745329252,
      -10*0.01745329252, -10*0.01745329252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10*0.01745329252, 10*0.01745329252, 10*0.01745329252, 10*0.01745329252, 10*0.01745329252, 10*0.01745329252, 10*0.01745329252, 10*0.01745329252, 10*0.01745329252;

    int iterations = 100;

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;

    std::vector<std::tuple<Eigen::Matrix<double,4,4>,double>> icp_solutions;

    auto homogenenous_from_vec = [](Eigen::Matrix<double,3,1> euler_angles){
        Eigen::Matrix<double,4,4> hom = Eigen::Matrix<double,4,4>::Identity();
        double t1 = cos(euler_angles[2]);
        double t2 = sin(euler_angles[2]);
        double t3 = cos(euler_angles[1]);
        double t4 = sin(euler_angles[1]);
        double t5 = cos(euler_angles[0]); 
        double t6 = sin(euler_angles[0]);

        hom(0,0) = t1 * t3;
        hom(1,0) = t2 * t3;
        hom(2,0) = -t4;

        hom(0,1) = t1 * t4 * t6 - t2 * t5;
        hom(1,1) = t1 * t5 + t2 * t4 * t6;
        hom(2,1) = t3 * t6;

        hom(0,2) = t2 * t6 + t1 * t4 * t5;
        hom(1,2) = t2 * t4 * t5 - t1 * t6;
        hom(2,2) = t3 * t5;
        return hom;
    };

    for(const auto& [transform,cost] : ordered_solutions){
        for(Eigen::Matrix<double,3,1> euler_angles : angle_array_around_estimated_solution.colwise()){
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setRANSACOutlierRejectionThreshold(0.03);
            icp.setMaximumIterations(iterations);
            icp.setInputSource(plc_compt_moving_cloud);
            icp.setInputTarget(plc_compt_fixed_cloud);
            icp.setMaxCorrespondenceDistance(0.01);
            icp.setTransformationEpsilon(1e-8);
            T_to_centroid_moving.block<3,1>(0,3) = transform.block<3,3>(0,0)*eigen_moving_cloud.rowwise().mean()+transform.block<3,1>(0,3);
            icp.align(aligned_cloud,(T_to_centroid_moving*homogenenous_from_vec(euler_angles)*T_to_centroid_moving.inverse()*transform).cast<float>());
            
            if (icp.hasConverged())
                icp_solutions.emplace_back(icp.getFinalTransformation().cast<double>(),icp.getFitnessScore(100));
            else 
                icp_solutions.emplace_back(icp.getFinalTransformation().cast<double>(),1.0e10);
        }
    }

    std::sort(icp_solutions.begin(), icp_solutions.end(),
            [](std::tuple<Eigen::Matrix<double, 4, 4>, double> &a,
               std::tuple<Eigen::Matrix<double, 4, 4>, double> &b) {
              return std::get<1>(a) < std::get<1>(b);
            });
    return std::get<0>(icp_solutions[0]);
}

}
}
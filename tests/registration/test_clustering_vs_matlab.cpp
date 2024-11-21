#include "utils/Reader.h"
#include <Eigen/Dense>
#include <iostream>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <random>
#include <pcl/registration/incremental_registration.h>


#include <fstream>

#include <algorithm>
#include <random>

Eigen::Matrix<double, 3, Eigen::Dynamic>
read_point_cloud(const std::string &path,double scaling_factor = 1.0) {
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

  Eigen::Matrix<double, 3, Eigen::Dynamic> point_cloud_eigen;
  if (points_in_file.rows() != 3 && points_in_file.cols()==3) {
    point_cloud_eigen = points_in_file.transpose();
  } else if (points_in_file.cols() != 3 && points_in_file.rows()==3) {
    point_cloud_eigen = points_in_file;
  } else {
    throw std::runtime_error("the point cloud has an incorrect size");
  }
  point_cloud_eigen *= scaling_factor;
  return point_cloud_eigen;
}

void write_point_cloud(Eigen::Matrix<double, 3, Eigen::Dynamic>& rotated_point_cloud,const std::string &path) {
  std::ofstream out{path};
  if (!out.is_open())
    throw std::runtime_error("failure to open output file");

  out << rotated_point_cloud;
};

std::tuple<double, Eigen::Matrix<size_t, Eigen::Dynamic, 1>,
           Eigen::Matrix<size_t, Eigen::Dynamic, 1>>
selmin(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &distances) {
  Eigen::Matrix<size_t, Eigen::Dynamic, 1> correspondance =
      Eigen::Matrix<size_t, Eigen::Dynamic, 1>::Zero(distances.cols(), 1);
  Eigen::Matrix<size_t, Eigen::Dynamic, 1> bins =
      Eigen::Matrix<size_t, Eigen::Dynamic, 1>::Zero(distances.rows(), 1);
  double cost = 0;
  for (size_t i = 0; i < distances.cols(); ++i) {
    Eigen::Index minRow, minCol;
    double minimum = distances.col(i).minCoeff(&minRow, &minCol);
    correspondance[i] = minRow;
    cost += minimum;
    bins[minRow] = bins[minRow] + 1;
  }
  return {cost, correspondance, bins};
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
kmeans(Eigen::Matrix<double, 3, Eigen::Dynamic> &points, size_t nclusters) {
  double cumdist_threshold = 1e-10;
  size_t maxIter = 100;
  Eigen::Matrix<double, 3, Eigen::Dynamic> clusters =
      Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, nclusters);

  std::vector<int> numbers;
  for (int i = 0; i < points.cols(); i++)
    numbers.push_back(i);

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::shuffle(numbers.begin(), numbers.end(),
               std::default_random_engine(seed));

  for (size_t i = 0; i < nclusters; ++i)
    clusters.col(i) = points.col(numbers[i]);

  Eigen::Matrix<double, 3, Eigen::Dynamic> loca_points = points;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> distances =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(
          nclusters, points.cols());

  double previous_cost = 1e20;

  for (size_t iterator = 0; iterator < maxIter; ++iterator) {
    for (size_t i = 0; i < nclusters; ++i)
      distances.row(i) = (points.colwise() - clusters.col(i))
                             .array()
                             .square()
                             .colwise()
                             .sum()
                             .sqrt()
                             .matrix();
    auto [cost, correspondance, bins] = selmin(distances);
    clusters = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, nclusters);
    for (size_t j = 0; j < points.cols(); ++j)
      clusters.col(correspondance[j]) +=
          (1.0 / bins[correspondance[j]]) * points.col(j);
    if (std::abs(cost - previous_cost) < cumdist_threshold)
      break;
    previous_cost = cost;
  }
  return clusters;
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr,
           Eigen::Matrix<double, 3, Eigen::Dynamic>>
convertEigenToPCLPointCloudWithSpacingVoxelGrid(
    const Eigen::Matrix<double, 3, Eigen::Dynamic> &eigenPoints,
    double spacing) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_intermediate(
      new pcl::PointCloud<pcl::PointXYZ>());

  cloud_intermediate->width = eigenPoints.cols();
  cloud_intermediate->height = 1;
  cloud_intermediate->points.resize(eigenPoints.cols());
  for (size_t i = 0; i < eigenPoints.cols(); ++i) {
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
  std::printf("size of final point cloud: (%d %d)\n", (int)outEigenPoints.rows(),
              (int)outEigenPoints.cols());
  return {cloudout, outEigenPoints};
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
permutations(size_t num_clusters) {
  std::vector<size_t> random_cluster_order(num_clusters);
  std::iota(std::begin(random_cluster_order), std::end(random_cluster_order),
            0);

  std::vector<std::vector<size_t>> index_permutations;
  do {
    index_permutations.push_back(random_cluster_order);
  } while (std::next_permutation(random_cluster_order.begin(),
                                 random_cluster_order.end()));

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
      index_permutations_eigen =
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(
              index_permutations.size(), num_clusters);

  for (size_t row = 0; row < index_permutations.size(); ++row)
    for (size_t col = 0; col < num_clusters; ++col)
      index_permutations_eigen(row, col) = index_permutations[row][col];

  return index_permutations_eigen;
}

std::tuple<Eigen::Matrix<double, 4, 4>, double> arun_estimate(const Eigen::Matrix<double, 3, Eigen::Dynamic> &P, const Eigen::Matrix<double, 3, Eigen::Dynamic> &Q) {
  if (P.cols() != Q.cols())
    throw std::runtime_error("the algorithm requires the same number of cols");
  Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

  Eigen::Matrix<double, 3, 1> centroid_p = P.rowwise().mean();
  Eigen::Matrix<double, 3, 1> centroid_q = Q.rowwise().mean();

  Eigen::Matrix<double, 3, Eigen::Dynamic> P_centered =
      P.colwise() - centroid_p;
  Eigen::Matrix<double, 3, Eigen::Dynamic> Q_centered =
      Q.colwise() - centroid_q;

  Eigen::Matrix<double, 3, 3> H = Eigen::Matrix<double, 3, 3>::Zero();
  for (size_t i = 0; i < Q.cols(); ++i)
    H += P_centered.col(i) * Q_centered.col(i).transpose();

  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd(H, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 1> t = Eigen::Matrix<double, 3, 1>::Zero();

  Eigen::Matrix<double, 3, 3> U = svd.matrixU();
  Eigen::Matrix<double, 3, 3> V = svd.matrixV();

  Eigen::Matrix<double, 3, 3> X = V * (U.transpose());

  double error = -1;
  
  if (X.determinant() > 0) {
    R = X;
    t = centroid_q - R * centroid_p;
    Eigen::Matrix<double, 3, Eigen::Dynamic> Q_rotated = R.transpose()*Q;
    Q_rotated.colwise() -= R.transpose() * t; 
    Eigen::Matrix<double, 1, Eigen::Dynamic> error_loc = (P - Q_rotated).array().square().matrix().colwise().sum().array().sqrt().matrix();
    error = error_loc.sum();
  } else {
    Eigen::FullPivLU<Eigen::Matrix<double, 3, 3>> lu_decomp(H);
    auto rank = lu_decomp.rank();
    if (rank < 3) {
      V.col(2) *= -1.0;
      R = V * (U.transpose());
      t = centroid_q - R * centroid_p;
      Eigen::Matrix<double, 3, Eigen::Dynamic> Q_rotated = R.transpose()*Q;
      Q_rotated.colwise() -= R.transpose() * t; 
      Eigen::Matrix<double, 1, Eigen::Dynamic> error_loc = (P - Q_rotated).array().square().matrix().colwise().sum().array().sqrt().matrix();
      error = error_loc.sum();
    }
  }
  T.block<3, 3>(0, 0) = R.transpose();
  T.block<3, 1>(0, 3) = -R.transpose() * t;
  return {T, error};
}

std::array<std::tuple<Eigen::Matrix<double, 4, 4>, double>, 6> extract_potential_solutions(
    Eigen::Matrix<double, 3, Eigen::Dynamic> &fixed_cloud,
    Eigen::Matrix<double, 3, Eigen::Dynamic> &moving_cloud,
    size_t number_of_clusters) {
  std::vector<std::tuple<Eigen::Matrix<double, 4, 4>, double>>
      potential_solutions;
  auto perms = permutations(number_of_clusters);
  Eigen::Matrix<double, 3, Eigen::Dynamic> fixed_clusters =
      kmeans(fixed_cloud, number_of_clusters);
  Eigen::Matrix<double, 3, Eigen::Dynamic> moving_clusters =
      kmeans(moving_cloud, number_of_clusters);
  Eigen::Matrix<double, 3, Eigen::Dynamic> reordered_moving_clusters =
      moving_clusters;

  for (size_t perm = 0; perm < perms.rows(); ++perm) {
    for (size_t p = 0; p < perms.cols(); ++p)
      reordered_moving_clusters.col(p) = moving_clusters.col(perms(perm, p));
    //std::cout << "reordered_moving_clusters:\n" << reordered_moving_clusters << std::endl;
    const auto & [est_sol,cos] = arun_estimate(fixed_clusters, reordered_moving_clusters);
    potential_solutions.push_back(std::make_tuple(est_sol,cos));
  }
  std::sort(potential_solutions.begin(), potential_solutions.end(),
            [](std::tuple<Eigen::Matrix<double, 4, 4>, double> &a,
               std::tuple<Eigen::Matrix<double, 4, 4>, double> &b) {
              return std::get<1>(a) < std::get<1>(b);
            });
  std::array<std::tuple<Eigen::Matrix<double, 4, 4>, double>, 6>
      ordered_potential_solutions;
  for (size_t perm = 0; perm < 6; ++perm) {
    ordered_potential_solutions.at(perm) = potential_solutions.at(perm);
  }
  return ordered_potential_solutions;
};

Eigen::Matrix<double,4,4> homogenenous_from_vec(Eigen::Matrix<double,3,1> euler_angles){
    Eigen::Matrix<double,4,4> hom = Eigen::Matrix<double,4,4>::Identity();
    double t1 = cos(0.01745329252*euler_angles[2]);
    double t2 = sin(0.01745329252*euler_angles[2]);
    double t3 = cos(0.01745329252*euler_angles[1]);
    double t4 = sin(0.01745329252*euler_angles[1]);
    double t5 = cos(0.01745329252*euler_angles[0]);
    double t6 = sin(0.01745329252*euler_angles[0]);

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
}

enum Units{
    METERS,
    MILLIMETERS,
    CENTIMETERS
};

void solve_registration_problem(Units point_cloud_units,Eigen::Matrix<double,3,Eigen::Dynamic>& fixed_point_cloud,Eigen::Matrix<double,3,Eigen::Dynamic>& moving_point_cloud, double grid_spacing){

}

int main() {
 try{
    std::cout << "reading point cloud 1..." << std::endl;
  auto [cloud_fixed, eige_cloud_fixed] = convertEigenToPCLPointCloudWithSpacingVoxelGrid(read_point_cloud(CURAN_COPIED_RESOURCE_PATH"/temporary/file_fixed.txt",0.001),0.004);
  std::cout << "reading point cloud 1!" << std::endl;
  std::cout << "reading point cloud 2..." << std::endl;
  auto [cloud_moving, eige_cloud_moving] = convertEigenToPCLPointCloudWithSpacingVoxelGrid(read_point_cloud(CURAN_COPIED_RESOURCE_PATH"/temporary/file_moving.txt",0.001),0.004);
  std::cout << "reading point cloud 2!" << std::endl;
  int num_clusters = 3;

  //std::cout << "eige_cloud_fixed:\n" << eige_cloud_fixed << std::endl;
  //std::cout << "eige_cloud_moving:\n" << eige_cloud_moving << std::endl;

  auto ordered_solutions = extract_potential_solutions(eige_cloud_fixed, eige_cloud_moving, num_clusters);

  for(const auto& [transf,cost] : ordered_solutions)
    std::cout << "transform:\n" << transf << std::endl; 

  Eigen::Matrix<double,4,4> T_to_centroid_moving = Eigen::Matrix<double,4,4>::Identity();


    Eigen::Matrix<double, 3, 27> angle_array_around_estimated_solution;
    angle_array_around_estimated_solution << -10, -10, -10, 0, 0, 0, 10, 10, 10,
      -10, -10, -10, 0, 0, 0, 10, 10, 10, -10, -10, -10, 0, 0, 0, 10, 10, 10,
      -10, 0, 10, -10, 0, 10, -10, 0, 10, -10, 0, 10, -10, 0, 10, -10, 0, 10,
      -10, 0, 10, -10, 0, 10, -10, 0, 10, -10, -10, -10, -10, -10, -10, -10,
      -10, -10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10, 10, 10, 10, 10, 10;

    int iterations = 100; // Default number of ICP iterations

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;

    double lowest_cost = 1e100;

    std::vector<std::tuple<Eigen::Matrix<double,4,4>,double>> icp_solutions;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for(const auto& [transform,cost] : ordered_solutions){
        for(Eigen::Matrix<double,3,1> euler_angles : angle_array_around_estimated_solution.colwise()){
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setRANSACOutlierRejectionThreshold(0.03);
            icp.setMaximumIterations(iterations);
            icp.setInputSource(cloud_moving);
            icp.setInputTarget(cloud_fixed);
            icp.setMaxCorrespondenceDistance(0.01);
            icp.setTransformationEpsilon(1e-8);
            T_to_centroid_moving.block<3,1>(0,3) = transform.block<3,3>(0,0)*eige_cloud_moving.rowwise().mean()+transform.block<3,1>(0,3);
            icp.align(aligned_cloud,(T_to_centroid_moving*homogenenous_from_vec(euler_angles)*T_to_centroid_moving.inverse()*transform).cast<float>());
            
            if (icp.hasConverged())
                icp_solutions.emplace_back(icp.getFinalTransformation().cast<double>(),icp.getFitnessScore(100));
            else 
                icp_solutions.emplace_back(icp.getFinalTransformation().cast<double>(),1.0e10);
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    std::sort(icp_solutions.begin(), icp_solutions.end(),
            [](std::tuple<Eigen::Matrix<double, 4, 4>, double> &a,
               std::tuple<Eigen::Matrix<double, 4, 4>, double> &b) {
              return std::get<1>(a) < std::get<1>(b);
            });
    const auto&[transf,cost] = icp_solutions[0];

    for(size_t best_index = 0; best_index < 6 && best_index < icp_solutions.size(); ++best_index ){
        const auto&[l_transf,l_cost] = icp_solutions[best_index];
        std::cout << "transform:\n" << l_transf << "\ncost:" << l_cost << std::endl;
    }

    Eigen::Matrix<double,3,Eigen::Dynamic> transformed_eige_cloud_moving = transf.block<3,3>(0,0)*eige_cloud_moving;
    transformed_eige_cloud_moving.colwise() += transf.block<3,1>(0,3);
    write_point_cloud(transformed_eige_cloud_moving,"rotated_moving.txt");
 } catch(std::runtime_error& e){
    std::cout << "exception thrown: " << e.what() <<"\n";
    return 1;
 } catch(...){
    std::cout << "exception thrown\n";
    return 1;
 }

  return 0;
}

/*
  std::vector<size_t> random_cluster_order(num_clusters) ; // vector with 100
ints. std::iota (std::begin(random_cluster_order),
std::end(random_cluster_order), 0); // Fill with 0, 1, ..., 99.

std::vector<std::vector<size_t>> index_permutations;
do{
    for(auto v : random_cluster_order)
        std::cout << v << " , ";
    index_permutations.push_back(random_cluster_order);
} while (std::next_permutation(random_cluster_order.begin(),
random_cluster_order.end()));

Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> index_permutations_eigen =
Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>::Zero(index_permutations.size(),num_clusters);

for(size_t row = 0; row < index_permutations.size(); ++row)
    for(size_t col = 0; col < num_clusters; ++col)
        index_permutations_eigen(row,col) = index_permutations[row][col];
Eigen::Matrix<double,3,Eigen::Dynamic> clusters = kmeans(eige_cloud,
num_clusters); std::cout << "clusters:\n" << clusters << std::endl;

std::cout << "permutations:\n" << permutations(num_clusters) << std::endl;
*/
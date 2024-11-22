#include "imageprocessing/ArunAlgorithm.h"
#include <numeric>
#include <random>
#include <chrono>

#include <iostream>

namespace curan {
namespace image {

std::tuple<double,Eigen::Matrix<size_t,Eigen::Dynamic,1>,Eigen::Matrix<size_t,Eigen::Dynamic,1>> selmin(const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& distances){
    Eigen::Matrix<size_t,Eigen::Dynamic,1> correspondance = Eigen::Matrix<size_t,Eigen::Dynamic,1>::Zero(distances.cols(),1);
    Eigen::Matrix<size_t,Eigen::Dynamic,1> bins = Eigen::Matrix<size_t,Eigen::Dynamic,1>::Zero(distances.rows(),1);
    double cost = 0;
    for(size_t i = 0; i< distances.cols(); ++i){
        Eigen::Index minRow, minCol;
        double minimum = distances.col(i).minCoeff(&minRow, &minCol);
        correspondance[i] = minRow;
        cost += minimum;
        bins[minRow] = bins[minRow]+1; 
    }
    return {cost,correspondance,bins};
}

Eigen::Matrix<double,3,Eigen::Dynamic> kmeans(const Eigen::Matrix<double,3,Eigen::Dynamic>& points , size_t nclusters){
    double cumdist_threshold = 1e-10;
    size_t maxIter = 100;
    Eigen::Matrix<double,3,Eigen::Dynamic> clusters = Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3,nclusters);

    std::vector<int> numbers;
    for(int i=0; i<points.cols(); i++)
        numbers.push_back(i);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::shuffle(numbers.begin(), numbers.end(), std::default_random_engine(seed));

    for(size_t i = 0 ; i < nclusters; ++i)
        clusters.col(i) = points.col(numbers[i]);

    Eigen::Matrix<double,3,Eigen::Dynamic> loca_points = points;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> distances = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>::Zero(nclusters,points.cols());

    double previous_cost = 1e30;

    for(size_t iterator = 0; iterator< maxIter ; ++iterator){
        for(size_t i = 0; i<nclusters; ++i)
            distances.row(i) = (points.colwise()-clusters.col(i)).array().square().colwise().sum().sqrt().matrix();
        auto [cost,correspondance,bins] = selmin(distances);  
        clusters = Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3,nclusters);      
        for(size_t j = 0; j <points.cols() ; ++j)
            clusters.col(correspondance[j]) += (1.0/bins[correspondance[j]])*points.col(j);
        if(std::abs(cost-previous_cost)<cumdist_threshold)
            break;
        previous_cost = cost;
    }
    return clusters;
}

std::tuple<Eigen::Matrix<double,4,4>,double> arun_estimate(const Eigen::Matrix<double,3,Eigen::Dynamic>& P, const Eigen::Matrix<double,3,Eigen::Dynamic>& Q){
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

std::array<std::tuple<Eigen::Matrix<double, 4, 4>, double>, 6> extract_potential_solutions(const Eigen::Matrix<double, 3, Eigen::Dynamic> &fixed_cloud,const  Eigen::Matrix<double, 3, Eigen::Dynamic> &moving_cloud,size_t number_of_clusters) {
  std::vector<std::tuple<Eigen::Matrix<double, 4, 4>, double>>potential_solutions;

 auto permutations = [](size_t num_clusters) {
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
};

  auto perms = permutations(number_of_clusters);
  Eigen::Matrix<double, 3, Eigen::Dynamic> fixed_clusters = curan::image::kmeans(fixed_cloud, number_of_clusters);
  Eigen::Matrix<double, 3, Eigen::Dynamic> moving_clusters = curan::image::kmeans(moving_cloud, number_of_clusters);

  std::cout << "fixed_clusters:\n" << fixed_clusters << std::endl;
  std::cout << "moving_clusters:\n" << moving_clusters << std::endl;

  Eigen::Matrix<double, 3, Eigen::Dynamic> reordered_moving_clusters = moving_clusters;

  for (size_t perm = 0; perm < perms.rows(); ++perm) {
    for (size_t p = 0; p < perms.cols(); ++p)
      reordered_moving_clusters.col(p) = moving_clusters.col(perms(perm, p));
    const auto & [est_sol,cos] = curan::image::arun_estimate(fixed_clusters, reordered_moving_clusters);
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

}
}
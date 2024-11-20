#include "imageprocessing/ArunAlgorithm.h"
#include <random>
#include <chrono>

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

Eigen::Matrix<double,3,Eigen::Dynamic> kmeans(Eigen::Matrix<double,3,Eigen::Dynamic>& points , size_t nclusters){
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
    if(P.cols() != Q.cols())
        throw std::runtime_error("the algorithm requires the same number of cols");
    Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();

    Eigen::Matrix<double,3,1> centroid_p = P.rowwise().mean();
    Eigen::Matrix<double,3,1> centroid_q = Q.rowwise().mean();

    Eigen::Matrix<double,3,Eigen::Dynamic> P_centered = P.colwise()-centroid_p;
    Eigen::Matrix<double,3,Eigen::Dynamic> Q_centered = Q.colwise()-centroid_q;

    Eigen::Matrix<double,3,3> H = Eigen::Matrix<double,3,3>::Zero();
    for(size_t i= 0; i<Q.cols() ; ++i )
        H += P_centered.col(i) * Q_centered.col(i).transpose(); 
    
    Eigen::JacobiSVD<Eigen::Matrix<double,3,3>> svd(H, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix<double,3,3> R = Eigen::Matrix<double,3,3>::Identity();
    Eigen::Matrix<double,3,1> t = Eigen::Matrix<double,3,1>::Zero();

    Eigen::Matrix<double,3,3> U = svd.matrixU();
    Eigen::Matrix<double,3,3> V = svd.matrixV();

    Eigen::Matrix<double,3,3> X = V*U.transpose();

    double error = -1;
    if(X.determinant()>0){
        R = X;
        t = centroid_q-R*centroid_p;
        error = ((P-((R.transpose()*Q).colwise()-R.transpose()*t)).array().square().rowwise().sum().sqrt().colwise().sum())(0,0);
    } else {
        Eigen::FullPivLU<Eigen::Matrix<double,3,3>> lu_decomp(H);
        auto rank = lu_decomp.rank();
        if(rank<3){
            V.col(2) *= -1.0;
            R = V*U.transpose();
            t = centroid_q-R*centroid_p;
            error = ((P-((R.transpose()*Q).colwise()-R.transpose()*t)).array().square().rowwise().sum().sqrt().colwise().sum())(0,0);
        }
    }
    T.block<3,3>(0,0) = R.transpose();
    T.block<3,1>(0,3) = -R.transpose()*t;
    return {T,error};
}

}
}
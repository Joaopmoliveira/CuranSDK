#include <iostream>
#include <Eigen/Dense>

int main(){
    Eigen::Matrix<double,6,6> random_mat = Eigen::Matrix<double,6,6>::Random();
    auto Lambda = random_mat*random_mat.transpose();

    Eigen::Matrix<double,6,6> diagonal_damping = Eigen::Matrix<double,6,6>::Identity();

    Eigen::Matrix<double,6,6> random_mat_2 = Eigen::Matrix<double,6,6>::Random();
    auto stiffness = random_mat_2*random_mat_2.transpose();

    Eigen::LLT<Eigen::Matrix<double,6,6>> lltOfLambda(Lambda);
    Eigen::Matrix<double,6,6> L = lltOfLambda.matrixL();
    Eigen::Matrix<double,6,6> R = L.inverse();
    Eigen::Matrix<double,6,6> C = R*stiffness*R.transpose();
    Eigen::JacobiSVD<Eigen::Matrix<double,6,6>> svdofsitff{C,Eigen::ComputeFullU};
    Eigen::Matrix<double,6,6> P = svdofsitff.matrixU();
    Eigen::Matrix<double,6,6> Q = R.inverse()*P;
    Eigen::Matrix<double,6,6> Qinv = Q.inverse();
    Eigen::Matrix<double,6,6> B0 = Qinv*stiffness*Qinv.transpose();
    Eigen::Matrix<double,6,6> damping = 2*Q.transpose()*diagonal_damping*B0.diagonal().array().sqrt().matrix().asDiagonal()*Q;

    std::cout << damping << std::endl;

    return 0;
}
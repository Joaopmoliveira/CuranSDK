#include <iostream>
#include <Eigen/Dense>
#include "robotutils/LBRController.h"

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

int main1()
{
    curan::robotic::State state;
    state.q = {1.4, 1.4, 1.4, 1.4, .1, 1., -1.0};
    state.sampleTime = 0.001;
    curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    robot_model.update(state);

    static Eigen::Matrix<double, 6, 6> Iden = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> Lambda = (robot_model.jacobian() * robot_model.invmass() * robot_model.jacobian().transpose() + (0.071 * 0.071) * Iden).inverse();

    Eigen::Matrix<double, 6, 6> diagonal_damping = Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 6, 6> random_mat_2 = Eigen::Matrix<double, 6, 6>::Random();
    auto stiffness = 100 * random_mat_2 * random_mat_2.transpose();

    Eigen::LLT<Eigen::Matrix<double, 6, 6>> lltOfLambda(Lambda);
    Eigen::Matrix<double, 6, 6> L = lltOfLambda.matrixL();
    Eigen::Matrix<double, 6, 6> R = L.inverse();
    Eigen::Matrix<double, 6, 6> C = R * stiffness * R.transpose();
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svdofsitff{C, Eigen::ComputeFullU};
    Eigen::Matrix<double, 6, 6> P = svdofsitff.matrixU();
    Eigen::Matrix<double, 6, 6> Q = R.inverse() * P;
    Eigen::Matrix<double, 6, 6> Qinv = Q.inverse();
    Eigen::Matrix<double, 6, 6> B0 = Qinv * stiffness * Qinv.transpose();
    Eigen::Matrix<double, 6, 6> damping = 2 * Q * diagonal_damping * B0.diagonal().array().sqrt().matrix().asDiagonal() * Q.transpose();

    // std::cout << robot_model.jacobian() << std::endl << std::endl;
    std::cout << Lambda << std::endl
              << std::endl;
    std::cout << Lambda - L * L.transpose() << std::endl
              << std::endl;
    std::cout << Lambda - Q * Q.transpose() << std::endl;
    std::cout << stiffness << std::endl
              << std::endl;
    std::cout << stiffness - Q * B0 * Q.transpose() << std::endl
              << std::endl;
    return 0;
}

int main(){
    Eigen::Matrix<double, 2, 2> L = Eigen::Matrix<double, 2, 2>::Zero();
    L << 1.111111111111111111111 , 2.111111111111111111111, 3.111111111111111111111 , 4.111111111111111111111;

    Eigen::Matrix<double, 2, 2> P = Eigen::Matrix<double, 2, 2>::Zero();
    P << 1.111111111111111111111 , 2.111111111111111111111, 3.111111111111111111111 , 4.111111111111111111111;
    
    std::cout << "is L similar to P? :" <<  L.isApprox(P) << std::endl;
    return 0;
}
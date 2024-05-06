#include <memory>
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"

constexpr size_t number_of_joints = 7;

void do_mass_with_multiplication_factor(){
    Vector3d pointPosition = Vector3d(0, 0, 0.045); 
    kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);

    std::unique_ptr<kuka::Robot> robot = std::make_unique<kuka::Robot>(myName); 
    std::unique_ptr<RobotParameters> iiwa = std::make_unique<RobotParameters>(); 

    double q[number_of_joints] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0};
    double dq[number_of_joints] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0};

    for (int index = 0; index < number_of_joints; ++index) {
        iiwa->q[index] = q[index];
        iiwa->qDot[index] = dq[index];
    }
    Vector3d tmp_p_0_7 = Vector3d::Zero();
    Matrix3d  tmp_R_0_7 = Matrix3d::Identity(); 
    MatrixNd tmp_jacobian = MatrixNd::Zero(number_of_joints,number_of_joints);
    robot->getMassMatrix(iiwa->M, iiwa->q);
    iiwa->M(6, 6) = 45 * iiwa->M(6, 6);   
    iiwa->Minv = iiwa->M.inverse();
    robot->getCoriolisAndGravityVector(iiwa->c, iiwa->g, iiwa->q, iiwa->qDot);
    robot->getWorldCoordinates(tmp_p_0_7, iiwa->q, pointPosition, 7);  
    robot->getRotationMatrix(tmp_R_0_7, iiwa->q, number_of_joints); 
    robot->getJacobian(tmp_jacobian, iiwa->q, pointPosition, 7);    
    std::cout << "mass : \n " << iiwa->M << "\n";
    std::cout << "inverse mass : \n " << iiwa->Minv << "\n";
}

void do_mass_with_diagonal_term(){
    Vector3d pointPosition = Vector3d(0, 0, 0.045); 
    kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);

    std::unique_ptr<kuka::Robot> robot = std::make_unique<kuka::Robot>(myName); 
    std::unique_ptr<RobotParameters> iiwa = std::make_unique<RobotParameters>(); 

    double q[number_of_joints] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0};
    double dq[number_of_joints] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0};

    for (int index = 0; index < number_of_joints; ++index) {
        iiwa->q[index] = q[index];
        iiwa->qDot[index] = dq[index];
    }
    Vector3d tmp_p_0_7 = Vector3d::Zero();
    Matrix3d  tmp_R_0_7 = Matrix3d::Identity(); 
    MatrixNd tmp_jacobian = MatrixNd::Zero(number_of_joints,number_of_joints);
    robot->getMassMatrix(iiwa->M, iiwa->q);
    iiwa->M += Eigen::Matrix<double,number_of_joints,number_of_joints>::Identity()*0.1;
    iiwa->Minv = iiwa->M.inverse();
    robot->getCoriolisAndGravityVector(iiwa->c, iiwa->g, iiwa->q, iiwa->qDot);
    robot->getWorldCoordinates(tmp_p_0_7, iiwa->q, pointPosition, 7);  
    robot->getRotationMatrix(tmp_R_0_7, iiwa->q, number_of_joints); 
    robot->getJacobian(tmp_jacobian, iiwa->q, pointPosition, 7);    
    std::cout << "mass : \n " << iiwa->M << "\n";
    std::cout << "inverse mass : \n " << iiwa->Minv << "\n";
}

int main(){
    std::cout << "mass with multiplication \n";
    do_mass_with_multiplication_factor();
    do_mass_with_multiplication_factor();
    std::cout << "==========================\nmass with diagonal term \n";
    do_mass_with_diagonal_term();
    do_mass_with_diagonal_term();
    return 0;
}
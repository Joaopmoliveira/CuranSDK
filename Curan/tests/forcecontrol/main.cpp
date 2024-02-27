#include <vsgParticleSystem/all.h>

class TestRobot {
public:
    TestRobot() : gravity{3, 0.05}, softplus{3} {
        softplus.setK(25);
        softplus.setUpperXi(1.6);
        softplus.setUpperXitilde(1.6);

        gravity.setSharpness(75);
        gravity.setUpperBound(.06);
        gravity.setLowerBound(-.5);

    }

    void applyDynamics(Eigen::Vector3d const &current_position, Eigen::Vector3d &desired_velocity) {
        vsgps::StateMatrix A = Eigen::MatrixXd::Zero(3, 3);
        A << -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0;
        vsgps::State equilibrium(3);
        equilibrium << -0.568243, 0.0131714, 0.428667;
        equilibrium(2) -= 0.257361;
        std::cout << "eqPoint before softplus:\n"
                  << equilibrium << "\n";
        softplus(equilibrium);
        std::cout << "eqPoint after softplus:\n"
                  << equilibrium << "\n";
        vsgps::State b = -A * equilibrium;
        std::cout << "b:\n"
                  << b << "\n";
        vsgps::State disp = Eigen::Vector3d::Zero();
        std::cout << "current_position:\n"
                  << current_position << "\n";
        // vectorI2O(current_position, disp);
        disp = current_position;
        std::cout << "disp:\n"
                  << disp << "\n";
        disp(2) -= 0.257361;
        vsgps::State gravPos = disp;
        softplus(disp);
        // vsgps::State desired_velocity = A * disp + b;
        vsgps::State dxi = A * disp + b;
        softplus.invJ(disp, dxi);
        gravity(gravPos, dxi);
        std::cout << "dxi with gravity:\n"
                  << dxi << "\n";
        // vectorI2O(dxi, desired_velocity);
        desired_velocity = dxi;
    }

protected:
    vsgps::SoftPlus softplus;
    vsgps::GravityField gravity;

};

int main(int argc, char const *argv[]) {
    Eigen::Vector3d pos = Eigen::Vector3d::Random(3);
    Eigen::Vector3d des_vel = Eigen::Vector3d::Zero();
    TestRobot r;
    r.applyDynamics(pos, des_vel);

    return 0;
}

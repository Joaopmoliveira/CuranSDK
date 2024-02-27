#include "DataTest.h"

int main(int argc, char **argv) {
    std::string filename6D =
        "../../../data/dynamics/pos-force-demo-gmm8-coupled-decoupledA-6D.json";
    vsgps::Dynamics dynamics = vsgps::Dynamics::fromJson(filename6D);
    Eigen::VectorXd xi(6), dxi_m(6), dxi_cpp(6);
    double e;
    int idx;
    bool thereWasError = false;
    std::ofstream logFile("log6D.log");
    logFile << dynamics.logAb() << "\n";
    logFile << dynamics.logProps() << "\n";
    logFile << "Starting test:\n";
    std::vector<Line> data = getData();
    std::chrono::steady_clock::time_point tic, toc;
    double dt;
    for (auto const &l : data) {
        idx = &l - &data[0];
        xi = Eigen::VectorXd{
            {l.xi[0], l.xi[1], l.xi[2], l.xi[3], l.xi[4], l.xi[5]}};
        dxi_m = Eigen::VectorXd{{l.dxi_m[0], l.dxi_m[1], l.dxi_m[2], l.dxi_m[3],
                                 l.dxi_m[4], l.dxi_m[5]}};
        tic = std::chrono::steady_clock::now();
        dxi_cpp = dynamics(xi);
        toc = std::chrono::steady_clock::now();
        dt = std::chrono::duration<double>(toc - tic).count();
        e = (dxi_m - dxi_cpp).norm();
        if (e / dxi_m.norm() > .1) {
            logFile << "###################### ERROR > .1 "
                       "######################\n";
            thereWasError = true;
        }
        logFile << "Took " << dt << " seconds to run\n";
        logFile << "xi[" << idx << "] =\n" << xi << "\n";
        logFile << "dxi_m[" << idx << "] =\n" << dxi_m << "\n";
        logFile << "dxi_cpp[" << idx << "] =\n" << dxi_cpp << "\n";
        logFile << "Error[" << idx << "] = " << e << "\n";
        logFile << dynamics.log(xi);
        if (e / dxi_m.norm() > .1) {
            logFile << "###################### END  ERROR "
                       "######################\n";
        }
        logFile << "\n";
        progressBar(idx / (float)data.size());
    }
    logFile.close();
    return thereWasError;
}

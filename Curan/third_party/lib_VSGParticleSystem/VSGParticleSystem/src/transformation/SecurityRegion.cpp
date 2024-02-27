#include <fstream>
#include <nlohmann/json.hpp>
#include <vsgParticleSystem/transformation/SecurityRegion.h>
#include <vsgParticleSystem/utils/stringUtils.h>
#include <vsgParticleSystem/utils/vectorUtils.h>

namespace vsgps {

SecurityRegion::SecurityRegion(Eigen::Matrix<double, 2, 4> const &corners) {
    // 1. Get the nearest point to the origin (defaults to the first point)
    origin << corners(Eigen::all, 0);
    for (auto const &c : corners.colwise()) {
        if (c.norm() < origin.norm() || c == Eigen::Vector2d::Zero()) {
            origin = c;
        }
    }

    // 2. Superimpose origin with the new origin and
    // 3. Get the two nearest points from the new origin (defaults to second and
    // third point)
    //
    // For that, we order the transformedCorners in terms of norm while being
    // translated to the new origin, which in return gives the new axis on the
    // second and third points
    std::vector<Eigen::Vector2d> orderingCorners;
    for (auto c : corners.colwise())
        orderingCorners.push_back(c - origin);

    sort<Eigen::Vector2d>(orderingCorners, [](Eigen::Vector2d const &i,
                                              Eigen::Vector2d const &j) {
        return i.norm() < j.norm();
    });

    // Removes the origin (first element) and farest point (last element)
    orderingCorners.erase(orderingCorners.begin());
    orderingCorners.pop_back();

    // https://math.stackexchange.com/a/1685315
    Eigen::Vector3d u, v;
    Eigen::Vector3d temp = Eigen::Vector3d::Zero();
    u << orderingCorners[0], 0;
    v << orderingCorners[1], 0;
    Eigen::Vector3d k = u.cross(v);
    Eigen::Vector3d z;
    z << 0, 0, 1;
    if (k.dot(z) < 0) {
        temp = Eigen::Vector3d(u);
        u = Eigen::Vector3d(v);
        v = Eigen::Vector3d(temp);
    }
    R.setZero();
    R(0, Eigen::all) << u(Eigen::seq(0, 1)).transpose() /
                            (u(Eigen::seq(0, 1)).norm() *
                             u(Eigen::seq(0, 1)).norm());
    R(1, Eigen::all) << v(Eigen::seq(0, 1)).transpose() /
                            (v(Eigen::seq(0, 1)).norm() *
                             v(Eigen::seq(0, 1)).norm());
}

void SecurityRegion::validate(State const &xiDisp) {
    Eigen::Vector2d p;
    p << xiDisp(Eigen::seq(0, 1));
    p = R * (p - origin);
    bool validPoint = 0 < p[0] && p[0] < 1 && 0 < p[1] && p[1] < 1;
    if (!validPoint) {
        std::cerr << "It's offlimits!\np = " << p << "\nxiDisp = " << xiDisp
                  << "\n";
        std::exit(0);
    }
}

SecurityRegion SecurityRegion::fromJson(std::string path) {
    replaceAll(path, "\\", "/");
    auto data = nlohmann::json::parse(std::ifstream(path))["config"]["limits"];
    Eigen::Matrix<double, 2, 4> limits;
    for (size_t i = 0; i < 2; i++) {
        for (size_t j = 0; j < 4; j++) {
            limits(i, j) = data[i][j];
        }
    }
    return {limits};
}

std::ostream &operator<<(std::ostream &os, SecurityRegion const &sr) {
    os << "SecurityRegion {\n\tRotation: [";
    for (int row = 0; row < sr.R.rows(); row++) {
        for (size_t column = 0; column < sr.R.cols(); column++) {
            os << sr.R(row, column);
            if (column != sr.R.cols() - 1)
                os << ", ";
        }
        if (row != sr.R.rows() - 1)
            os << "; ";
    }
    os << "]\n\tOrigin: [";
    for (auto const &o : sr.origin) {
        os << o;
        if (&o - &sr.origin[0] != sr.origin.size() - 1) {
            os << ", ";
        }
    }
    os << "]\n};\n";
    return os;
}

} // namespace vsgps

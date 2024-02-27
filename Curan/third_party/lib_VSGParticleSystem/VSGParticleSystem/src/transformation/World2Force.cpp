#include <fstream>
#include <nlohmann/json.hpp>
#include <vsgParticleSystem/transformation/World2Force.h>
#include <vsgParticleSystem/utils/stringUtils.h>

namespace vsgps {

World2Disp::World2Disp(size_t transformationIdx)
    : Transformation(transformationIdx), normal(0, 0, 1, 0), origin(0, 0, 0),
      velIdxs{-1, -1, -1}, indices{0, 1, 2} {}

// todo: adaptar para diferentes tipos de dimensões do vetor State
// todo: Implements conversion from delta to Force

void World2Disp::computeK(Eigen::Vector3d const &k) {
    K << 0, -k(2), k(1), k(2), 0, -k(0), -k(1), k(0), 0;
}

void World2Disp::computeR() {
    Eigen::Vector3d z(0, 0, 1);
    Eigen::Vector3d n(normal(Eigen::seq(0, 2)));
    Eigen::Vector3d k = z.cross(n);
    if (k == Eigen::Vector3d::Zero()) {
        k = {0, 1, 0};
    } else {
        k = k.normalized();
    }
    double theta = std::acos(z.dot(n) / n.norm());
    double s = std::sin(theta);
    double c = std::cos(theta);
    this->computeK(k);
    R = Eigen::Matrix3d::Identity() + s * K + (1 - c) * (K * K);
}

void World2Disp::setVelIdxs(std::array<int, 3> const &v) {
    velIdxs = std::array<int, 3>(v);
    indices = std::vector<int>{0, 1, 2};
    for (int i = 3; i < 7; i++) {
        if (velIdxs[i - 3] != -1)
            indices.push_back(velIdxs[i - 3] + 4);
        else
            break;
    }
}

std::array<int, 3> const &World2Disp::getVelIdxs() const { return velIdxs; }

void World2Disp::computeT() {
    this->computeR();
    T.setZero();
    T(Eigen::seq(0, 3), Eigen::seq(0, 3)) << R, origin,
        Eigen::Matrix<double, 1, 3>::Zero(), 1;
    T(Eigen::seq(4, Eigen::last), Eigen::seq(4, Eigen::last)) << R;
    invT = T.inverse();
}

Eigen::Matrix<double, 7, 7> const &World2Disp::getT() const { return this->T; }

void World2Disp::setNormal(Normal const &n) { normal = n; }

Normal const &World2Disp::getNormal() const { return normal; }

void World2Disp::setOrigin(Origin const &o) { origin = o; }

Origin const &World2Disp::getOrigin() const { return origin; }

void World2Disp::setIsPosition(bool isPos) { isPosition = isPos; }

bool const &World2Disp::getIsPosition() const { return isPosition; }

void World2Disp::operator()(State &xi) {
    h.setZero();
    h(Eigen::seq(0, 3)) << xi(Eigen::seq(0, 2)), isPosition;
    for (int const &i : velIdxs) {
        if (i != -1)
            h(i + 4) = xi(i + 3);
        else
            break;
    }
    xi = (invT * h)(indices);
}

void World2Disp::operator()(State const &xi, State &xitilde) {
    xitilde = xi;
    this->operator()(xitilde);
}

void World2Disp::inv(State const &xitilde, State &xi) {
    xi = xitilde;
    this->inv(xi);
}

void World2Disp::inv(State &xitilde) {
    h.setZero();
    h(Eigen::seq(0, 3)) << xitilde(Eigen::seq(0, 2)), isPosition;
    for (int const &i : velIdxs) {
        if (i != -1)
            h(i + 4) = xitilde(i + 3);
        else
            break;
    }
    xitilde = (T * h)(indices);
}

World2Disp World2Disp::fromJson(std::string path) {
    replaceAll(path, "\\", "/");
    auto data = nlohmann::json::parse(std::ifstream(path))["config"];
    World2Disp w2d;
    Normal n;
    Origin o;
    size_t normalIdx = 0;
    for (auto &d : data["normal"])
        n(normalIdx++) = d;
    normalIdx = 0;
    for (auto &d : data["origin"])
        o(normalIdx++) = d;

    w2d.setNormal(n);
    w2d.setOrigin(o);
    w2d.computeT();

    return w2d;
}

std::ostream &operator<<(std::ostream &out, World2Disp const &w2d) {
    std::ostringstream os;
    os << "World2Disp {\n\tnormal: [";
    for (auto &n : w2d.normal) {
        os << n;
        if ((&n - &w2d.normal[0]) != w2d.normal.size() - 1) {
            os << ", ";
        }
    }
    os << "],\n\torigin: [";
    for (auto &o : w2d.origin) {
        os << o;
        if (&o - &w2d.origin[0] != w2d.origin.size() - 1) {
            os << ", ";
        }
    }
    os << "],\n\tT: [";
    for (size_t i = 0; i < w2d.T.rows(); i++) {
        for (size_t j = 0; j < w2d.T.cols(); j++) {
            os << w2d.T.operator()(i, j);
            if (j != w2d.T.cols() - 1)
                os << ", ";
        }
        if (i != w2d.T.rows() - 1) {
            os << "; ";
        }
    }
    os << "],\n\tinvT: [";
    for (size_t i = 0; i < w2d.T.rows(); i++) {
        for (size_t j = 0; j < w2d.T.cols(); j++) {
            os << w2d.invT.operator()(i, j);
            if (j != w2d.invT.cols() - 1)
                os << ", ";
        }
        if (i != w2d.invT.rows() - 1) {
            os << "; ";
        }
    }
    os << "],\n\tindices: [";
    for (auto const &i : w2d.indices) {
        os << i;
        if (&i - &w2d.indices[0] != w2d.indices.size() - 1) {
            os << ", ";
        }
    }
    os << "]\n};\n\n";
    return out << os.str();
}

void World2Disp::invVel(State &dxitilde) {
    this->setIsPosition(false);
    this->inv(dxitilde);
    this->setIsPosition(true);
}
void World2Disp::invVel(State const &dxitilde, State &dxi) {
    dxi = dxitilde;
    this->invVel(dxi);
}

// #############################################################################

Disp2Force::Disp2Force(size_t idxTransformation)
    : Transformation{idxTransformation}, A{0.01}, E{100}, L{.01} {}

void Disp2Force::computeK() { K = (A * E) / L; }

void Disp2Force::setLength(double newL) {
    assert(newL > 0);
    L = newL;
    computeK();
}
double const &Disp2Force::getLength() const { return L; }

void Disp2Force::setArea(double newA) {
    assert(newA > 0);
    A = newA;
    computeK();
}
double const &Disp2Force::getArea() const { return A; }

void Disp2Force::setYoungModulus(double newE) {
    assert(newE > 0);
    E = newE;
    computeK();
}
double const &Disp2Force::getYoungModulus() const { return E; }

void Disp2Force::setStiffness(double newK) {
    assert(newK > 0);
    K = newK;
}
double const &Disp2Force::getStiffness() const { return K; }

void Disp2Force::operator()(State &xi) {
    assert(xi.size() > idx);
    if (xi[idx] < limit)
        xi[idx] = limit;
    xi[idx] *= K;
}

void Disp2Force::operator()(State const &xi, State &xitilde) {
    xitilde = xi;
    this->operator()(xitilde);
}

void Disp2Force::inv(State &xitilde) { xitilde[idx] /= K; }

void Disp2Force::inv(State const &xitilde, State &xi) {
    xi = xitilde;
    this->inv(xi);
}

Disp2Force Disp2Force::fromJson(std::string path) {
    replaceAll(path, "\\", "/");
    auto data = nlohmann::json::parse(std::ifstream(path))["disp"];
    Disp2Force d2f(((int)data["idxF"]) - 1);
    if (data.contains("K")) {
        d2f.setStiffness(data["K"]);
    } else {
        d2f.setLength(data["L"]);
        d2f.setArea(data["A"]);
        d2f.setYoungModulus(data["E"]);
    }
    if (data.contains("limit"))
        d2f.setLimit(data["limit"]);

    return d2f;
}

std::ostream &operator<<(std::ostream &out, Disp2Force const &d2f) {
    std::ostringstream os;
    os << "Disp2Force {\n\tK: " << d2f.K << "\n\tidxF: " << d2f.idx
       << "\n\tlimit: " << d2f.limit << "\n};\n\n";
    return out << os.str();
}

void Disp2Force::setLimit(double newLimit) {
    assert(newLimit > 0);
    limit = newLimit;
}
double const &Disp2Force::getLimit() const { return limit; }

} // namespace vsgps

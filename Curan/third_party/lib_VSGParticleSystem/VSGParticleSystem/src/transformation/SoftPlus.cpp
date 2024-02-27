#include <fstream>
#include <nlohmann/json.hpp>
#include <vsgParticleSystem/transformation/SoftPlus.h>
#include <vsgParticleSystem/utils/stringUtils.h>

namespace vsgps {
SoftPlus::SoftPlus(size_t transformationIdx, double otherEPS)
    : Transformation(transformationIdx), k{1}, eps(otherEPS) {
    upperXi = 40 / k;
    upperXitilde = softplus(upperXi);
}

// todo: usar este site https://www.desmos.com/calculator/a5cgngpdry

void SoftPlus::operator()(State &xi) {
    assert(xi.size() > idx);
    xi[idx] = xi[idx] > upperXi ? xi[idx] : softplus(xi[idx]);
}

void SoftPlus::operator()(State const &xi, State &xitilde) {
    assert(xi.size() > idx);
    xitilde = xi;
    this->operator()(xitilde);
}

void SoftPlus::inv(State &xitilde) {
    assert(xitilde.size() > idx);
    xitilde[idx] =
        xitilde[idx] > upperXitilde ? xitilde[idx] : invsoftplus(xitilde[idx]);
}

void SoftPlus::inv(State const &xitilde, State &xi) {
    assert(xitilde.size() > idx);
    xi = xitilde;
    this->inv(xi);
}

void SoftPlus::invVel(State const &xiDeformed, State &dxi) {
    this->invJ(xiDeformed, dxi);
}

void SoftPlus::setK(double newK) {
    assert(k > 0);
    k = newK;
}

double const SoftPlus::getK() const { return k; }

SoftPlus SoftPlus::fromJson(std::string path) {
    replaceAll(path, "\\", "/");
    auto data = nlohmann::json::parse(std::ifstream(path))["softplus"];
    SoftPlus sp(((int)data["idxF"]) - 1);
    sp.setK(data["k"]);
    sp.setUpperXi(data["upperxi"]);
    sp.setUpperXitilde(data["upperxitilde"]);
    return sp;
}

void SoftPlus::setUpperXi(double newUpperXi) { upperXi = newUpperXi; }
double const SoftPlus::getUpperXi() const { return upperXi; }

void SoftPlus::setUpperXitilde(double newUpperXitilde) {
    upperXitilde = newUpperXitilde;
}
double const SoftPlus::getUpperXitilde() const { return upperXitilde; }

void SoftPlus::J(State const &xi, State &dxi) {
    StateMatrix Jac = Eigen::MatrixXd::Identity(dxi.rows(), xi.rows());
    Jac(idx, idx) = xi[idx] > upperXi ? 1 : dsoftplus(xi[idx]);
    dxi = Jac * dxi;
}

void SoftPlus::J(State const &xi, State const &dxi, State &dxitilde) {
    dxitilde = dxi;
    this->J(xi, dxitilde);
}

void SoftPlus::invJ(State const &xitilde, State &dxitilde) {
    StateMatrix invJac = StateMatrix::Identity(dxitilde.rows(), xitilde.rows());
    invJac(idx, idx) =
        xitilde[idx] > upperXitilde ? 1 : dinvsoftplus(xitilde[idx]);
    dxitilde = invJac * dxitilde;
}

void SoftPlus::invJ(State const &xitilde, State const &dxitilde, State &dxi) {
    dxi = dxitilde;
    this->invJ(xitilde, dxi);
}

std::ostream &operator<<(std::ostream &out, SoftPlus const &sp) {
    std::ostringstream os;
    os << "SoftPlus {\n\tk: " << sp.k << ",\n\tupperXi: " << sp.upperXi;
    os << ",\n\tupperXitilde: " << sp.upperXitilde << ",\n\teps: " << sp.eps;
    os << ",\n\tindex: " << sp.idx << "\n};\n\n";
    return out << os.str();
}

} // namespace vsgps

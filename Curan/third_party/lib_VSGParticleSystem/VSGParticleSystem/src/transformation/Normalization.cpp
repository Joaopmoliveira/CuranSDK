#include <nlohmann/json.hpp>
#include <fstream>
#include <vsgParticleSystem/transformation/Normalization.h>
#include <vsgParticleSystem/utils/stringUtils.h>

namespace vsgps {
Normalization::Normalization(size_t transformationIdx)
    : Transformation{transformationIdx}, limits{Eigen::VectorXd::Ones(3)} {}

void Normalization::operator()(State &xi) {
    for (auto const &l : limits)
        xi[&l - &limits[0]] /= l;
}

void Normalization::operator()(State const &xi, State &xitilde) {
    xitilde = xi;
    this->operator()(xitilde);
}

void Normalization::inv(State &xitilde) {
    for (auto const &l : limits)
        xitilde[&l - &limits[0]] *= l;
}

void Normalization::inv(State const &xi, State &xitilde) {
    xitilde = xi;
    this->inv(xitilde);
}

void Normalization::setLimits(State const &newLimits) { limits = newLimits; }

State const &Normalization::getLimits() const { return limits; }

Normalization Normalization::fromJson(std::string path) {
    replaceAll(path, "\\", "/");
    auto data = nlohmann::json::parse(std::ifstream(path))["limits"];
    State limits = State(data.size());
    limits.setZero();
    for (auto const &l : data)
        limits[&l - &data[0]] = l;
    Normalization n;
    n.setLimits(limits);
    return n;
}

std::ostream &operator<<(std::ostream &out, Normalization const &n) {
    std::ostringstream os;
    os << "Normalization {\n\tlimits: [";
    for (auto &l : n.limits) {
        os << l;
        if (&l - &n.limits[0] != n.limits.size() - 1) {
            os << ", ";
        }
    }

    os << "]\n};\n\n";
    return out << os.str();
}

} // namespace vsgps

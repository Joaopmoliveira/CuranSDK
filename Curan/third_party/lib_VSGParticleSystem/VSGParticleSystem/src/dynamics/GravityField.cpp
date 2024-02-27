#include <fstream>
#include <nlohmann/json.hpp>
#include <vsgParticleSystem/dynamics/GravityField.h>
#include <vsgParticleSystem/utils/stringUtils.h>
#include <vsgParticleSystem/utils/vectorUtils.h>
#include <vsgParticleSystem/utils/additionalFuncs.h>

namespace vsgps {
GravityField::GravityField(long gravityIndex, double nominalVelocity,
                           double lowerBound, double upperBound,
                           double sharpness, int dim,
                           vsgps::State const &eqPoint)
    : idx{gravityIndex}, v0{nominalVelocity}, lower{lowerBound},
      upper{upperBound}, k{sharpness}, dimension{dim} {
    assert(gravityIndex > -1);
    A = StateMatrix::Zero(dim, dim);
    A(idx, idx) = -1;
    this->eqPoint = eqPoint;
}

/**
 * Calculates the gravitational field contribution to the state update.
 *
 * @param xi The current state vector.
 * @param dxi The current state derivative vector.
 * @return The updated state vector after considering the gravitational field.
 */
State GravityField::operator()(State const &xi, State const &dxi) {
    std::pair<double, double> h = weight(xi(idx));
    gravityVector = v0 * A * (xi - eqPoint);
    return h.first * gravityVector + h.second * dxi;
}

void GravityField::setIndex(long newGravityIndex) {
    assert(newGravityIndex < dimension);
    idx = newGravityIndex;
    A.setZero();
    A(idx, idx) = -1;
}
long const &GravityField::getIndex() const { return idx; }

void GravityField::setDimension(int dim) {
    assert(idx < dim);
    dimension = dim;
    A = StateMatrix::Zero(dim, dim);
    A(idx, idx) = -1;
}
int const &GravityField::getDimension() const { return dimension; }

void GravityField::setLowerBound(double lowerBound) {
    assert(lowerBound < upper);
    lower = lowerBound;
}
double const &GravityField::getLowerBound() const { return lower; }

void GravityField::setUpperBound(double upperBound) {
    assert(lower < upperBound);
    upper = upperBound;
}
double const &GravityField::getUpperBound() const { return upper; }

void GravityField::setSharpness(double sharpness) {
    assert(k > 0);
    k = sharpness;
}
double const &GravityField::getSharpness() const { return k; }

void GravityField::setNominalVelocity(double velocity) {
    v0 = velocity;
    gravityVector((int)idx) = v0;
}
double const &GravityField::getNominalVelocity() const { return v0; }

GravityField GravityField::fromJson(std::string path) {
    replaceAll(path, "\\", "/");
    auto source = nlohmann::json::parse(std::ifstream(path));
    auto data = source["gravity"];
    int dim = source["dimension"];
    auto eqPointDisp = source["eqPoint"];
    State eqPoint = State::Zero(dim, 1);
    if (eqPointDisp.is_array()) {
        int index = 0;
        for (auto const &d : eqPointDisp)
            eqPoint[index++] = (double)d;
    } else if (eqPointDisp.is_object() && eqPointDisp.contains("disp")) {
        int index = 0;
        for (auto const &d : eqPointDisp["disp"])
            eqPoint[index++] = (double)d;
    }
    long idxF = ((long)data["idxF"]) - 1l;
    return {idxF,      data["vel"], data["lower"], data["upper"],
            data["k"], dim,         eqPoint};
}

std::ostream &operator<<(std::ostream &out, GravityField const &g) {
    std::ostringstream os;
    os << "GravityField {\n\tindex: " << g.idx << ",\n\t";
    os << "lower: " << g.lower << ",\n\tupper: " << g.upper << ",\n\t";
    os << "sharpness: " << g.k;
    os << ",\n\teqPoint: [";
    int i = 0;
    for (auto const &e : g.eqPoint) {
        os << e;
        if (i++ < g.eqPoint.size() - 1) {
            os << ", ";
        }
    }
    os << "],\n\tv0: " << g.v0;
    os << "\n};\n\n";
    return out << os.str();
}

} // namespace vsgps

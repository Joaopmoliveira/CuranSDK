#include <fstream>
#include <sstream>
#include <vsgParticleSystem/dynamics/Dynamics.h>
#include <vsgParticleSystem/utils/stringUtils.h>

namespace vsgps {
Dynamics::Dynamics() {}

Dynamics Dynamics::fromJson(std::string source) {
    replaceAll(source, "\\", "/");
    auto sourceData = nlohmann::json::parse(std::ifstream(source));
    auto info = sourceData["dynamics"];
    int dim = sourceData["dimension"];
    auto velIdxs = sourceData["velIn"];
    Dynamics d;

    d.setVelocityIndexes({velIdxs[0], velIdxs[1], velIdxs[2]});
    d.setDimension(dim);

    std::vector<StateMatrix> A, invSigma;
    std::vector<State> mu, b;
    std::vector<double> denominators, priors;
    StateMatrix tempA(dim, dim), tempInvSigma(dim, dim);
    State tempb(dim), tempmu(dim);
    for (auto const &d : info) {
        for (size_t col = 0; col < dim; col++) {
            for (size_t row = 0; row < dim; row++) {
                tempA(row, col) = d["A"][col][row];
                tempInvSigma(row, col) = d["invSigma"][col][row];
            }
            tempb[col] = d["b"][col];
            tempmu[col] = d["mu"][col];
        }
        A.push_back(tempA);
        invSigma.push_back(tempInvSigma);
        b.push_back(tempb);
        mu.push_back(tempmu);
        denominators.push_back(d["denominator"]);
        priors.push_back(d["prior"]);
    }
    d.setA(A);
    d.setinvSigma(invSigma);
    d.setb(b);
    d.setmu(mu);
    d.setdenominators(denominators);
    d.setpriors(priors);

    State eqPoint = State(dim);
    eqPoint.setZero();
    if (sourceData.contains("eqPoint") && sourceData["eqPoint"].is_array())
        for (auto const &d : sourceData["eqPoint"])
            eqPoint[&d - &sourceData["eqPoint"][0]] = d;
    else if (sourceData.contains("eqPoint") &&
             sourceData["eqPoint"].is_object() &&
             sourceData["eqPoint"].contains("normalied")) {
        int i = 0;
        for (auto const &d : sourceData["eqPoint"]["normalized"])
            eqPoint[i++] = d;
    }

    d.setEquilibriumPoint(eqPoint);

    return d;
}

std::vector<StateMatrix> const &Dynamics::getA() const { return A; }
void Dynamics::setA(std::vector<StateMatrix> const &inA) { A = inA; }

std::vector<StateMatrix> const &Dynamics::getinvSigma() const {
    return invSigma;
}
void Dynamics::setinvSigma(std::vector<StateMatrix> const &ininvSigma) {
    invSigma = ininvSigma;
}

std::vector<State> const &Dynamics::getb() const { return b; }
void Dynamics::setb(std::vector<State> const &inb) { b = inb; }

std::vector<State> const &Dynamics::getmu() const { return mu; }
void Dynamics::setmu(std::vector<State> const &inmu) { mu = inmu; }

std::vector<double> const &Dynamics::getpriors() const { return priors; }
void Dynamics::setpriors(std::vector<double> const &inpriors) {
    priors = inpriors;
}

std::vector<double> const &Dynamics::getdenominators() const {
    return denominators;
}
void Dynamics::setdenominators(std::vector<double> const &inden) {
    denominators = inden;
}

bool hasInf(State const &xi) {
    for (auto const &x : xi) {
        if (std::isinf(x) || std::isnan(x))
            return true;
    }
    return false;
}

double Dynamics::findOffset(State const &xi) const {
    double offset = .5 * (xi - mu[0]).dot(invSigma[0] * (xi - mu[0]));
    for (size_t i = 1; i < A.size(); i++) {
        double o = .5 * (xi - mu[i]).dot(invSigma[i] * (xi - mu[i]));
        if (o < offset && !std::isinf(o) && !std::isnan(o))
            offset = o;
    }
    return std::isinf(offset) || std::isnan(offset)
               ? std::numeric_limits<double>::infinity() * -1
               : offset;
}

double Dynamics::N(size_t idx, State const &xi) const {
    double exponent = -.5 * (xi - mu[idx]).dot(invSigma[idx] * (xi - mu[idx]));
    double num =
        std::exp(exponent == -currentOffset ? 0 : exponent + currentOffset);
    return num / denominators[idx];
}

double Dynamics::h(size_t idx, State const &xi) const {
    double sum = 0;
    double numerator = priors[idx] * N(idx, xi);
    for (auto const &m : mu)
        sum += priors[&m - &mu[0]] * N(&m - &mu[0], xi);
    return numerator / sum;
}

State Dynamics::operator()(State const &xi) {
    State dxi(dimension);
    dxi.setZero();
    currentOffset = findOffset(xi);
    int idx = 0;
    for (auto const &a : A) {
        dxi += h(idx, xi) * (A[idx] * xi + b[idx]);
        idx++;
    }
    return dxi;
}

std::string const Dynamics::logAb() const {
    std::ostringstream os;
    os << "Values of A:\n";
    int i = 0;
    for (auto const &a : A) {
        os << "A[" << i << "] =\n" << a << "\n";
        i++;
    }
    os << "\nValues of b:\n";
    i = 0;
    for (auto const &B : b) {
        os << "b[" << i << "] =\n" << B << "\n";
        i++;
    }
    os << "\n";
    return os.str();
}

std::string const Dynamics::logProps() const {
    std::ostringstream os;
    os << "Values of invSigma:\n";
    int i = 0;
    for (auto const &is : invSigma) {
        os << "invSigma[" << i << "] =\n" << is << "\n";
        i++;
    }
    os << "\nValues of means:\n";
    i = 0;
    for (auto const &m : mu) {
        os << "mu[" << i << "] =\n" << m << "\n";
        i++;
    }
    os << "\nValues of denominator:\n";
    i = 0;
    for (auto const &d : denominators) {
        os << "denominator[" << i << "] = " << d << "\n";
        i++;
    }
    os << "\nValues of priors:\n";
    i = 0;
    for (auto const &p : priors) {
        os << "priors[" << i << "] = " << p << "\n";
        i++;
    }
    os << "\n";
    return os.str();
}

std::string const Dynamics::log(State const &xi) const {
    State dxi(dimension);
    dxi.setZero();
    double o = findOffset(xi);
    int idx = 0;
    double H;
    std::ostringstream os;
    os << "Dynamics:\noffset = " << o << "\n";
    for (auto const &a : A) {
        H = h(idx, xi);
        os << "H[" << idx << "] = " << H << "\n";
        dxi += H * (A[idx] * xi + b[idx]);
        os << "dxi[" << idx << "] =\n" << dxi << "\n";
        idx++;
    }
    return os.str();
}

int Dynamics::getDimension() const { return dimension; }

void Dynamics::setDimension(int dim) { dimension = dim; }

std::array<int, 3> const &Dynamics::getVelocityIndexes() const {
    return velIndices;
}

void Dynamics::setVelocityIndexes(std::array<int, 3> const &vi) {
    velIndices = vi;
}

State const &Dynamics::getEquilibriumPoint() const { return eqPoint; }

void Dynamics::setEquilibriumPoint(State const &eq) { eqPoint = eq; }

std::ostream &operator<<(std::ostream &os, Dynamics const &d) {
    os << "Dynamics {\n";
    os << "\tdimension: " << d.getDimension() << "\n";
    os << "\tvelocity indexes: [" << d.getVelocityIndexes()[0] << ", "
       << d.getVelocityIndexes()[1] << ", " << d.getVelocityIndexes()[2] << "]\n";
    os << "\tlogAb:\n" << d.logAb();
    os << "\tlogProps:\n" << d.logProps() << "};\n\n";
    return os;
}

} // namespace vsgps

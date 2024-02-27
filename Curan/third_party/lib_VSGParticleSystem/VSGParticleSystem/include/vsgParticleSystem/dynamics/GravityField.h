#pragma once

#include "State.h"

namespace vsgps {
class GravityField {
  public:
    GravityField(long gravityIndex = 0, double nominalVelocity = 0.0,
                 double lowerBound = -1, double upperBound = 1,
                 double sharpness = 1, int dim = 3,
                 vsgps::State const &eqPoint = vsgps::State::Zero(3, 1));
    State operator()(State const &xi, State const &dxi);
    static GravityField fromJson(std::string path);

    void setIndex(long newGravityIndex);
    long const &getIndex() const;
    void setDimension(int dim);
    int const &getDimension() const;
    void setLowerBound(double lowerBound);
    double const &getLowerBound() const;
    void setUpperBound(double upperBound);
    double const &getUpperBound() const;
    void setSharpness(double sharpness);
    double const &getSharpness() const;
    void setNominalVelocity(double velocity);
    double const &getNominalVelocity() const;

    friend std::ostream &operator<<(std::ostream &out, GravityField const &g);

  protected:
    long idx;
    int dimension;
    double lower, upper, k, v0;
    StateMatrix A;
    State gravityVector, eqPoint;
    double prior = .5;

    constexpr std::pair<double, double> weight(double x) {
        auto p = g(x);
        return {p.first / (p.first + p.second),
                p.second / (p.first + p.second)};
    }

    constexpr inline double offsetExponent(double x, double mu, double sigma) {
        return .5 * ((x - mu) / sigma) * ((x - mu) / sigma);
    }

    constexpr inline double N(double x, double mu, double sigma,
                              double offset) {
        double exponent = -.5 * ((x - mu) / sigma) * ((x - mu) / sigma);
        return std::exp(exponent + offset) / (sigma * sqrt(2 * std::acos(-1)));
    }

    constexpr std::pair<double, double> g(double x) {
        double offsetLower = offsetExponent(x, lower, k);
        double offsetUpper = offsetExponent(x, upper, k);
        double offset = std::min(offsetLower, offsetUpper);
        return {prior * N(x, lower, k, offset), prior * N(x, upper, k, offset)};
    }
};
} // namespace vsgps

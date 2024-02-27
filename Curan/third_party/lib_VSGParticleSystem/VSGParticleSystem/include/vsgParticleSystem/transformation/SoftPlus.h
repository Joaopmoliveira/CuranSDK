#pragma once

#include "Transformation.h"

namespace vsgps {

/*
 * f(x) = ln(-1 + e^{kx}) / k
 */
class SoftPlus : public Transformation {
  public:
    inline static std::string label = "softplus";

    SoftPlus(size_t transformationIdx = 1,
             double otherEPS = __DBL_EPSILON__ * 10.0);

    virtual void operator()(State &xi);
    virtual void operator()(State const &xi, State &xitilde);
    virtual void inv(State &xitilde);
    virtual void inv(State const &xitilde, State &xi);
    virtual void invVel(State const &xiDeformed, State &dxi);
    static SoftPlus fromJson(std::string path);

    friend std::ostream &operator<<(std::ostream &out, SoftPlus const &sp);

    void J(State const &xi, State &dxi);
    void invJ(State const &xitilde, State &dxitilde);
    void J(State const &xi, State const &dxi, State &dxitilde);
    void invJ(State const &xitilde, State const &dxitilde, State &dxi);

    void setK(double newK);
    double const getK() const;
    void setUpperXi(double newUpperXi);
    double const getUpperXi() const;
    void setUpperXitilde(double newUpperXitilde);
    double const getUpperXitilde() const;

  protected:
    double k, upperXi, upperXitilde, eps;

    inline double softplus(double x) {
        return std::log(-1.0 + std::exp(k * x)) / k;
    }

    inline double invsoftplus(double x) {
        return std::log(1.0 + std::exp(k * x)) / k;
    }

    inline double dsoftplus(double x) {
        return 1.0 / (1.0 - std::exp(-k * x));
    }

    inline double dinvsoftplus(double x) {
        return 1.0 / (1.0 + std::exp(-k * x));
    }
};

using Force2Deform = SoftPlus;

} // namespace vsgps

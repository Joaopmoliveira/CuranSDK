#pragma once

#include "State.h"
#include <array>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace vsgps {
class Dynamics {
  public:
    Dynamics();
    static Dynamics fromJson(std::string source);

    std::vector<StateMatrix> const &getA() const;
    void setA(std::vector<StateMatrix> const &inA);
    std::vector<StateMatrix> const &getinvSigma() const;
    void setinvSigma(std::vector<StateMatrix> const &ininvSigma);
    std::vector<State> const &getb() const;
    void setb(std::vector<State> const &inb);
    std::vector<State> const &getmu() const;
    void setmu(std::vector<State> const &inmu);
    std::vector<double> const &getpriors() const;
    void setpriors(std::vector<double> const &inpriors);
    std::vector<double> const &getdenominators() const;
    void setdenominators(std::vector<double> const &inden);

    int getDimension() const;
    void setDimension(int dim);
    std::array<int, 3> const &getVelocityIndexes() const;
    void setVelocityIndexes(std::array<int, 3> const &vi);

    double h(size_t idx, State const &xi) const;
    State operator()(State const &xi);

    std::string const log(State const &xi) const;
    std::string const logAb() const;
    std::string const logProps() const;

    State const &getEquilibriumPoint() const;
    void setEquilibriumPoint(State const &eq);

    friend std::ostream &operator<<(std::ostream &os, Dynamics const &d);

  protected:
    std::vector<State> b, mu;
    std::vector<StateMatrix> A, invSigma;
    std::vector<double> denominators, priors;
    std::array<int, 3> velIndices;
    State eqPoint;
    int dimension;
    double currentOffset;

    double N(size_t idx, State const &xi) const;
    double findOffset(State const &xi) const;
};
} // namespace vsgps

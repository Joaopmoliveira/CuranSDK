#pragma once

#include <fstream>
#include <iostream>
#include <math.h>
#include <vsg/all.h>
#include "../dynamics/State.h"

namespace vsgps {
// https://www.desmos.com/calculator/qhngmu0ysj
constexpr double staircase(double x, size_t degree) {
    if (degree == 0)
        return x - std::sin(x);
    else
        return staircase(x, --degree);
}
vsg::dvec4 coloringLinear(double x, double darkLevel = 1.0);
vsg::dvec4 red2blue(double x, double darkLevel = 1.0);
inline void printVector(std::ofstream &file, State const &xi) {
    file << xi(0) << ", " << xi(1) << ", " << xi(2) << "\n";
}
} // namespace vsgps

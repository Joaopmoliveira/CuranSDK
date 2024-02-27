#pragma once

#include "../dynamics/State.h"

namespace vsgps {
// https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
class SecurityRegion {
  public:
    SecurityRegion(Eigen::Matrix<double, 2, 4> const &corners =
                       Eigen::Matrix<double, 2, 4>::Zero());

    void validate(State const &xiDisp);

    static SecurityRegion fromJson(std::string path); 

    friend std::ostream &operator<<(std::ostream &os, SecurityRegion const &sr);

  protected:
  Eigen::Matrix2d R;
  Eigen::Vector2d origin;
};
} // namespace vsgps

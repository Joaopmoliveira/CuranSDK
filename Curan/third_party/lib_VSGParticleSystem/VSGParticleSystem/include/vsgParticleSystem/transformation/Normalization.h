#pragma once

#include "Transformation.h"

namespace vsgps {
class Normalization : public Transformation {
  public:
    inline static std::string label = "norm";

    Normalization(size_t transformationIdx = 1);
    virtual void operator()(State &xi);
    virtual void operator()(State const &xi, State &xitilde);
    virtual void inv(State &xitilde);
    virtual void inv(State const &xi, State &xitilde);
    static Normalization fromJson(std::string path);

    friend std::ostream &operator<<(std::ostream &out, Normalization const &n);

    void setLimits(State const &newLimits);
    State const &getLimits() const;

  protected:
    State limits;
};

using Deform2Norm = Normalization;

} // namespace vsgps

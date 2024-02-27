#pragma once

#include "../dynamics/State.h"
#include <string>

namespace vsgps {

class Transformation {
  public:
    inline static std::string label = "base";

    Transformation(size_t transformationIdx);
    virtual void operator()(State &xi);
    virtual void operator()(State const &xi, State &xitilde);
    virtual void inv(State &xitilde);
    virtual void inv(State const &xitilde, State &xi);
    virtual void invVel(State &dxitilde);
    virtual void invVel(State const &dxitilde, State &dxi);
    static Transformation fromJson(std::string path);
    friend std::ostream &operator<<(std::ostream &out, Transformation const &t);
    void setIndex(size_t newIdx);
    size_t const &getIndex() const;

  protected:
    size_t idx;

  private:
    void UnimplementedMethod();
};
} // namespace vsgps

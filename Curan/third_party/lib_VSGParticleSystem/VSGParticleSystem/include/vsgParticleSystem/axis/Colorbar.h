#pragma once

#include "../builder/BuilderProps.h"
#include <vector>

namespace vsgps {
class Indicator {
  public:
    Indicator(BuilderProps &props);

    vsg::ref_ptr<vsg::Group> root;

    void changeColor(double z);
    void changeColor(vsg::dvec4 color);
    void move(vsg::dvec3 const &pos);
    void scale(double len = 1.0);

  protected:
    vsg::ref_ptr<vsg::MatrixTransform> body;
    vsg::ref_ptr<vsg::vec4Array> colors;
    vsg::dvec3 position = {.0, .0, .0};
    double thickness = .05, length = 1.0;

    void updateBody();
};

class Colorbar {
  public:
    Colorbar(BuilderProps &props);

    void recompute();

    void setCenter(vsg::dvec3 const &c);

  protected:
    vsg::ref_ptr<vsg::Group> root;
    std::vector<Indicator> indicators;
    BuilderProps props;
    size_t maxIndicators = 50;
    vsg::dvec3 center, radius, step, origin;
    bool fixCenter = true, fixRadius = false;
    double ratio = .95;
};
} // namespace vsgps

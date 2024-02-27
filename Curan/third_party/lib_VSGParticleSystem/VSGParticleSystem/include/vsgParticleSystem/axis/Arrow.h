#pragma once

#include "../builder/BuilderProps.h"
#include <vsg/all.h>

namespace vsgps {
// Class based on the
// [HumanRoboticSDK](https://github.com/Human-Robotics-Lab/HumanRoboticsSDK)
// repository, with some additional features
class Arrow {
  public:
    Arrow(vsg::dvec3 const &origin = {0, 0, 0},
          vsg::dvec3 const &direction = {0, 0, 1});

    void makeArrow();
    void makeArrow(BuilderProps const &props);
    void recomputeArrow(vsg::dvec3 const &origin, vsg::dvec3 const &direction);
    void addToScene(vsg::ref_ptr<vsg::Group> &scene);

    void setNorm(double const &in);
    double const &getNorm() const;

  protected:
    vsg::ref_ptr<vsg::Group> root;
    vsg::ref_ptr<vsg::MatrixTransform> adjustArrow;
    vsg::ref_ptr<vsg::MatrixTransform> transform;
    vsg::ref_ptr<vsg::vec4Array> colors;
    vsg::dvec3 origin, ending, k;
    // The percentage of how much the arrow's body takes relative to the arrow's
    // tip within the norm
    double ratio = .95, norm = .0, theta = .0;

    void setupArrow(vsg::ref_ptr<vsg::Node> body, vsg::ref_ptr<vsg::Node> tip);

    inline void setupBody(vsg::ref_ptr<vsg::Node> body) {
        auto adjustingBody = vsg::MatrixTransform::create();
        adjustingBody->matrix = vsg::scale(.02, .02, ratio);
        adjustingBody->matrix = adjustingBody->transform(
            vsg::translate(.0, .0, -(1.0 - ratio) * .5));

        adjustingBody->addChild(body);
        adjustArrow->addChild(adjustingBody);
    }

    inline void setupTip(vsg::ref_ptr<vsg::Node> tip) {
        auto adjustingBody = vsg::MatrixTransform::create();
        adjustingBody->matrix = vsg::scale(.07, .07, 1 - ratio);
        adjustingBody->matrix =
            adjustingBody->transform(vsg::translate(.0, .0, ratio * .5));

        adjustingBody->addChild(tip);
        adjustArrow->addChild(adjustingBody);
    }

    inline void recomputeArrow() {
        transform->matrix = vsg::scale(1.0, 1.0, norm);
        transform->matrix = transform->transform(vsg::rotate(theta, k));
        transform->matrix = transform->transform(vsg::translate(origin));
    }
};
} // namespace vsgps

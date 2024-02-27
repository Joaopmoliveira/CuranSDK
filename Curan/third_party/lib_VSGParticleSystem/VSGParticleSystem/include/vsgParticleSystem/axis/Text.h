#pragma once

#include <vsg/all.h>

namespace vsgps {

class Text {
  public:
    Text(std::string const &value, vsg::dvec3 const &pos,
         vsg::ref_ptr<vsg::Options> const &options, double fontSize = .2);

    void addToScene(vsg::ref_ptr<vsg::Group> const &scene);
    void addToScene(vsg::ref_ptr<vsg::Group> &scene, size_t idx);
    void moveText(vsg::dvec3 const &newPos);
    void changeText(std::string const &newValue);

  private:
    vsg::ref_ptr<vsg::stringValue> label;
    vsg::ref_ptr<vsg::StandardLayout> layout;
    vsg::ref_ptr<vsg::Text> text;
    vsg::ref_ptr<vsg::Options> options;
};
} // namespace vsgps

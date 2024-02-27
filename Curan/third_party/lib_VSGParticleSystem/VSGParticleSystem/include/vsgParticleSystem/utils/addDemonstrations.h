#pragma once

#include "../builder/BuilderProps.h"
#include <vector>
#include <vsg/maths/vec3.h>

namespace vsgps {

void addData(std::string filename, BuilderProps const &props,
             std::vector<vsg::dvec3> &data, double radius = 0.05);

using DataTransform = std::function<void(State const &, vsg::dvec3 &)>;
void addDataWithTransform(std::string filename, BuilderProps const &props,
                          std::vector<vsg::dvec3> &data,
                          DataTransform const &dt, double radius = 0.05);

void addData(std::string filename, std::vector<vsg::dvec3> &data,
             double radius = 0.05);

double computeRadius(std::vector<vsg::dvec3> const &data,
                     vsg::dvec3 const &center);
} // namespace vsgps
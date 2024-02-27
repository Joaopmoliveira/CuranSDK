#pragma once

#include "ResolutionBuilder.h"
#include <vsg/all.h>

namespace vsgps {
struct BuilderProps {
    vsg::ref_ptr<vsg::ResolutionBuilder> builder;
    vsg::ref_ptr<vsg::Options> options;
    vsg::GeometryInfo *geomInfo;
    vsg::StateInfo stateInfo;
    vsg::ref_ptr<vsg::Group> root;
};
}
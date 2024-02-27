#include <vsgParticleSystem/utils/vsgAdditionalFunctions.h>

std::pair<vsg::dvec3, vsg::dvec3>
computeCenterRadius(vsg::ref_ptr<vsg::Group> &root) {
    vsg::ComputeBounds cb;
    root->accept(cb);
    vsg::dvec3 min = cb.bounds.min;
    vsg::dvec3 max = cb.bounds.max;
    vsg::dvec3 center = (min + max) * .5;
    vsg::dvec3 radius = max - min;
    return {center, radius};
}
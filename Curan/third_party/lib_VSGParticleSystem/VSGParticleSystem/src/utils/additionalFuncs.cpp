#include <vsgParticleSystem/utils/additionalFuncs.h>
#include <vsgParticleSystem/utils/vsgAdditionalFunctions.h>
#include <vsgParticleSystem/utils/vsgAdditionalOperators.h>

namespace vsgps {
vsg::dvec4 coloringLinear(double x, double darkLevel) {
    vsg::dvec3 a = {.5, .5, .5};
    vsg::dvec3 b = {.5, .5, .5};
    vsg::dvec3 c = {1, 1, 1};
    vsg::dvec3 d = {0, 1.0 / 3.0, 2.0 / 3.0};
    double V = x < .0 ? darkLevel : 1.;
    vsg::dvec3 color = a + b * cos(6.28318 * (c * x + d));
    return vsg::dvec4(color * V, 1);
}
vsg::dvec4 red2blue(double x, double darkLevel) {
    vsg::dvec3 a = {.5, .0, .5};
    vsg::dvec3 b = {.5, .0, .5};
    vsg::dvec3 c = {1., 0., 1.};
    vsg::dvec3 d = {0, 0., 0.5};
    double V = x < .0 ? darkLevel : 1.;
    vsg::dvec3 color = a + b * cos(6.28318 * (c * x + d));
    return vsg::dvec4(color, 1);
}
} // namespace vsgps

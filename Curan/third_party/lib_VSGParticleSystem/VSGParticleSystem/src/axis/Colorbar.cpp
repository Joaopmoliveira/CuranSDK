#include <vsgParticleSystem/axis/Colorbar.h>
#include <vsgParticleSystem/utils/additionalFuncs.h>
#include <vsgParticleSystem/utils/vsgAdditionalFunctions.h>
#include <vsgParticleSystem/utils/vsgAdditionalOperators.h>

namespace vsgps {
Indicator::Indicator(BuilderProps &props) {
    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set(1.0, 0.0, 0.0);
    geomInfo.dy.set(0.0, 1.0, 0.0);
    geomInfo.dz.set(0.0, 0.0, 1.0);
    size_t numOfObjects = 10;
    geomInfo.positions = vsg::vec4Array::create(numOfObjects);
    colors = vsg::vec4Array::create(numOfObjects);
    geomInfo.colors = colors;
    for (auto &c : *colors)
        c.set(1.0, 1.0, 1.0, 1.0);
    colors->properties.dataVariance = vsg::DYNAMIC_DATA;

    vsg::StateInfo stateInfo;

    root = vsg::Group::create();
    body = vsg::MatrixTransform::create();
    updateBody();
    body->addChild(props.builder->createCylinder(geomInfo, stateInfo));
    root->addChild(body);
}

void Indicator::changeColor(double z) {
    vsg::dvec4 newColor = coloringLinear(z, .25);
    changeColor(newColor);
}

void Indicator::changeColor(vsg::dvec4 color) {
    for (auto &c : *colors) {
        c.set(color.r, color.g, color.b, color.a);
    }
    colors->dirty();
}

void Indicator::move(vsg::dvec3 const &pos) {
    position = pos;
    updateBody();
}

void Indicator::scale(double len) {
    length = len;
    updateBody();
}

void Indicator::updateBody() {
        body->matrix = vsg::scale(thickness, thickness, length);
        body->matrix = body->transform(vsg::translate(position));
}

Colorbar::Colorbar(BuilderProps &props) : props{props} {
    root = vsg::Group::create();
    props.root->addChild(root);
    auto [c, r] = computeCenterRadius(props.root);
    center = fixCenter ? center : c;
    radius = fixRadius ? radius : r * ratio;
    radius[0] = 0;
    radius[1] = 0;
    step = radius / (double)maxIndicators;
    origin = center - radius / 2.0;
    size_t i = 0;
    for (size_t i = 0; i < maxIndicators; i++) {
        indicators.push_back(Indicator(props));
        root->addChild(indicators[i].root);
        indicators[i].changeColor((double)i / (double)maxIndicators);
        indicators[i].move(origin + step * i);
        indicators[i].scale(vsg::length(step));
    }
}

void Colorbar::setCenter(vsg::dvec3 const &c) {
    center = c;
    fixCenter = true;
}

void Colorbar::recompute() {
    auto [c, r] = computeCenterRadius(props.root);
    center = fixCenter ? center : c;
    radius = fixRadius ? radius : r * ratio;
    radius[0] = 0;
    radius[1] = 0;
    step = radius / (double)maxIndicators;
    origin = center - radius / 2.0;
    size_t i = 0;
    for (auto &indicator : indicators) {
        indicator.changeColor((double)i / (double)maxIndicators);
        indicator.move(origin + step * i);
        indicator.scale(vsg::length(step));
        i++;
    }
}
} // namespace vsgps

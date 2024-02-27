#include <vsgParticleSystem/axis/Arrow.h>

namespace vsgps {
Arrow::Arrow(vsg::dvec3 const &origin, vsg::dvec3 const &direction) {
    root = vsg::Group::create();
    transform = vsg::MatrixTransform::create();
    root->addChild(transform);
    adjustArrow = vsg::MatrixTransform::create();
    adjustArrow->matrix = vsg::translate(.0, .0, .5);
    transform->addChild(adjustArrow);

    this->origin = origin;
    this->ending = origin + direction;
}

void Arrow::makeArrow(BuilderProps const &props) {
    setupArrow(props.builder->createCylinder(*props.geomInfo, props.stateInfo),
               props.builder->createCone(*props.geomInfo, props.stateInfo));
}
void Arrow::makeArrow() {
    auto builder = vsg::Builder::create();

    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set(1.0, 0.0, 0.0);
    geomInfo.dy.set(0.0, 1.0, 0.0);
    geomInfo.dz.set(0.0, 0.0, 1.0);
    geomInfo.positions = vsg::vec4Array::create(1);
    colors = vsg::vec4Array::create(1);
    colors->properties.dataVariance = vsg::DYNAMIC_DATA;
    geomInfo.colors = colors;
    for (auto &c : *colors)
        c.set(1.0, 1.0, 1.0, 1.0);

    vsg::StateInfo stateInfo;

    setupArrow(builder->createCylinder(geomInfo, stateInfo),
               builder->createCone(geomInfo, stateInfo));
}

void Arrow::setupArrow(vsg::ref_ptr<vsg::Node> body,
                       vsg::ref_ptr<vsg::Node> tip) {
    adjustArrow->matrix = vsg::translate(.0, .0, .5);
    setupBody(body);
    setupTip(tip);

    vsg::dvec3 direction = ending - origin;
    vsg::dvec3 directionNormalize = vsg::normalize(ending - origin);
    norm = vsg::length(direction);

    theta = std::acos(vsg::dot({.0, .0, 1.}, directionNormalize));
    k = vsg::normalize(vsg::cross({.0, .0, 1.}, directionNormalize));

    this->recomputeArrow();
}

void Arrow::addToScene(vsg::ref_ptr<vsg::Group> &scene) {
    scene->addChild(root);
}

void Arrow::setNorm(const double &in) {
    this->norm = norm;
    this->recomputeArrow();
}

void Arrow::recomputeArrow(vsg::dvec3 const &origin,
                           vsg::dvec3 const &direction) {
    norm = vsg::length(direction);
    vsg::dvec3 directionNormalize = vsg::normalize(direction);
    theta = std::acos(vsg::dot({.0, .0, 1.}, directionNormalize));
    if (double dotDN = vsg::dot(directionNormalize, {.0, .0, 1.});
        std::abs(dotDN - 1.) < __DBL_EPSILON__ * 1.01) {
        k = vsg::dvec3{.0, .0, 1.} * dotDN;
    } else {
        k = vsg::normalize(vsg::cross({.0, .0, 1.}, directionNormalize));
    }
    this->origin = origin;
    this->ending = origin + direction;
    this->recomputeArrow();
}
} // namespace vsgps

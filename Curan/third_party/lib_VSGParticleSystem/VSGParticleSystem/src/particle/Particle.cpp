#include <vsgParticleSystem/particle/Particle.h>
namespace vsgps {
void SimpleParticle::defineGeometry(
    vsg::ref_ptr<vsg::MatrixTransform> &parent,
    vsg::ref_ptr<vsg::ResolutionBuilder> const &builder,
    vsg::GeometryInfo const &geomInfo, vsg::StateInfo const &stateInfo,
    size_t res) {
    parent->addChild(builder->createSphere(geomInfo, stateInfo, res, res));
}

SimpleParticle::SimpleParticle(BuilderProps props, vsg::dvec3 const &pos,
                               vsg::dvec4 const &initColor, double radius,
                               size_t resolution)
    : radius{radius} {

    vsg::StateInfo stateInfo = props.stateInfo;
    vsg::ref_ptr<vsg::ResolutionBuilder> builder = props.builder;

    stateInfo.lighting = false;

    vsg::GeometryInfo geomInfo;
    geomInfo.positions = vsg::vec4Array::create(1);

    color = vsg::vec4Array::create(1);
    color->properties.dataVariance = vsg::DYNAMIC_DATA;
    geomInfo.colors = color;
    for (auto &c : *color)
        c.set(initColor.r, initColor.g, initColor.b, initColor.a);

    root = vsg::Group::create();
    transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(pos);

    scalling = vsg::MatrixTransform::create();
    scalling->matrix = vsg::scale(radius);

    size_t numOfColumns = resolution;
    defineGeometry(scalling, builder, geomInfo, stateInfo, numOfColumns);
    transform->addChild(scalling);
    root->addChild(transform);
}

void SimpleParticle::setRadius(double r) {
    radius = r;
    scalling->matrix = vsg::scale(radius);
}

void SimpleParticle::addToScene(vsg::ref_ptr<vsg::Group> const &scene) const {
    scene->addChild(root);
}

void SimpleParticle::addToScene(vsg::ref_ptr<vsg::Group> &scene,
                                size_t idx) const {
    scene->children[idx] = root;
}

void SimpleParticle::move(vsg::dvec3 const &newPos) {
    transform->matrix = vsg::translate(newPos);
}
void SimpleParticle::move(vsg::vec3 const &newPos) {
    move(vsg::dvec3(newPos.x, newPos.y, newPos.z));
}
void SimpleParticle::updateColor(vsg::dvec4 const &newColor) {
    for (auto &c : *(color))
        c.set(newColor.r, newColor.g, newColor.b, newColor.a);
    color->dirty();
}

void SimpleParticle::updateColor(vsg::vec4 const &newColor) {
    updateColor(vsg::dvec4(newColor.r, newColor.g, newColor.b, newColor.a));
}

double const SimpleParticle::getRadius() const { return radius; }

void CylinderParticle::defineGeometry(
    vsg::ref_ptr<vsg::MatrixTransform> &parent,
    vsg::ref_ptr<vsg::ResolutionBuilder> const &builder,
    vsg::GeometryInfo const &geomInfo, vsg::StateInfo const &stateInfo,
    size_t res) {
    parent->addChild(builder->createCylinder(geomInfo, stateInfo));
}

CylinderParticle::CylinderParticle(BuilderProps props, vsg::dvec3 const &pos,
                                   vsg::dvec4 const &initColor, double radius,
                                   size_t resolution)
    : radius{radius} {

    vsg::StateInfo stateInfo = props.stateInfo;
    vsg::ref_ptr<vsg::ResolutionBuilder> builder = props.builder;

    stateInfo.lighting = false;

    vsg::GeometryInfo geomInfo;
    geomInfo.positions = vsg::vec4Array::create(1);

    color = vsg::vec4Array::create(1);
    color->properties.dataVariance = vsg::DYNAMIC_DATA;
    geomInfo.colors = color;
    for (auto &c : *color)
        c.set(initColor.r, initColor.g, initColor.b, initColor.a);

    root = vsg::Group::create();
    transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(pos);

    scalling = vsg::MatrixTransform::create();
    scalling->matrix = vsg::scale(radius);

    size_t numOfColumns = resolution;
    defineGeometry(scalling, builder, geomInfo, stateInfo, numOfColumns);
    transform->addChild(scalling);
    root->addChild(transform);
}

void CylinderParticle::setRadius(double r) {
    radius = r;
    scalling->matrix = vsg::scale(radius);
}

void CylinderParticle::addToScene(vsg::ref_ptr<vsg::Group> const &scene) const {
    scene->addChild(root);
}

void CylinderParticle::addToScene(vsg::ref_ptr<vsg::Group> &scene,
                                  size_t idx) const {
    scene->children[idx] = root;
}

void CylinderParticle::move(vsg::dvec3 const &newPos) {
    transform->matrix = vsg::translate(newPos);
}

void CylinderParticle::move(vsg::vec3 const &newPos) {
    move(vsg::dvec3(newPos.x, newPos.y, newPos.z));
}

void CylinderParticle::updateColor(vsg::dvec4 const &newColor) {
    for (auto &c : *(color))
        c.set(newColor.r, newColor.g, newColor.b, newColor.a);
    color->dirty();
}

void CylinderParticle::updateColor(vsg::vec4 const &newColor) {
    updateColor(vsg::dvec4(newColor.r, newColor.g, newColor.b, newColor.a));
}

double const CylinderParticle::getRadius() const { return radius; }

Particle::Particle(BuilderProps props, vsg::dvec3 const &position,
                   vsg::dvec3 const &velocity, double radius,
                   vsg::dvec4 const &initColor, size_t resolution)
    : SimpleParticle(props, position, initColor, radius, resolution),
      position{position}, velocity{velocity} {
    previousCheck = std::chrono::steady_clock::now();
}

void Particle::changeFactor(double delta) {
    factor = factor <= 0 && delta < 0 ? 0 : factor + delta;
}

double round(double x, size_t units) {
    return std::round(x * std::pow(10.0, units)) / std::pow(10.0, units);
}

vsg::dvec3 round(vsg::dvec3 const &vector, size_t units) {
    vsg::dvec3 result;
    for (size_t i = 0; i < 3; i++)
        result[i] = round(vector[i], units);
    return result;
}

void Particle::move(vsg::dvec3 const &newVel) {
    auto current = std::chrono::steady_clock::now();
    dt =
        std::chrono::duration<double>(current - previousCheck).count() * factor;
    vsg::dvec3 newPos = newVel * dt;
    transform->matrix = transform->transform(vsg::translate(newPos));
    position += newPos;
    velocity = newVel;
    previousCheck = current;
}

void Particle::tick() { previousCheck = std::chrono::steady_clock::now(); }

void Particle::move(vsg::vec3 const &newVel) {
    move(vsg::dvec3(newVel.x, newVel.y, newVel.z));
}

vsg::dvec3 const SimpleParticle::getPosition() const {
    return vsg::dvec3(transform->matrix[3][0], transform->matrix[3][1],
                      transform->matrix[3][2]);
}

vsg::dvec3 const Particle::getPosition() const { return position; }

vsg::dvec3 const Particle::getVelocity() const { return velocity; }

std::ostream &operator<<(std::ostream &os, Particle const &p) {
    return os << "Particle:\n\tposition: " << p.position
              << "\n\tvelocity: " << p.velocity << "\n";
}

void Particle::setVelocity(vsg::dvec3 vel) { velocity = vel; }

double const Particle::getDeltaTime() const { return dt; }
}
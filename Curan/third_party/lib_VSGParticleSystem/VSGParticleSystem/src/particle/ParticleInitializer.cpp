#include <vsgParticleSystem/particle/ParticleInitializer.h>

namespace vsgps {
vsg::dvec3 RandomParticleInitializer::givePosition() {
    return vsg::dvec3(random(rangeX), random(rangeY), random(rangeZ));
}

vsg::dvec3 RandomParticleInitializer::giveVelocity() {
    return vsg::dvec3(random(rangeDX), random(rangeDY), random(rangeDZ));
}

void CircleParticleInitializer::setVelocityRanges(SpatialRange const &dx,
                                                  SpatialRange const &dy,
                                                  SpatialRange const &dz) {
    this->dx = dx;
    this->dy = dy;
    this->dz = dz;
}

vsg::dvec3 CircleParticleInitializer::givePosition() {
    double theta = random(0, vsg::PI);
    double phi = random(0, 2.0 * vsg::PI);
    return center + radius * vsg::dvec3(std::sin(theta) * std::cos(phi),
                                        std::cos(theta) * std::cos(phi),
                                        std::sin(phi) * verticalSpaceEnabled);
}

vsg::dvec3 CircleParticleInitializer::giveVelocity() {
    return vsg::dvec3(random(dx), random(dy), random(dz));
}

ShapeParticleInitializer::ShapeParticleInitializer(double rho, size_t sides,
                                                   vsg::dvec3 c)
    : radiansStep{2.0 * vsg::PI / (double)sides}, center{c}, radius{rho} {}

vsg::dvec3 ShapeParticleInitializer::givePosition() {
    double theta = radiansStep * currentSidePosition;
    currentSidePosition++;
    return center + vsg::dvec3{radius * std::cos(theta), radius * std::sin(theta), 0};
}

vsg::dvec3 ShapeParticleInitializer::giveVelocity() {
    double theta = radiansStep * currentSideVelocity;
    currentSideVelocity++;
    return {-radius * std::cos(theta), -radius * std::sin(theta), random()};
}

void ShapeParticleInitializer::setElevation(double e) { center[2] = e; }
double ShapeParticleInitializer::getElevation() const {
    return center[2];
}
void ShapeParticleInitializer::setStep(double dt) { radiansStep = dt; }
double const &ShapeParticleInitializer::getStep() const { return radiansStep; }
} // namespace vsgps

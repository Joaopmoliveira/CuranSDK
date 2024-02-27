#pragma once

#include "../dynamics/State.h"
#include "Particle.h"
#include "ParticleInitializer.h"
#include <functional>
#include <vsg/all.h>

namespace vsgps {

class Emitter {
  public:
    Emitter(BuilderProps const &props);
    void setDynamicFunction(DynamicFunction const &dynFun);
    void moveParticles();
    void setStates(std::array<int, 3> const &velIdxs, size_t dim);
    void setNumberOfDiameters(double n);
    void increaseFactor(double delta = 0.01);
    void decreaseFactor(double delta = -0.01);
    void populate(size_t nParticles, ParticleInitializer<vsg::dvec3> &pi,
                  double radius = .1);
    void tick();

  protected:
    std::vector<Particle> particles;
    std::vector<vsg::dvec3> lastPositions;
    BuilderProps props;
    double nDiameters = 1;
    std::vector<State> states;
    DynamicFunction dynamicFunction;

    double random(double a = 0, double b = 1);
    double random(SpatialRange range);
    void addTail(size_t idx, Particle const &p);
};
} // namespace vsgps

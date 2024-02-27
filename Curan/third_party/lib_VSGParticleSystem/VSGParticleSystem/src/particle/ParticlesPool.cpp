#include <vsgParticleSystem/particle/ParticlesPool.h>
#include <vsgParticleSystem/utils/additionalFuncs.h>

namespace vsgps {
Emitter::Emitter(BuilderProps const &props) : props{props} {}

void Emitter::setDynamicFunction(DynamicFunction const &dynFun) {
    dynamicFunction = dynFun;
}

void Emitter::addTail(size_t idx, Particle const &p) {
    double radius = p.getRadius();
    vsg::dvec3 pos = p.getPosition();
    if (vsg::length(lastPositions[idx] - pos) > radius * 2 * nDiameters) {
        SimpleParticle tail(props, pos, coloringLinear(pos.z, .25), radius * .4,
                            8);
        tail.addToScene(props.root);
        lastPositions[idx] = pos;
    }
}

void Emitter::tick() {
    for (auto &p : particles)
        p.tick();
}

void Emitter::moveParticles() {
    int idx;
    State dxi(states[0].size());
    for (auto &p : particles) {
        p.move();
        idx = &p - &particles[0];

        addTail(idx, p);

        dxi.setZero();
        dynamicFunction(states[idx], dxi);
        states[idx] += dxi * p.getDeltaTime();
        p.setVelocity(vsg::dvec3{dxi[0], dxi[1], dxi[2]});
    }
}

void Emitter::setNumberOfDiameters(double n) { nDiameters = n; }
void Emitter::increaseFactor(double delta) {
    for (auto &p : particles)
        p.changeFactor(delta);
}
void Emitter::decreaseFactor(double delta) {
    for (auto &p : particles)
        p.changeFactor(delta);
}

void Emitter::populate(size_t nParticles, ParticleInitializer<vsg::dvec3> &pi,
                       double radius) {
    vsg::dvec3 pos, vel;
    for (size_t i = 0; i < nParticles; i++) {
        pos = pi.givePosition();
        vel = pi.giveVelocity();

        Particle p = Particle(props, pos, vel, radius);
        p.addToScene(props.root);

        SimpleParticle tail(props, pos, {0, 0, 0, 1}, radius * .6, 8);
        tail.addToScene(props.root);

        particles.push_back(p);
        lastPositions.push_back(pos);
    }
}

void Emitter::setStates(std::array<int, 3> const &velIdxs, size_t dim) {
    vsg::dvec3 pos, vel;
    int idx = 0;
    for (auto const &p : particles) {
        states.push_back(State(dim));
        states[idx].setZero();
        pos = p.getPosition();
        vel = p.getVelocity();
        for (size_t i = 0; i < dim; i++) {
            if (i < 3) {
                states[idx][i] = pos[i];
            } else if (i < 6) {
                if (velIdxs[i - 3] != -1) {
                    states[idx][i] = vel[velIdxs[i - 3] - 1];
                } else {
                    states[idx][i] = 0;
                }
            } else {
                states[idx][i] = 0;
            }
        }
        idx++;
    }
}

double Emitter::random(double a, double b) {
    return a + (b - a) * (double(std::rand()) / double(RAND_MAX));
}

double Emitter::random(SpatialRange range) {
    return random(range[0], range[1]);
}
} // namespace vsgps

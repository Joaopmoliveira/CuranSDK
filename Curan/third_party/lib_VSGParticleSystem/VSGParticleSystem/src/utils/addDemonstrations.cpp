#include <nlohmann/json.hpp>
#include <vsgParticleSystem/dynamics/State.h>
#include <vsgParticleSystem/particle/Particle.h>
#include <vsgParticleSystem/utils/addDemonstrations.h>

namespace vsgps {
void addData(std::string filename, BuilderProps const &props,
             std::vector<vsg::dvec3> &data, double radius) {
    auto info = nlohmann::json::parse(std::ifstream(filename))["data"];
    vsg::dvec3 pos;
    for (auto const &col : info) {
        for (size_t i = 0; i < 3; i++)
            pos[i] = col[i];

        SimpleParticle sp(props, pos, {.7, .7, .7, 1}, radius, 6);
        sp.addToScene(props.root);
        data.push_back(pos);
    }
}

void addDataWithTransform(std::string filename, BuilderProps const &props,
                          std::vector<vsg::dvec3> &data,
                          DataTransform const &dt, double radius) {
    auto info = nlohmann::json::parse(std::ifstream(filename))["data"];
    vsg::dvec3 pos;
    State state(3);
    state.setZero();
    for (auto const &col : info) {
        for (size_t i = 0; i < 3; i++)
            state(i) = col[i];

        dt(state, pos);

        SimpleParticle sp(props, pos, {.7, .7, .7, 1}, radius, 6);
        sp.addToScene(props.root);
        data.push_back(pos);
    }
}

void addData(std::string filename, std::vector<vsg::dvec3> &data,
             double radius) {
    auto info = nlohmann::json::parse(std::ifstream(filename))["data"];
    vsg::dvec3 pos;
    for (auto const &col : info) {
        for (size_t i = 0; i < 3; i++)
            pos[i] = col[i];

        data.push_back(pos);
    }
}

double computeRadius(std::vector<vsg::dvec3> const &data,
                     vsg::dvec3 const &center) {
    double radius = 0, tempRadius;
    vsg::dvec3 r;
    for (auto const &d : data) {
        r = d - center;
        // r.z = 0;
        tempRadius = vsg::length(r);
        if (tempRadius > radius)
            radius = tempRadius;
    }
    return radius;
}
} // namespace vsgps
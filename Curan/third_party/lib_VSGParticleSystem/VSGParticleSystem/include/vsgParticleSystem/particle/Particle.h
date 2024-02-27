#pragma once

#include "../builder/BuilderProps.h"
#include "../builder/ResolutionBuilder.h"
#include <iostream>
#include <vsg/all.h>

namespace vsgps {
class SimpleParticle {
  public:
    SimpleParticle(BuilderProps props, vsg::dvec3 const &pos,
                   vsg::dvec4 const &color, double radius = .4,
                   size_t resolution = 4);

    virtual void move(vsg::dvec3 const &newPos);
    virtual void move(vsg::vec3 const &newPos);
    virtual vsg::dvec3 const getPosition() const;

    void addToScene(vsg::ref_ptr<vsg::Group> const &scene) const;
    void addToScene(vsg::ref_ptr<vsg::Group> &scene, size_t idx) const;
    void updateColor(vsg::dvec4 const &newColor);
    void updateColor(vsg::vec4 const &newColor);
    void setRadius(double r);

    double const getRadius() const;

  protected:
    vsg::ref_ptr<vsg::Group> root;
    vsg::ref_ptr<vsg::MatrixTransform> transform, scalling;
    vsg::ref_ptr<vsg::vec4Array> color;
    double radius = .1;

    void defineGeometry(vsg::ref_ptr<vsg::MatrixTransform> &parent,
                        vsg::ref_ptr<vsg::ResolutionBuilder> const &builder,
                        vsg::GeometryInfo const &geomInfo,
                        vsg::StateInfo const &stateInfo, size_t res = 10);
};

class CylinderParticle {
  public:
    CylinderParticle(BuilderProps props, vsg::dvec3 const &pos,
                     vsg::dvec4 const &initColor, double radius = .4,
                     size_t resolution = 4);

    void move(vsg::dvec3 const &newPos);
    void move(vsg::vec3 const &newPos);
    vsg::dvec3 const getPosition() const;

    void addToScene(vsg::ref_ptr<vsg::Group> const &scene) const;
    void addToScene(vsg::ref_ptr<vsg::Group> &scene, size_t idx) const;
    void updateColor(vsg::dvec4 const &newColor);
    void updateColor(vsg::vec4 const &newColor);
    void setRadius(double r);

    double const getRadius() const;

  protected:
    vsg::ref_ptr<vsg::Group> root;
    vsg::ref_ptr<vsg::MatrixTransform> transform, scalling;
    vsg::ref_ptr<vsg::vec4Array> color;
    double radius = .1;

  protected:
    void defineGeometry(vsg::ref_ptr<vsg::MatrixTransform> &parent,
                        vsg::ref_ptr<vsg::ResolutionBuilder> const &builder,
                        vsg::GeometryInfo const &geomInfo,
                        vsg::StateInfo const &stateInfo, size_t res = 10);
};

class Particle : public SimpleParticle {
  public:
    Particle(BuilderProps props, vsg::dvec3 const &position = {0.0, 0.0, 0.0},
             vsg::dvec3 const &velocity = {0.0, 0.0, 0.0}, double radius = .4,
             vsg::dvec4 const &initColor = {1, 0, 0, 1}, size_t resolution = 9);

    void move(vsg::dvec3 const &newVel);
    void move(vsg::vec3 const &newVel);
    inline void move() { move(velocity); }
    inline void reverse() { move(-velocity); }
    void changeFactor(double delta);

    friend std::ostream &operator<<(std::ostream &os, Particle const &p);

    virtual vsg::dvec3 const getPosition() const;
    vsg::dvec3 const getVelocity() const;
    void setVelocity(vsg::dvec3 vel);
    double const getDeltaTime() const;
    void tick();

  private:
    vsg::dvec3 position;
    vsg::dvec3 velocity;
    std::chrono::steady_clock::time_point previousCheck;
    double factor = .1;
    double dt;
};
} // namespace vsgps
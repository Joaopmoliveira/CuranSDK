#pragma once

#include "../utils/vsgAdditionalOperators.h"
#include <iostream>
#include <vsg/all.h>

namespace vsgps {
using SpatialRange = std::array<double, 2>;

template <typename D> class ParticleInitializer {
  public:
    virtual D givePosition() = 0;
    virtual D giveVelocity() = 0;

  protected:
    /**
     * Gives a random number ranging from [a, b)
     */
    double random(double a = 0, double b = 1) {
        return a + (b - a) * (double(std::rand()) / double(RAND_MAX));
    }

    double random(SpatialRange range) { return random(range[0], range[1]); }
};

class RandomParticleInitializer : public ParticleInitializer<vsg::dvec3> {
  public:
    constexpr RandomParticleInitializer()
        : rangeX{SpatialRange{-2, 2}}, rangeY{SpatialRange{-2, 2}},
          rangeZ{SpatialRange{-2, 2}}, rangeDX{SpatialRange{-2, 2}},
          rangeDY{SpatialRange{-2, 2}}, rangeDZ{SpatialRange{-2, 2}} {}
    constexpr RandomParticleInitializer(SpatialRange const &rX,
                                        SpatialRange const &rY,
                                        SpatialRange const &rZ)
        : rangeX{rX}, rangeY{rY}, rangeZ{rZ}, rangeDX{SpatialRange{-2, 2}},
          rangeDY{SpatialRange{-2, 2}}, rangeDZ{SpatialRange{-2, 2}} {}

    constexpr RandomParticleInitializer(SpatialRange const &rX,
                                        SpatialRange const &rY,
                                        SpatialRange const &rZ,
                                        SpatialRange const &rDX,
                                        SpatialRange const &rDY,
                                        SpatialRange const &rDZ)
        : rangeX{rX}, rangeY{rY}, rangeZ{rZ}, rangeDX{rDX}, rangeDY{rDY},
          rangeDZ{rDZ} {}

    virtual vsg::dvec3 givePosition();

    virtual vsg::dvec3 giveVelocity();

    inline void setRangeX(SpatialRange const &rX) { rangeX = rX; }
    inline void setRangeY(SpatialRange const &rY) { rangeY = rY; }
    inline void setRangeZ(SpatialRange const &rZ) { rangeZ = rZ; }
    inline void setRangeDX(SpatialRange const &rDX) { rangeDX = rDX; }
    inline void setRangeDY(SpatialRange const &rDY) { rangeDY = rDY; }
    inline void setRangeDZ(SpatialRange const &rDZ) { rangeDZ = rDZ; }

  private:
    SpatialRange rangeX = {-2, 2}, rangeY = {-2, 2}, rangeZ = {-2, 2};
    SpatialRange rangeDX = {-4, 4}, rangeDY = {-4, 4}, rangeDZ = {-4, 4};
};

class CircleParticleInitializer : public ParticleInitializer<vsg::dvec3> {
  public:
    constexpr CircleParticleInitializer()
        : radius{1}, center{0, 0, 0}, dx{0, 1}, dy{0, 1}, dz{0, 1} {}

    constexpr CircleParticleInitializer(double r, vsg::dvec3 const &c)
        : radius{r}, center{c}, dx{0, 1}, dy{0, 1}, dz{0, 1} {}

    constexpr CircleParticleInitializer(SpatialRange const &dx,
                                        SpatialRange const &dy,
                                        SpatialRange const &dz)
        : radius{1}, center{0, 0, 0}, dx{dx}, dy{dy}, dz{dz} {}

    constexpr CircleParticleInitializer(double r, vsg::dvec3 const &c,
                                        SpatialRange const &dx,
                                        SpatialRange const &dy,
                                        SpatialRange const &dz)
        : radius{r}, center{c}, dx{dx}, dy{dy}, dz{dz} {}

    inline void enableVerticalSpace(bool isEnable) {
        verticalSpaceEnabled = isEnable;
    }

    inline void setRadius(double r) { radius = r; }

    inline void setCenter(vsg::dvec3 const &c) { center = c; }

    void setVelocityRanges(SpatialRange const &dx, SpatialRange const &dy,
                           SpatialRange const &dz);

    inline void setVelocityRangeX(SpatialRange const &dx) { this->dx = dx; }
    inline void setVelocityRangeY(SpatialRange const &dy) { this->dy = dy; }
    inline void setVelocityRangeZ(SpatialRange const &dz) { this->dz = dz; }

    virtual vsg::dvec3 givePosition();
    virtual vsg::dvec3 giveVelocity();

  private:
    double radius, theta = 0;
    vsg::dvec3 center;
    SpatialRange dx, dy, dz;
    bool verticalSpaceEnabled = true;
};

class ShapeParticleInitializer : public ParticleInitializer<vsg::dvec3> {
  public:
    ShapeParticleInitializer(double rho = 1, size_t sides = 10, vsg::dvec3 c = {0,0,0});
    virtual vsg::dvec3 givePosition();
    virtual vsg::dvec3 giveVelocity();

    void setElevation(double e);
    double getElevation() const;
    void setStep(double dt);
    double const &getStep() const;

  protected:
    int currentSidePosition = 0, currentSideVelocity = 0;
    double radiansStep, radius;
    vsg::dvec3 center;
};

} // namespace vsgps
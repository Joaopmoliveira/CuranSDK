#ifndef POLYHEADER_HEADER_H
#define POLYHEADER_HEADER_H

#include <Mathematics/IntrConvexMesh3Plane3.h>

namespace curan{
namespace geometry{

struct PolyHeadra{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    template<typename T>
    PolyHeadra(T&& in_geometry) : geometry{in_geometry}{

    }
};

struct Cube{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Cube(double xExtent, double yExtent, double zExtent);
};

struct OpenCylinder{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    OpenCylinder(uint32_t numAxisSamples, uint32_t numRadialSamples, float radius, float height);
};

struct ClosedCylinder{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    ClosedCylinder(uint32_t numAxisSamples, uint32_t numRadialSamples, float radius, float height);
};

struct Sphere{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Sphere(uint32_t numZSamples, uint32_t numRadialSamples, float radius);
};

struct Torus{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Torus(uint32_t numCircleSamples, uint32_t numRadialSamples, float outerRadius, float innerRadius);
};

struct Tetrahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Tetrahedron();
};

struct Hexahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Hexahedron();
};

struct Octahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Octahedron();
};

struct Dodecahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Dodecahedron();
};

struct Icosahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Icosahedron();
};

}
}

#endif
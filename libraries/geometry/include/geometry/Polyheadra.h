#ifndef POLYHEADER_HEADER_H
#define POLYHEADER_HEADER_H

#include <Mathematics/IntrConvexMesh3Plane3.h>

namespace curan{
namespace geometry{

class Cube{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Cube(double xExtent, double yExtent, double zExtent);
};

class OpenCylinder{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    OpenCylinder(uint32_t numAxisSamples, uint32_t numRadialSamples, float radius, float height);
};

class ClosedCylinder{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    ClosedCylinder(uint32_t numAxisSamples, uint32_t numRadialSamples, float radius, float height);
};

class Sphere{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Sphere(uint32_t numZSamples, uint32_t numRadialSamples, float radius);
};

class Torus{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Torus(uint32_t numCircleSamples, uint32_t numRadialSamples, float outerRadius, float innerRadius);
};

class Tetrahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Tetrahedron();
};

class Hexahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Hexahedron();
};

class Octahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Octahedron();
};

class Dodecahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Dodecahedron();
};

class Icosahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Icosahedron();
};

class reference_helper_converter{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational>& geometry;

    template<typename T>
    reference_helper_converter(T&& in) : geometry{in.geometry} {

    }
};

class reference_helper_mover{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    template<typename T>
    reference_helper_mover(T&& in) : geometry{std::move(in.geometry)} {

    }
};

}
}

#endif
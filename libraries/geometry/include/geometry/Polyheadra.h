#ifndef POLYHEADER_HEADER_H
#define POLYHEADER_HEADER_H

#include <Mathematics/IntrConvexMesh3Plane3.h>
#include <Eigen/Dense>

namespace curan{
namespace geometry{

struct PolyHeadra{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    template<typename T>
    PolyHeadra(T&& in_geometry) : geometry{in_geometry.geometry}{

    }
};

struct Cube{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Cube(double xExtent, double yExtent, double zExtent);

    friend std::ostream& operator<< (std::ostream& os, const Cube& cube){
        os << "V = [";
        for(size_t i = 0; i< cube.geometry.vertices.size(); ++i)
            os << (double)cube.geometry.vertices[i][0] << " " <<  (double)cube.geometry.vertices[i][1] << " " << (double)cube.geometry.vertices[i][2] << ";\n";
        os << "];";
        return os;
    }

    void transform(const Eigen::Matrix<double,4,4>& transf);
};




struct OpenCylinder{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    OpenCylinder(uint32_t numAxisSamples, uint32_t numRadialSamples, float radius, float height);

    friend std::ostream& operator<< (std::ostream& os, const OpenCylinder& cube){
        os << "V = [";
        for(size_t i = 0; i< cube.geometry.vertices.size(); ++i)
            os << (double)cube.geometry.vertices[i][0] << " " <<  (double)cube.geometry.vertices[i][1] << " " << (double)cube.geometry.vertices[i][2] << ";\n";
        os << "];";

        os << "I = [";
        for(size_t i = 0; i< cube.geometry.triangles.size(); ++i)
            os << (double)cube.geometry.triangles[i][0] << " " <<  (double)cube.geometry.triangles[i][1] << " " << (double)cube.geometry.triangles[i][2] << ";\n";
        os << "];";
        return os;
    }
};

/*

struct ClosedCylinder{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    ClosedCylinder(uint32_t numAxisSamples, uint32_t numRadialSamples, float radius, float height);

    friend std::ostream& operator<< (std::ostream& os, const ClosedCylinder& cube){
        os << "V = [";
        for(size_t i = 0; i< cube.geometry.vertices.size(); ++i)
            os << (double)cube.geometry.vertices[i][0] << " " <<  (double)cube.geometry.vertices[i][1] << " " << (double)cube.geometry.vertices[i][2] << ";\n";
        os << "];";
        return os;
    }
};

struct Sphere{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Sphere(uint32_t numZSamples, uint32_t numRadialSamples, float radius);

    friend std::ostream& operator<< (std::ostream& os, const Sphere& cube){
        os << "V = [";
        for(size_t i = 0; i< cube.geometry.vertices.size(); ++i)
            os << (double)cube.geometry.vertices[i][0] << " " <<  (double)cube.geometry.vertices[i][1] << " " << (double)cube.geometry.vertices[i][2] << ";\n";
        os << "];";
        return os;
    }
};

struct Torus{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Torus(uint32_t numCircleSamples, uint32_t numRadialSamples, float outerRadius, float innerRadius);

    friend std::ostream& operator<< (std::ostream& os, const Torus& cube){
        os << "V = [";
        for(size_t i = 0; i< cube.geometry.vertices.size(); ++i)
            os << (double)cube.geometry.vertices[i][0] << " " <<  (double)cube.geometry.vertices[i][1] << " " << (double)cube.geometry.vertices[i][2] << ";\n";
        os << "];";
        return os;
    }
};

*/

struct Tetrahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Tetrahedron();

    friend std::ostream& operator<< (std::ostream& os, const Tetrahedron& cube){
        os << "V = [";
        for(size_t i = 0; i< cube.geometry.vertices.size(); ++i)
            os << (double)cube.geometry.vertices[i][0] << " " <<  (double)cube.geometry.vertices[i][1] << " " << (double)cube.geometry.vertices[i][2] << ";\n";
        os << "];";
        return os;
    }

    void transform(const Eigen::Matrix<double,4,4>& transf);
};

struct Hexahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Hexahedron();

    friend std::ostream& operator<< (std::ostream& os, const Hexahedron& cube){
        os << "V = [";
        for(size_t i = 0; i< cube.geometry.vertices.size(); ++i)
            os << (double)cube.geometry.vertices[i][0] << " " <<  (double)cube.geometry.vertices[i][1] << " " << (double)cube.geometry.vertices[i][2] << ";\n";
        os << "];";
        return os;
    }

    void transform(const Eigen::Matrix<double,4,4>& transf);
};

struct Octahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Octahedron();

    friend std::ostream& operator<< (std::ostream& os, const Octahedron& cube){
        os << "V = [";
        for(size_t i = 0; i< cube.geometry.vertices.size(); ++i)
            os << (double)cube.geometry.vertices[i][0] << " " <<  (double)cube.geometry.vertices[i][1] << " " << (double)cube.geometry.vertices[i][2] << ";\n";
        os << "];";
        return os;
    }

    void transform(const Eigen::Matrix<double,4,4>& transf);
};

struct Dodecahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Dodecahedron();

    friend std::ostream& operator<< (std::ostream& os, const Dodecahedron& cube){
        os << "V = [";
        for(size_t i = 0; i< cube.geometry.vertices.size(); ++i)
            os << (double)cube.geometry.vertices[i][0] << " " <<  (double)cube.geometry.vertices[i][1] << " " << (double)cube.geometry.vertices[i][2] << ";\n";
        os << "];";
        return os;
    }

    void transform(const Eigen::Matrix<double,4,4>& transf);
};

struct Icosahedron{
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    gte::ConvexMesh3<Rational> geometry;

    Icosahedron();

    friend std::ostream& operator<< (std::ostream& os, const Icosahedron& cube){
        os << "V = [";
        for(size_t i = 0; i< cube.geometry.vertices.size(); ++i)
            os << (double)cube.geometry.vertices[i][0] << " " <<  (double)cube.geometry.vertices[i][1] << " " << (double)cube.geometry.vertices[i][2] << ";\n";
        os << "];";
        return os;
    }

    void transform(const Eigen::Matrix<double,4,4>& transf);
};

}
}

#endif
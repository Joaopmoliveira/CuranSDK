#include <cstdint>
#include <unordered_set>
#include <stdexcept>
#include <array>
#include <vector>

#include <map>

#include <Mathematics/DistPointHyperplane.h>
#include <Mathematics/Vector2.h>
#include <Mathematics/Logger.h>
#include <Mathematics/EdgeKey.h>
#include <Mathematics/TriangleKey.h>
#include <Mathematics/IntrConvexMesh3Plane3.h>


#include <iostream>

#include <fstream>


using Rational = gte::BSRational<gte::UIntegerAP32>;

struct Factory
{

    bool mOutside = true;

    // The box has center (0,0,0); unit-length axes (1,0,0), (0,1,0), and
    // (0,0,1); and extents (half-lengths) xExtent, yExtent, and zExtent.
    // The mesh has 8 vertices and 12 triangles.  For example, the box
    // corner in the first octant is (xExtent, yExtent, zExtent).
    template <typename Numeric>
    gte::ConvexMesh3<Numeric> CreateBox(double xExtent, double yExtent, double zExtent)
    {
        gte::ConvexMesh3<Numeric> geometry;

        // Quantities derived from inputs.
        int32_t numVertices = 8;
        int32_t numTriangles = 12;

        // Generate geometry.
        geometry.vertices.resize(numVertices);

        std::vector<gte::Vector3<double>> vertices;
        vertices.resize(numVertices);
        gte::Vector3<double> pos{}, nor{};
        std::array<gte::Vector3<double>, 3> basis{};
        gte::Vector2<double> tcd{};

        // Choose vertex normals in the diagonal directions.
        gte::Vector3<double> diag{xExtent, yExtent, zExtent};
        Normalize(diag);

        for (uint32_t z = 0, v = 0; z < 2; ++z)
        {
            double fz = static_cast<double>(z), omfz = 1.0f - fz;
            double zSign = 2.0f * fz - 1.0f;
            pos[2] = zSign * zExtent;
            nor[2] = zSign * diag[2];
            for (uint32_t y = 0; y < 2; ++y)
            {
                double fy = static_cast<double>(y);
                double ySign = 2.0f * fy - 1.0f;
                pos[1] = ySign * yExtent;
                nor[1] = ySign * diag[1];
                tcd[1] = (1.0f - fy) * omfz + (0.75f - 0.5f * fy) * fz;
                for (uint32_t x = 0; x < 2; ++x, ++v)
                {
                    double fx = static_cast<double>(x);
                    double xSign = 2.0f * fx - 1.0f;
                    pos[0] = xSign * xExtent;
                    nor[0] = xSign * diag[0];
                    tcd[0] = fx * omfz + (0.25f + 0.5f * fx) * fz;

                    basis[0] = nor;
                    ComputeOrthogonalComplement(1, basis.data());
                    vertices[v] = pos;
                }
            }
        }

        for (size_t i = 0; i < numVertices; ++i)
            geometry.vertices[i] = {{Rational{vertices[i][0]}, Rational{vertices[i][1]}, Rational{vertices[i][2]}}};

        // Generate indices (outside view).
        geometry.triangles.resize(numTriangles);
        geometry.triangles[0] = {0, 2, 3};
        geometry.triangles[1] = {0, 3, 1};
        geometry.triangles[2] = {0, 1, 5};
        geometry.triangles[3] = {0, 5, 4};
        geometry.triangles[4] = {0, 4, 6};
        geometry.triangles[5] = {0, 6, 2};
        geometry.triangles[6] = {7, 6, 4};
        geometry.triangles[7] = {7, 4, 5};
        geometry.triangles[8] = {7, 5, 1};
        geometry.triangles[9] = {7, 1, 3};
        geometry.triangles[10] = {7, 3, 2};
        geometry.triangles[11] = {7, 2, 6};
        return geometry;
    }

    // The cylinder has center (0,0,0), the specified radius, and the
    // specified height.  The cylinder axis is a line segment of the form
    // (0,0,0) + t*(0,0,1) for |t| <= height/2.  The cylinder wall is
    // implicitly defined by x^2+y^2 = radius^2.  CreateCylinderOpen leads
    // to a cylinder whose end-disks are omitted; you have an open tube.
    // CreateCylinderClosed leads to a cylinder with end-disks.  Each
    // end-disk is a regular polygon that is tessellated by including a
    // vertex at the center of the polygon and decomposing the polygon
    // into triangles that all share the center vertex and each triangle
    // containing an edge of the polygon.
    template <typename Numeric>
    gte::ConvexMesh3<Numeric> CreateCylinderOpen(uint32_t numAxisSamples, uint32_t numRadialSamples, float radius, float height)
    {
        gte::ConvexMesh3<Numeric> geometry;

        // Quantities derived from inputs.
        uint32_t numVertices = numAxisSamples * (numRadialSamples + 1);
        uint32_t numTriangles = 2 * (numAxisSamples - 1) * numRadialSamples;
        float invRS = 1.0f / static_cast<float>(numRadialSamples);
        float invASm1 = 1.0f / static_cast<float>(numAxisSamples - 1);
        float halfHeight = 0.5f * height;

        // Generate geometry.
        geometry.vertices.resize(numVertices);

        std::vector<gte::Vector3<float>> vertices;
        vertices.resize(numVertices);

        gte::Vector3<float> pos{}, nor{};
        std::array<gte::Vector3<float>, 3> basis{};
        gte::Vector2<float> tcd{};

        // Generate points on the unit circle to be used in computing the mesh
        // points on a cylinder slice.
        std::vector<float> cs(static_cast<size_t>(numRadialSamples) + 1);
        std::vector<float> sn(static_cast<size_t>(numRadialSamples) + 1);
        for (uint32_t r = 0; r < numRadialSamples; ++r)
        {
            float angle = invRS * r * static_cast<float>(GTE_C_TWO_PI);
            cs[r] = std::cos(angle);
            sn[r] = std::sin(angle);
        }
        cs[numRadialSamples] = cs[0];
        sn[numRadialSamples] = sn[0];

        // Generate the cylinder itself.
        for (uint32_t a = 0, i = 0; a < numAxisSamples; ++a)
        {
            float axisFraction = a * invASm1; // in [0,1]
            float z = -halfHeight + height * axisFraction;

            // Compute center of slice.
            gte::Vector3<float> sliceCenter{0.0f, 0.0f, z};

            // Compute slice vertices with duplication at endpoint.
            for (uint32_t r = 0; r <= numRadialSamples; ++r, ++i)
            {
                float radialFraction = r * invRS; // in [0,1)
                nor = {cs[r], sn[r], 0.0f};
                pos = sliceCenter + radius * nor;
                if (!mOutside)
                {
                    nor = -nor;
                }

                basis[0] = nor;
                ComputeOrthogonalComplement(1, basis.data());
                tcd = {radialFraction, axisFraction};
                vertices[i] = pos;
            }
        }

        for (size_t i = 0; i < numVertices; ++i)
            geometry.vertices[i] = {{Numeric{vertices[i][0]}, Numeric{vertices[i][1]}, Numeric{vertices[i][2]}}};

        geometry.triangles.resize(numTriangles);
        for (uint32_t a = 0, aStart = 0, t = 0; a < numAxisSamples - 1; ++a)
        {
            uint32_t i0 = aStart;
            uint32_t i1 = i0 + 1;
            aStart += numRadialSamples + 1;
            uint32_t i2 = aStart;
            uint32_t i3 = i2 + 1;
            for (uint32_t i = 0; i < numRadialSamples; ++i, ++i0, ++i1, ++i2, ++i3)
            {
                geometry.triangles[t++] = {(int32_t)i0, (int32_t)i1, (int32_t)i2};
                geometry.triangles[t++] = {(int32_t)i1, (int32_t)i3, (int32_t)i2};
            }
        }
        return geometry;
    }

    template <typename Numeric>
    gte::ConvexMesh3<Numeric> CreateCylinderClosed(uint32_t numAxisSamples, uint32_t numRadialSamples, float radius, float height)
    {
        gte::ConvexMesh3<Numeric> geometry;
        // Create a sphere and then deform it into a closed cylinder.
        gte::ConvexMesh3<float> to_deform_geometry = CreateSphere<float>(numAxisSamples, numRadialSamples, radius);

        uint32_t numVertices = to_deform_geometry.vertices.size();
        gte::Vector3<float> pos{};
        std::array<gte::Vector3<float>, 3> basis{};
        uint32_t i;

        // Flatten sphere at poles.
        float hDiv2 = 0.5f * height;
        i = numVertices - 2;
        pos = to_deform_geometry.vertices[i];
        pos[2] = -hDiv2;
        to_deform_geometry.vertices[i] = pos; // south pole
        i = numVertices - 1;
        pos = to_deform_geometry.vertices[i];
        pos[2] = +hDiv2;
        to_deform_geometry.vertices[i] = pos; // north pole

        // Remap z-values to [-h/2,h/2].
        float zFactor = 2.0f / static_cast<float>(numAxisSamples - 1);
        float tmp0 = radius * (-1.0f + zFactor);
        float tmp1 = 1.0f / (radius * (+1.0f - zFactor));
        for (i = 0; i < numVertices - 2; ++i)
        {
            pos = to_deform_geometry.vertices[i];
            pos[2] = hDiv2 * (-1.0f + tmp1 * (pos[2] - tmp0));
            float adjust = radius / std::sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
            pos[0] *= adjust;
            pos[1] *= adjust;
            to_deform_geometry.vertices[i] = pos;
        }

        geometry.triangles = to_deform_geometry.triangles;
        geometry.vertices.resize(numVertices);
        for (size_t i = 0; i < numVertices; ++i)
            geometry.vertices[i] = {{Numeric{to_deform_geometry.vertices[i][0]}, Numeric{to_deform_geometry.vertices[i][1]}, Numeric{to_deform_geometry.vertices[i][2]}}};
        return geometry;
    }

    // The sphere has center (0,0,0) and the specified radius.  The north
    // pole is at (0,0,radius) and the south pole is at (0,0,-radius).
    // The mesh has the topology of an open cylinder (which is also the
    // topology of a rectangle with wrap-around for one pair of parallel
    // edges) and is then stitched to the north and south poles.  The
    // triangles are unevenly distributed.  If you want a more even
    // distribution, create an icosahedron and subdivide it.
    template <typename Numeric>
    gte::ConvexMesh3<Numeric> CreateSphere(uint32_t numZSamples, uint32_t numRadialSamples, float radius)
    {
        gte::ConvexMesh3<Numeric> geometry;

        // Quantities derived from inputs.
        uint32_t zsm1 = numZSamples - 1;
        uint32_t zsm2 = numZSamples - 2;
        uint32_t zsm3 = numZSamples - 3;
        uint32_t rsp1 = numRadialSamples + 1;
        float invRS = 1.0f / static_cast<float>(numRadialSamples);
        float zFactor = 2.0f / static_cast<float>(zsm1);
        uint32_t numVertices = zsm2 * rsp1 + 2;
        uint32_t numTriangles = 2 * zsm2 * numRadialSamples;

        // Generate geometry.
        std::vector<gte::Vector3<float>> vertices;
        vertices.resize(numVertices);
        geometry.vertices.resize(numVertices);
        geometry.triangles.resize(numTriangles);

        gte::Vector3<float> pos{}, nor{};
        std::array<gte::Vector3<float>, 3> basis{};
        gte::Vector2<float> tcd{};

        // Generate points on the unit circle to be used in computing the mesh
        // points on a sphere slice.
        std::vector<float> cs(rsp1), sn(rsp1);
        for (uint32_t r = 0; r < numRadialSamples; ++r)
        {
            float angle = invRS * r * static_cast<float>(GTE_C_TWO_PI);
            cs[r] = std::cos(angle);
            sn[r] = std::sin(angle);
        }
        cs[numRadialSamples] = cs[0];
        sn[numRadialSamples] = sn[0];

        // Generate the sphere itself.
        uint32_t i = 0;
        for (uint32_t z = 1; z < zsm1; ++z)
        {
            float zFraction = -1.0f + zFactor * static_cast<float>(z); // in (-1,1)
            float zValue = radius * zFraction;

            // Compute center of slice.
            gte::Vector3<float> sliceCenter{0.0f, 0.0f, zValue};

            // Compute radius of slice.
            float sliceRadius = std::sqrt(std::max(radius * radius - zValue * zValue, 0.0f));

            // Compute slice vertices with duplication at endpoint.
            for (uint32_t r = 0; r <= numRadialSamples; ++r, ++i)
            {
                float radialFraction = r * invRS; // in [0,1)
                gte::Vector3<float> radial{cs[r], sn[r], 0.0f};
                pos = sliceCenter + sliceRadius * radial;
                nor = pos;
                Normalize(nor);
                if (!mOutside)
                {
                    nor = -nor;
                }

                basis[0] = nor;
                ComputeOrthogonalComplement(1, basis.data());
                tcd = {radialFraction, 0.5f * (zFraction + 1.0f)};
                vertices[i] = pos;
            }
        }

        // The point at the south pole.
        pos = {0.0f, 0.0f, -radius};
        if (mOutside)
        {
            nor = {0.0f, 0.0f, -1.0f};
        }
        else
        {
            nor = {0.0f, 0.0f, 1.0f};
        }
        basis[0] = nor;
        ComputeOrthogonalComplement(1, basis.data());
        tcd = {0.5f, 0.0f};
        vertices[i] = pos;
        ++i;

        // The point at the north pole.
        pos = {0.0f, 0.0f, radius};
        if (mOutside)
        {
            nor = {0.0f, 0.0f, 1.0f};
        }
        else
        {
            nor = {0.0f, 0.0f, -1.0f};
        }
        basis[0] = nor;
        ComputeOrthogonalComplement(1, basis.data());
        tcd = {0.5f, 1.0f};
        vertices[i] = pos;

        uint32_t t = 0;
        for (uint32_t z = 0, zStart = 0; z < zsm3; ++z)
        {
            uint32_t i0 = zStart;
            uint32_t i1 = i0 + 1;
            zStart += rsp1;
            uint32_t i2 = zStart;
            uint32_t i3 = i2 + 1;
            for (i = 0; i < numRadialSamples; ++i, ++i0, ++i1, ++i2, ++i3)
            {
                geometry.triangles[t++] = {(int32_t)i0, (int32_t)i1, (int32_t)i2};
                geometry.triangles[t++] = {(int32_t)i1, (int32_t)i3, (int32_t)i2};
            }
        }

        for (size_t i = 0; i < numVertices; ++i)
            geometry.vertices[i] = {{Numeric{vertices[i][0]}, Numeric{vertices[i][1]}, Numeric{vertices[i][2]}}};

        // The south pole triangles (outside view).
        uint32_t numVerticesM2 = numVertices - 2;
        for (i = 0; i < numRadialSamples; ++i, ++t)
            geometry.triangles[t] = {(int32_t)i, (int32_t)numVerticesM2, (int32_t)i + 1};

        // The north pole triangles (outside view).
        uint32_t numVerticesM1 = numVertices - 1, offset = zsm3 * rsp1;
        for (i = 0; i < numRadialSamples; ++i, ++t)
            geometry.triangles[t] = {(int32_t)i + (int32_t)offset, (int32_t)i + 1 + (int32_t)offset, (int32_t)numVerticesM1};
        return geometry;
    }

    // The torus has center (0,0,0).  If you observe the torus along the
    // line with direction (0,0,1), you will see an annulus.  The circle
    // that is the center of the annulus has radius 'outerRadius'.  The
    // distance from this circle to the boundaries of the annulus is the
    // 'inner radius'.
    template <typename Numeric>
    gte::ConvexMesh3<Numeric> CreateTorus(uint32_t numCircleSamples, uint32_t numRadialSamples, float outerRadius, float innerRadius)
    {
        gte::ConvexMesh3<Numeric> geometry;
        // Quantities derived from inputs.
        float invCS = 1.0f / static_cast<float>(numCircleSamples);
        float invRS = 1.0f / static_cast<float>(numRadialSamples);
        uint32_t numVertices = (numCircleSamples + 1) * (numRadialSamples + 1);
        uint32_t numTriangles = 2 * numCircleSamples * numRadialSamples;

        // Generate geometry.
        // Generate geometry.
        std::vector<gte::Vector3<float>> vertices;
        vertices.resize(numVertices);
        geometry.vertices.resize(numVertices);

        gte::Vector3<float> pos{}, nor{};
        std::array<gte::Vector3<float>, 3> basis{};
        gte::Vector2<float> tcd{};

        // Generate an open cylinder that is warped into a torus.
        uint32_t i = 0;
        for (uint32_t c = 0; c < numCircleSamples; ++c)
        {
            // Compute center point on torus circle at specified angle.
            float circleFraction = static_cast<float>(c) * invCS; // in [0,1)
            float theta = circleFraction * static_cast<float>(GTE_C_TWO_PI);
            float cosTheta = std::cos(theta);
            float sinTheta = std::sin(theta);
            gte::Vector3<float> radial{cosTheta, sinTheta, 0.0f};
            gte::Vector3<float> torusMiddle = outerRadius * radial;

            // Compute slice vertices with duplication at endpoint.
            for (uint32_t r = 0; r <= numRadialSamples; ++r, ++i)
            {
                float radialFraction = static_cast<float>(r) * invRS; // in [0,1)
                float phi = radialFraction * static_cast<float>(GTE_C_TWO_PI);
                float cosPhi = std::cos(phi);
                float sinPhi = std::sin(phi);
                nor = cosPhi * radial + sinPhi * gte::Vector3<float>::Unit(2);
                pos = torusMiddle + innerRadius * nor;
                if (!mOutside)
                {
                    nor = -nor;
                }

                basis[0] = nor;
                ComputeOrthogonalComplement(1, basis.data());
                tcd = {radialFraction, circleFraction};
                vertices[i] = pos;
            }
        }

        // Duplicate the cylinder ends to form a torus.
        for (uint32_t r = 0; r <= numRadialSamples; ++r, ++i)
            vertices[i] = vertices[r];

        for (size_t i = 0; i < numVertices; ++i)
            geometry.vertices[i] = {{Numeric{vertices[i][0]}, Numeric{vertices[i][1]}, Numeric{vertices[i][2]}}};

        // Generate indices (outside view).
        geometry.triangles.resize(numTriangles);

        uint32_t t = 0;
        for (uint32_t c = 0, cStart = 0; c < numCircleSamples; ++c)
        {
            uint32_t i0 = cStart;
            uint32_t i1 = i0 + 1;
            cStart += numRadialSamples + 1;
            uint32_t i2 = cStart;
            uint32_t i3 = i2 + 1;
            for (i = 0; i < numRadialSamples; ++i, ++i0, ++i1, ++i2, ++i3)
            {
                geometry.triangles[t++] = {(int32_t)i0, (int32_t)i2, (int32_t)i1};
                geometry.triangles[t++] = {(int32_t)i1, (int32_t)i2, (int32_t)i3};
            }
        }
        return geometry;
    }

    // Platonic solids, all inscribed in a unit sphere centered at
    // (0,0,0).
    template <typename Numeric>
    gte::ConvexMesh3<Numeric> CreateTetrahedron()
    {
         gte::ConvexMesh3<Numeric> geometry;
        float const sqrt2Div3 = std::sqrt(2.0f) / 3.0f;
        float const sqrt6Div3 = std::sqrt(6.0f) / 3.0f;
        float const oneThird = 1.0f / 3.0f;
        uint32_t const numVertices = 4;
        uint32_t const numTriangles = 4;

        geometry.triangles.resize(numTriangles);
        geometry.vertices.resize(numVertices);

        gte::Vector3<float> pos{}, nor{};
        std::array<gte::Vector3<float>, 3> basis{};

        geometry.vertices[0] = {0.0f, 0.0f, 1.0f};
        geometry.vertices[1] = {2.0f * sqrt2Div3, 0.0f, -oneThird};
        geometry.vertices[2] = {-sqrt2Div3, sqrt6Div3, -oneThird};
        geometry.vertices[3] = {-sqrt2Div3, -sqrt6Div3, -oneThird};

        geometry.triangles[0] = {0, 1, 2};
        geometry.triangles[1] = {0, 2, 3};
        geometry.triangles[2] = {0, 3, 1};
        geometry.triangles[3] = {1, 3, 2};

        return geometry;
    }

    template <typename Numeric>
    gte::ConvexMesh3<Numeric> CreateHexahedron()
    {
         gte::ConvexMesh3<Numeric> geometry;
        float const sqrtThird = std::sqrt(1.0f / 3.0f);
        uint32_t const numVertices = 8;
        uint32_t const numTriangles = 12;

        geometry.triangles.resize(numTriangles);
        geometry.vertices.resize(numVertices);


        gte::Vector3<float> pos{}, nor{};
        std::array<gte::Vector3<float>, 3> basis{};

        geometry.vertices[0] = {-sqrtThird, -sqrtThird, -sqrtThird};
        geometry.vertices[1] = {sqrtThird, -sqrtThird, -sqrtThird};
        geometry.vertices[2] = {sqrtThird, sqrtThird, -sqrtThird};
        geometry.vertices[3] = {-sqrtThird, sqrtThird, -sqrtThird};
        geometry.vertices[4] = {-sqrtThird, -sqrtThird, sqrtThird};
        geometry.vertices[5] = {sqrtThird, -sqrtThird, sqrtThird};
        geometry.vertices[6] = {sqrtThird, sqrtThird, sqrtThird};
        geometry.vertices[7] = {-sqrtThird, sqrtThird, sqrtThird};

        geometry.triangles[0] = { 0, 3, 2};
        geometry.triangles[1] = { 0, 2, 1};
        geometry.triangles[2] = { 0, 1, 5};
        geometry.triangles[3] = { 0, 5, 4};
        geometry.triangles[4] = { 0, 4, 7};
        geometry.triangles[5] = { 0, 7, 3};
        geometry.triangles[6] = { 6, 5, 1};
        geometry.triangles[7] = { 6, 1, 2};
        geometry.triangles[8] = { 6, 2, 3};
        geometry.triangles[9] = { 6, 3, 7};
        geometry.triangles[10] = { 6, 7, 4};
        geometry.triangles[11] = { 6, 4, 5};

        return geometry;
    }

    template <typename Numeric>
    gte::ConvexMesh3<Numeric> CreateOctahedron()
    {
         gte::ConvexMesh3<Numeric> geometry;
        uint32_t const numVertices = 6;
        uint32_t const numTriangles = 8;

        geometry.triangles.resize(numTriangles);
        geometry.vertices.resize(numVertices);

        gte::Vector3<float> pos{}, nor{};
        std::array<gte::Vector3<float>, 3> basis{};

        geometry.vertices[0] = {1.0f, 0.0f, 0.0f};
        geometry.vertices[1] = {-1.0f, 0.0f, 0.0f};
        geometry.vertices[2] = {0.0f, 1.0f, 0.0f};
        geometry.vertices[3] = {0.0f, -1.0f, 0.0f};
        geometry.vertices[4] = {0.0f, 0.0f, 1.0f};
        geometry.vertices[5] = {0.0f, 0.0f, -1.0f};

        geometry.triangles[0] = { 4, 0, 2};
        geometry.triangles[1] = { 4, 2, 1};
        geometry.triangles[2] = { 4, 1, 3};
        geometry.triangles[3] = { 4, 3, 0};
        geometry.triangles[4] = { 5, 2, 0};
        geometry.triangles[5] = { 5, 1, 2};
        geometry.triangles[6] = { 5, 3, 1};
        geometry.triangles[7] = { 5, 0, 3};

        return geometry;
    }

    template <typename Numeric>
    gte::ConvexMesh3<Numeric> CreateDodecahedron()
    {
        gte::ConvexMesh3<Numeric> geometry;
        float const a = 1.0f / std::sqrt(3.0f);
        float const b = std::sqrt((3.0f - std::sqrt(5.0f)) / 6.0f);
        float const c = std::sqrt((3.0f + std::sqrt(5.0f)) / 6.0f);
        uint32_t const numVertices = 20;
        uint32_t const numTriangles = 36;

        geometry.triangles.resize(numTriangles);
        geometry.vertices.resize(numVertices);

        gte::Vector3<float> pos{}, nor{};
        std::array<gte::Vector3<float>, 3> basis{};

        geometry.vertices[0] = {a, a, a};
        geometry.vertices[1] = {a, a, -a};
        geometry.vertices[2] = {a, -a, a};
        geometry.vertices[3] = {a, -a, -a};
        geometry.vertices[4] = {-a, a, a};
        geometry.vertices[5] = {-a, a, -a};
        geometry.vertices[6] = {-a, -a, a};
        geometry.vertices[7] = {-a, -a, -a};
        geometry.vertices[8] = {b, c, 0.0f};
        geometry.vertices[9] = {-b, c, 0.0f};
        geometry.vertices[10] = {b, -c, 0.0f};
        geometry.vertices[11] = {-b, -c, 0.0f};
        geometry.vertices[12] = {c, 0.0f, b};
        geometry.vertices[13] = {c, 0.0f, -b};
        geometry.vertices[14] = {-c, 0.0f, b};
        geometry.vertices[15] = {-c, 0.0f, -b};
        geometry.vertices[16] = {0.0f, b, c};
        geometry.vertices[17] = {0.0f, -b, c};
        geometry.vertices[18] = {0.0f, b, -c};
        geometry.vertices[19] = {0.0f, -b, -c};

        geometry.triangles[0] = {0, 8, 9};
        geometry.triangles[1] = { 0, 9, 4};
        geometry.triangles[2] = { 0, 4, 16};
        geometry.triangles[3] = { 0, 12, 13};
        geometry.triangles[4] = { 0, 13, 1};
        geometry.triangles[5] = { 0, 1, 8};
        geometry.triangles[6] = { 0, 16, 17};
        geometry.triangles[7] = { 0, 17, 2};
        geometry.triangles[8] = { 0, 2, 12};
        geometry.triangles[9] = { 8, 1, 18};
        geometry.triangles[10] = { 8, 18, 5};
        geometry.triangles[11] = { 8, 5, 9};
        geometry.triangles[12] = { 12, 2, 10};
        geometry.triangles[13] = { 12, 10, 3};
        geometry.triangles[14] = { 12, 3, 13};
        geometry.triangles[15] = { 16, 4, 14};
        geometry.triangles[16] = { 16, 14, 6};
        geometry.triangles[17] = { 16, 6, 17};
        geometry.triangles[18] = { 9, 5, 15};
        geometry.triangles[19] = { 9, 15, 14};
        geometry.triangles[20] = { 9, 14, 4};
        geometry.triangles[21] = { 6, 11, 10};
        geometry.triangles[22] = { 6, 10, 2};
        geometry.triangles[23] = { 6, 2, 17};
        geometry.triangles[24] = { 3, 19, 18};
        geometry.triangles[25] = { 3, 18, 1};
        geometry.triangles[26] = { 3, 1, 13};
        geometry.triangles[27] = { 7, 15, 5};
        geometry.triangles[28] = { 7, 5, 18};
        geometry.triangles[29] = { 7, 18, 19};
        geometry.triangles[30] = { 7, 11, 6};
        geometry.triangles[31] = { 7, 6, 14};
        geometry.triangles[32] = { 7, 14, 15};
        geometry.triangles[33] = { 7, 19, 3};
        geometry.triangles[34] = { 7, 3, 10};
        geometry.triangles[35] = { 7, 10, 11};

        return geometry;
    }

    template <typename Numeric>
    gte::ConvexMesh3<Numeric> CreateIcosahedron()
    {
         gte::ConvexMesh3<Numeric> geometry;
        float const goldenRatio = 0.5f * (1.0f + std::sqrt(5.0f));
        float const invRoot = 1.0f / std::sqrt(1.0f + goldenRatio * goldenRatio);
        float const u = goldenRatio * invRoot;
        float const v = invRoot;
        uint32_t const numVertices = 12;
        uint32_t const numTriangles = 20;

        geometry.triangles.resize(numTriangles);
        geometry.vertices.resize(numVertices);

        gte::Vector3<float> pos{}, nor{};
        std::array<gte::Vector3<float>, 3> basis{};

        geometry.vertices[0] = {u, v, 0.0f};
        geometry.vertices[1] = {-u, v, 0.0f};
        geometry.vertices[2] = {u, -v, 0.0f};
        geometry.vertices[3] = {-u, -v, 0.0f};
        geometry.vertices[4] = {v, 0.0f, u};
        geometry.vertices[5] = {v, 0.0f, -u};
        geometry.vertices[6] = {-v, 0.0f, u};
        geometry.vertices[7] = {-v, 0.0f, -u};
        geometry.vertices[8] = {0.0f, u, v};
        geometry.vertices[9] = {0.0f, -u, v};
        geometry.vertices[10] = {0.0f, u, -v};
        geometry.vertices[11] = {0.0f, -u, -v};

        geometry.triangles[0] = { 0, 8, 4};
        geometry.triangles[1] = { 0, 5, 10};
        geometry.triangles[2] = { 2, 4, 9};
        geometry.triangles[3] = { 2, 11, 5};
        geometry.triangles[4] = { 1, 6, 8};
        geometry.triangles[5] = { 1, 10, 7};
        geometry.triangles[6] = { 3, 9, 6};
        geometry.triangles[7] = { 3, 7, 11};
        geometry.triangles[8] = { 0, 10, 8};
        geometry.triangles[9] = {1, 8, 10};
        geometry.triangles[10] = { 2, 9, 11};
        geometry.triangles[11] = { 3, 11, 9};
        geometry.triangles[12] = { 4, 2, 0};
        geometry.triangles[13] = { 5, 0, 2};
        geometry.triangles[14] = { 6, 1, 3};
        geometry.triangles[15] = { 7, 3, 1};
        geometry.triangles[16] = { 8, 6, 4};
        geometry.triangles[17] = { 9, 4, 6};
        geometry.triangles[18] = { 10, 5, 7};
        geometry.triangles[19] = { 11, 7, 5};

        return geometry;
    }
};

int inner_main_dual()
{
    using Query = gte::FIQuery<Rational, gte::ConvexMesh3<Rational>, gte::Plane3<Rational>>;
    Factory factory;
    gte::ConvexMesh3<Rational> mPolyhedron = factory.CreateBox<Rational>(1, 1, 1);
    gte::ConvexMesh3<Rational> mPolyhedron2 = factory.CreateCylinderOpen<Rational>(100, 100, 1, 1);
    gte::ConvexMesh3<Rational> mPolyhedron4 = factory.CreateSphere<Rational>(100, 100, 1);
    gte::ConvexMesh3<Rational> mPolyhedron3 = factory.CreateCylinderClosed<Rational>(100, 100, 1, 1);
    gte::ConvexMesh3<Rational> mPolyhedron5 = factory.CreateTorus<Rational>(100, 100, 1, 1);

    gte::ConvexMesh3<Rational> mPolyhedron6 = factory.CreateTetrahedron<Rational>();
    gte::ConvexMesh3<Rational> mPolyhedron7 = factory.CreateHexahedron<Rational>();
    gte::ConvexMesh3<Rational> mPolyhedron8 = factory.CreateOctahedron<Rational>();
    gte::ConvexMesh3<Rational> mPolyhedron9 = factory.CreateDodecahedron<Rational>();
    gte::ConvexMesh3<Rational> mPolyhedron10 = factory.CreateIcosahedron<Rational>();
          
    gte::Plane3<Rational> mPlane;
    Query quarey;
    auto mResult = quarey(mPolyhedron, mPlane, Query::REQ_ALL);

    auto const &polyVertices = mResult.intersectionPolygon;
    size_t const numPolyVertices = polyVertices.size();
    bool mValidPolygonCurve = (numPolyVertices > 0);
    if (mValidPolygonCurve)
    {
        std::cout << "V_polygon = [" << std::endl;
        for (auto &val : polyVertices)
            std::cout << " " << (double)val[0] << " " << (double)val[1] << "  " << (double)val[2] << ";\n";
        std::cout << "];\n";
    }

    auto const &imesh = mResult.intersectionMesh; 
    bool mValidPolygonMesh = (imesh.vertices.size() > 0);
    if (mValidPolygonMesh)
    {
        std::cout << "V_mesh_polygon = [" << std::endl;
        for (auto &val : imesh.vertices)
            std::cout << " " << (double)val[0] << " " << (double)val[1] << "  " << (double)val[2] << ";\n";
        std::cout << "];\n";
        uint32_t const numVertices = static_cast<uint32_t>(imesh.vertices.size());

        std::vector<int32_t> indices;
        indices.resize(mResult.intersectionMesh.triangles.size()*3);
        size_t const numBytes = imesh.triangles.size() * sizeof(std::array<int32_t, 3>);
        std::memcpy(indices.data(), imesh.triangles.data(), numBytes);

        std::cout << "Ind_mesh_polygon=[";
        for (auto &index : indices)
            std::cout << " " << index << ";\n";
        std::cout << "];\n";
    }
}

int main()
{
    try
    {
        inner_main_dual();
    }
    catch (std::runtime_error &e)
    {
        std::cout << "exception: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "unexpected exception" << std::endl;
        return 1;
    }

    return 0;
}

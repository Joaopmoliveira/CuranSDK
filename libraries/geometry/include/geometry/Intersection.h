#ifndef INTERSECTION_HEADER_H
#define INTERSECTION_HEADER_H

#include <Mathematics/IntrConvexMesh3Plane3.h>
#include <Eigen/Dense>
#include <optional>

namespace curan{
namespace geometry{

inline std::optional<Eigen::Matrix<double,3,Eigen::Dynamic>> clip_with_plane(const PolyHeadra& helper, Eigen::Matrix<double,3,1> inNormal, Eigen::Matrix<double,3,1> inOrigin){
    using Rational = gte::BSRational<gte::UIntegerAP32>;
    using Query = gte::FIQuery<Rational, gte::ConvexMesh3<Rational>, gte::Plane3<Rational>>;

    assert(inNormal.norm() > 0.999 && inNormal.norm() < 1.0001);

    gte::Plane3<Rational> mPlane{{inNormal[0],inNormal[1],inNormal[2]},{inOrigin[0],inOrigin[1],inOrigin[2]}};

    Query quarey;
    auto mResult = quarey(helper.geometry, mPlane, Query::REQ_ALL);

    auto const &polyVertices = mResult.intersectionPolygon;
    size_t const numPolyVertices = polyVertices.size();

    if(numPolyVertices<1)
        return std::nullopt;

    Eigen::Matrix<double,3,Eigen::Dynamic> polygon_vertices = Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3,mResult.intersectionPolygon.size());
    for(size_t i = 0; i < polygon_vertices.cols(); ++i ){
        polygon_vertices(0,i) = mResult.intersectionPolygon[i][0];
        polygon_vertices(1,i) = mResult.intersectionPolygon[i][1];
        polygon_vertices(2,i) = mResult.intersectionPolygon[i][2];
    }
    return polygon_vertices;

}

}
}

#endif
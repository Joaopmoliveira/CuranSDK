#ifndef INTERSECTION_HEADER_H
#define INTERSECTION_HEADER_H

#include <Mathematics/IntrConvexMesh3Plane3.h>
#include <Eigen/Dense>
#include <optional>

namespace curan{
namespace geometry{

template<typename T>
std::optional<std::vector<Eigen::Matrix<double,3,1>>> clip_with_plane(reference_helper_converter<T> helper, Eigen::Matrix<double,3,1> inNormal, Eigen::Matrix<double,3,1> inOrigin){
    using Query = gte::FIQuery<Rational, gte::ConvexMesh3<Rational>, gte::Plane3<Rational>>;

    gte::Plane3<Rational> mPlane;
    Query quarey;
    auto mResult = quarey(helper.geometry, mPlane, Query::REQ_ALL);

    auto const &polyVertices = mResult.intersectionPolygon;
    size_t const numPolyVertices = polyVertices.size();

    if(numPolyVertices<1)
        return std::nullopt;

    return mResult.intersectionPolygon;

}

}
}

#endif
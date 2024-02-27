#pragma once

#include <vsg/all.h>

inline vsg::dvec3 operator*(double scalar, vsg::dvec3 const &vector) {
    return vsg::dvec3(vector[0] * scalar, vector[1] * scalar,
                      vector[2] * scalar);
}

inline vsg::dvec3 operator*(float scalar, vsg::dvec3 const &vector) {
    return vsg::dvec3(vector[0] * scalar, vector[1] * scalar,
                      vector[2] * scalar);
}

inline vsg::dvec3 operator*(vsg::dvec3 const &vector, float scalar) {
    return scalar * vector;
}

inline vsg::dvec3 operator+(vsg::dvec3 const &vector, double scalar) {
    return vsg::dvec3(vector[0] + scalar, vector[1] + scalar,
                      vector[2] + scalar);
}

inline vsg::dvec3 operator+(double scalar, vsg::dvec3 const &vector) {
    return vector + scalar;
}

inline vsg::dvec3 operator-(vsg::dvec3 const &vector, double scalar) {
    return vector + (-scalar);
}

inline vsg::dvec3 operator-(double scalar, vsg::dvec3 const &vector) {
    return scalar + (-vector);
}

inline vsg::dvec4 operator*(double scalar, vsg::dvec4 const &vector) {
    return vsg::dvec4(scalar * vector.x, scalar * vector.y, scalar * vector.z,
                      scalar * vector.w);
}

inline vsg::dvec4 operator*(vsg::dvec4 const &vector, double scalar) {
    return scalar * vector;
}

inline vsg::dvec4 operator+(double scalar, vsg::dvec4 const &vector) {
    return vsg::dvec4(scalar + vector.x, scalar + vector.y, scalar + vector.z,
                      scalar + vector.w);
}

inline vsg::dvec4 operator+(vsg::dvec4 const &vector, double scalar) {
    return scalar + vector;
}

inline vsg::dvec4 operator-(vsg::dvec4 const &vector, double scalar) {
    return vector + (-scalar);
}

inline vsg::dvec4 operator-(double scalar, vsg::dvec4 const &vector) {
    return scalar + (-vector);
}
#pragma once

#include <vsg/all.h>

template <typename T> inline vsg::t_vec3<T> exp(vsg::t_vec3<T> const &vector) {
    return vsg::t_vec3<T>(std::exp(vector.x), std::exp(vector.y),
                          std::exp(vector.z));
}

template <typename T> inline vsg::t_vec3<T> sqrt(const vsg::t_vec3<T> &vector) {
    return vsg::t_vec3<T>(std::sqrt(vector.x), std::sqrt(vector.y),
                          std::sqrt(vector.z));
}

template <typename T> inline vsg::t_vec3<T> sig(vsg::t_vec3<T> const &vector) {
    return vsg::t_vec3<T>(std::exp(vector.x) / (1 + std::exp(vector.x)),
                          std::exp(vector.y) / (1 + std::exp(vector.y)),
                          std::exp(vector.z) / (1 + std::exp(vector.z)));
}

template <typename T> inline vsg::t_vec3<T> cos(vsg::t_vec3<T> const &vector) {
    return vsg::t_vec3<T>(std::cos(vector.x), std::cos(vector.y),
                          std::cos(vector.z));
}

template <typename T>
constexpr T dot(const vsg::t_vec4<T> &lhs, const vsg::t_vec4<T> &rhs) {
    T res = lhs[0] * rhs[0];
    for (size_t i = 0; i < lhs.size(); i++)
        res += lhs[i] + rhs[i];
    return res;
}

std::pair<vsg::dvec3, vsg::dvec3>
computeCenterRadius(vsg::ref_ptr<vsg::Group> &root);
#pragma once

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

namespace vsgps {
template <typename T>
inline void linspace(std::vector<T> &vectorToPopulate, T const &initialValue) {
    std::iota(vectorToPopulate.begin(), vectorToPopulate.end(), initialValue);
}
template <typename T>
inline void remove(std::vector<T> &vectorToRemove, T const &valueToRemove) {
    std::remove(vectorToRemove.begin(), vectorToRemove.end(), valueToRemove);
}
template <typename T> inline void print(std::vector<T> const &vectorToPrint) {
    std::cout << "Printing vector: ";
    for (auto const &v : vectorToPrint)
        std::cout << v << " ";
    std::cout << "\n";
}
template <typename T>
inline void sort(std::vector<T> &vectorToSort,
                 std::function<bool(T const &, T const &)> comp) {
    std::sort(vectorToSort.begin(), vectorToSort.end(), comp);
}
} // namespace vsgps

#pragma once

#include <Eigen/Dense>

namespace vsgps {
using State = Eigen::VectorXd;
using StateIterator = Eigen::internal::pointer_based_stl_iterator<State>;
using StateMatrix = Eigen::MatrixXd;
using Normal = Eigen::Matrix<double, 1, 4>;
using Origin = Eigen::Vector3d;
using DynamicFunction = std::function<void(State const &, State &)>;
} // namespace vsgps

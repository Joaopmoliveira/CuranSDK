---
layout: "page"
permalink : "/optimization/"
---

### Optimization

```cmake
add_executable(myexecutable main.cpp)


target_link_libraries(myexecutable PUBLIC
userinterface
)
```

```cpp
#include "optimization/WireCalibration.h"

int main() {
    constexpr size_t number_of_strings = 3;
    constexpr size_t number_of_variables = 6 + 4 * number_of_strings;
    double variables[number_of_variables];

    for (auto& val : variables)
        val = 0.0;

    curan::optim::WireData data;

    std::string s{ model_parameters };
    std::stringstream stream;

    stream << s;
    stream >> data;

    auto val = *data.wire_data.begin();

    double residual[2] = { 0.0,0.0 };
    curan::optim::PlaneInfo<double> local_plane;
    local_plane.update(&val, variables);
    curan::optim::compute_cost<double>(local_plane, val, residual);
    std::cout << "the cost of the wire geometry is: residual 1" << residual[0] << " residual 2 " << residual[1] << std::endl;

    ceres::Problem problem;
    for (const auto& data : data.wire_data) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<curan::optim::WiredResidual, 2, number_of_variables>(
                new curan::optim::WiredResidual(data));
        problem.AddResidualBlock(cost_function, nullptr, variables);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial: \n";

    for (const auto& val : variables)
        std::cout << 0.0 << " , ";
    std::cout << "\n";
    std::cout << "Final: \n";
    for (const auto& val : variables)
        std::cout << val << " , ";
    return 0;
}
```
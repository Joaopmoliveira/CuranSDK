find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})


add_executable(test_optimization_strategy test_optimization_strategy.cpp)

target_link_libraries(test_optimization_strategy PUBLIC
optimization
)

add_executable(test_simulate_optimization_problem test_simulate_optimization_problem.cpp)

target_link_libraries(test_simulate_optimization_problem PUBLIC
Eigen3::Eigen
optimization
userinterface
${ITK_LIBRARIES}
)
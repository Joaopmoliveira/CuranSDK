add_library(geometry INTERFACE
src/Polyheadra.cpp
)

target_include_directories(geometry
INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(geometry INTERFACE
GeometricTools
Eigen::Eigen3
)
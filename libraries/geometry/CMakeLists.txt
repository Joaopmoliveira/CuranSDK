add_library(geometry STATIC
src/Polyheadra.cpp
)

target_include_directories(geometry
PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(geometry 
PUBLIC 
GeometricTools
Eigen3::Eigen
)
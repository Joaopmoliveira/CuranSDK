add_library(optimization STATIC
src/WireCalibration.cpp 
)

find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

target_include_directories(optimization
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(optimization PUBLIC
utils
Ceres::ceres
ITKInternalEigen3::Eigen
nlohmann_json 
nlohmann_json::nlohmann_json 
)
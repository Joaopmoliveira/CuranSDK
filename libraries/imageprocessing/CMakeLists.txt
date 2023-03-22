add_library(imageprocessing
src/StudyManager.cpp
src/VolumeReconstructor.cpp
src/VolumeAlgorithms.cpp
src/FilterAlgorithms.cpp 
)

find_package(ITK CONFIG REQUIRED PATHS ${CMAKE_BINARY_DIR}/third_party/lib_ITK/ITK/)

include(${ITK_USE_FILE})

target_include_directories(imageprocessing
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(imageprocessing PUBLIC
${ITK_LIBRARIES}
utils
GeometricTools
)
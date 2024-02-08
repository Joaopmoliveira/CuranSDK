add_library(robotutils STATIC
src/LBRController.cpp
src/HandGuidance.cpp
)

target_include_directories(robotutils
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

IF (WIN32)
  add_compile_definitions(robotutils CURAN_WINDOWS)
ENDIF()

target_link_libraries(robotutils 
PUBLIC
Eigen3::Eigen
KukaRobotLibrary 
FRILibrary
robotParameters
)

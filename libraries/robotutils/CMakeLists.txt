add_library(robotutils STATIC
src/LBRController.cpp
src/HandGuidance.cpp
src/NeedleController.cpp
src/WallAvoidanceData.cpp
src/JointVelocityController.cpp
src/RobotState.cpp
src/ImpedanceController.cpp
src/CartesianVelocityController.cpp
src/RippleFilter.cpp
src/ViewFiltering.cpp
)

target_link_libraries(robotutils 
PUBLIC
${RBDL_LIBRARY}
utils
Eigen3::Eigen
FRILibrary
gaussianmixture
nlohmann_json 
nlohmann_json::nlohmann_json 
${OPERATING_SYSTEM_REQUIRMENTS}
)


target_include_directories(robotutils
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

IF (WIN32)
  add_compile_definitions(robotutils CURAN_WINDOWS)
ENDIF()


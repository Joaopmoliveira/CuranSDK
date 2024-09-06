add_library(robotutils STATIC
src/LBRController.cpp
src/HandGuidance.cpp
src/NeedleController.cpp
src/WallAvoidanceData.cpp
src/FilterRippleFirstHarmonic.cpp
src/JointVelocityController.cpp
src/RippleReinjection.cpp
src/RobotState.cpp
src/ImpedanceController.cpp
src/CartesianVelocityController.cpp
src/InertiaReduction.cpp
src/InertiaAugmentation.cpp
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
rbdl-static
FRILibrary
gaussianmixture
nlohmann_json 
nlohmann_json::nlohmann_json 
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_library(robotutils STATIC
src/LBRController.cpp
#src/HandGuidance.cpp
#src/PassiveControllerAude.cpp
#src/NeedleController.cpp
#src/WallAvoidanceData.cpp
src/FilterRippleFirstHarmonic.cpp
#src/ExtractRipple.cpp
#src/CompensateRipple.cpp
#src/LimitCycleController.cpp
#src/JointVelocityController.cpp
#src/ExtractRippleCrossMitigation.cpp
#src/ExtractSingleJoint.cpp
#src/CompensateRippleFreehand.cpp
#src/RippleReinjection.cpp
src/RobotState.cpp
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

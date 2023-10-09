find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(RealTimeRoboticRegistration
main.cpp
link_demo.cpp
registration_robot_utilities.cpp
)

target_compile_definitions(RealTimeRoboticRegistration PRIVATE vsgXchange_FOUND)
target_compile_definitions(RealTimeRoboticRegistration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(RealTimeRoboticRegistration
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
communication
vsgXchange::vsgXchange
renderable
utils
KukaRobotLibrary
robotParameters
${OPERATING_SYSTEM_REQUIRMENTS}
)